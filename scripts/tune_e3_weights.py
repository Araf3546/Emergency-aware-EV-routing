"""Tune E3 ACO+PSO+BCO weights with Optuna's TPE sampler.

Each trial runs the real SUMO/TraCI E3 evaluator with a fresh algorithm state.
Four parameters in [0, 1] are mapped one-to-one onto the five weights by inverse-CDF
stick-breaking, so the weights sum to 1.0 and are uniformly distributed on the simplex.
That is audit B7 (2026-09-14). Until then five raw values were sampled and normalised,
which concentrated the prior near equal weights and left one dimension redundant.

    # once per seed and duration: the no-rerouting baseline every trial is measured against
    python scripts/tune_e3_weights.py --build-reference
    # first launch of a campaign (refuses a folder that already holds one)
    python scripts/tune_e3_weights.py --new --output-dir results/e3_tuning_campaign_<date>
    # continue it - after a pause, a normal stop, a crash or a power cut (never creates a study)
    python scripts/tune_e3_weights.py --resume --output-dir results/e3_tuning_campaign_<date>
    # optional: --stop-after N runs at most N trials in this invocation; creating a file named
    # STOP in the campaign folder stops it after the trial in progress

**Persistent, resumable campaign (2026-09-13).** The study lives in `<output-dir>/optuna_study.db`
(SQLite), which commits every finished trial: its weights, objective value, state and records.
The campaign's settings are fixed when it is created and checked on every resume, and so is a
fingerprint of the code and input files a trial runs. Optuna keeps the trial history but not the
sampler's random-number state, and a TPESampler rebuilt with the same seed on resume re-proposes
the first trials' weights (measured: trials 2 and 3 repeated trials 0 and 1). So the sampler is
rebuilt before every trial with seed = tuning seed + number of trials already evaluated. Each
proposal is then a function of the stored history alone, and a campaign run in chunks, or
interrupted and resumed, proposes exactly what one uninterrupted run would. A trial whose process
died stays RUNNING in the database (Optuna does not detect it). On resume it is marked FAIL with
reason "interrupted" and is not counted, so its weights are evaluated again as the next trial,
exactly as an uninterrupted run would have done; completed trials are never re-run. A file lock
refuses a second process on the same campaign, which also makes that marking safe. Every trial
has its own folder `trials/trial_<number>/`, never reused, with a `TRIAL_STATE.json` that mirrors
the database: only a trial marked COMPLETE there, and in the database, is a result.
`best_e3_weights.json` is written only when the target number of trials has been evaluated.

**What a trial runs (tuner smoke test, 2026-09-12).** A trial is the P3 treatment arm exactly as
`scripts/16_compare_baseline_vs_p3.py` runs it: `P3RuntimeCoordinator(emergency_layer=True)`
driven by `manager.update` then `coordinator.update_step` once per step, with the trial's weights
placed on that coordinator's router. Until this date a trial ran its own loop over
`algorithms/e3`, which had drifted from the production runtime in three ways measured by the smoke
test (results/tuner_smoke_2026-09-12/asis_1800): it never settled route outcomes, so pheromone and
PSO never learned - the pheromone table ended the run empty; it passed no V2V messages, so the
protocol term was the neutral constant; and it never told the router which roads an incident had
closed. In that run pheromone, PSO velocity and protocol were identical across the candidates of
all 344 contested decisions, so THREE of the five tuned weights (0.4 + 0.05 + 0.4 of the committed
set) could not change any decision, and the other two were tuned against an algorithm that is not
the one reported.

**What a trial is scored on (2026-09-13, decisions D1-D8, contract decision 13.17).** The formula,
its terms and its rationale live in `metrics/tuning_objective.py`. It is computed from SUMO's own
tripinfo (written with unfinished vehicles) and summary files, relative to a no-rerouting
baseline at the same seed and duration, so the baseline scores exactly 1.0. It replaced an
objective that polled TraCI and repeated four defects the 2026-09-07 metric fixes had removed
from reporting but never from here: waiting read as ~0 (0.0048 s against SUMO's 51.17 s),
teleports counted as a per-step maximum (1 against 8), congestion as a speed proxy over 15.0 m/s
(37.7% of the old objective), and failed emergency responses silently dropped.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

# The documented command is `python scripts/tune_e3_weights.py ...`, which puts scripts/ - not the
# repository root - on sys.path, so `algorithms` and the other packages could not be imported.
# Found by the 2026-09-12 smoke test; every other runner already does this.
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    import optuna
except ImportError as exc:  # pragma: no cover - exercised by the CLI environment
    raise SystemExit(
        "Optuna is required. Install dependencies with: "
        "python -m pip install -r requirements-tuning.txt"
    ) from exc

import traci

from algorithms import aco_pso
from algorithms.route_utils import get_route_stats
from emergency_manager import ROAD_PERMISSION_POLICY, get_emergency_manager
from evaluation_seeds import evaluation_seeds, frozen_demand_paths, route_choice_seed, sumo_seed_args, tuning_seed
from metrics.tuning_objective import (
    OBJECTIVE_WEIGHTS,
    check_reference,
    objective,
    run_metrics,
    scheduled_population,
)
from p3_hybrid_router import CONTRACT_TUNING_TRIALS, TUNER_EVALUATOR
from p3_simulation_config import AMBIENT_EMERGENCY_TYPES

WEIGHT_NAMES = ("pheromone", "heuristic", "protocol", "bco", "velocity")
# The sampler's parameters since audit B7 (2026-09-14): one fewer than the weights, because
# the score divides by the total weight, so only the weights' proportions matter.
STICK_PARAMS = tuple(f"stick_{i}" for i in range(1, len(WEIGHT_NAMES)))
SAMPLER_DESCRIPTION = ("optuna TPESampler(seed = tuning seed + trials already evaluated, n_startup_trials), "
                       "rebuilt before every trial; weights = inverse-CDF stick-breaking of "
                       f"{len(STICK_PARAMS)} uniform parameters in the order {', '.join(WEIGHT_NAMES)} "
                       "(uniform on the simplex, audit B7 2026-09-14)")
# Contract decision 13.15: weights are tuned on the held-out tuning seed, never on an
# evaluation seed. Until 2026-09-12 this default was the bare number 20260721 while SUMO still
# loaded demand/porto.rou.xml, whose speed factors are frozen for seed 20260722 - so every
# trial drove one seed's drivers under another seed's randomness. The seed now selects its own
# frozen demand file through evaluation_seeds.py, exactly as every other runner does.
DEFAULT_SEED = tuning_seed()
DEFAULT_TRIALS = CONTRACT_TUNING_TRIALS
# Optuna's own TPESampler default, which the in-memory tuner used implicitly.
DEFAULT_STARTUP_TRIALS = 10
SCENARIO_FILE = REPO_ROOT / "scenarios" / "emergency_scenarios.yaml"
STORAGE_FILE = "optuna_study.db"
STOP_FILE = "STOP"


def contract_tuning_duration() -> int:
    """`tuning.duration_seconds` from config/scenario.yaml - the contracted tuning window.

    This default used to be a hard-coded 3000 s while the config says 3600 and "do not tune at a
    shorter window". Incident 403 and the end-of-hour congestion then never reached a trial.
    """
    import yaml

    root = Path(__file__).resolve().parent.parent
    with open(root / "config" / "scenario.yaml", "r", encoding="utf-8") as handle:
        return int(yaml.safe_load(handle)["tuning"]["duration_seconds"])


def default_reference_path(seed: int, sim_time: int) -> Path:
    return REPO_ROOT / "results" / "tuning_reference" / f"seed_{int(seed)}_{int(sim_time)}s" / "reference.json"


class TrialRejected(RuntimeError):
    """A trial whose run logged an exception. Standing rule: such a run is not evidence."""


@dataclass(frozen=True)
class EvaluationResult:
    """One SUMO trial: its objective (None when no reference was given) and what fed it."""

    objective: Optional[float]
    relative: Dict[str, float]
    metrics: Dict[str, Any]
    reroutes: int
    # Cross-checks recorded by evaluate_weights (trial.json). Not part of the objective.
    checks: Dict[str, Any] = field(default_factory=dict, compare=False)

    def to_dict(self) -> Dict[str, Any]:
        row: Dict[str, Any] = {"objective": self.objective}
        row.update({f"rel_{name}": value for name, value in self.relative.items()})
        for key in ("travel_s", "time_loss_s", "incomplete_share", "teleports", "finished",
                    "unfinished_in_network", "never_inserted", "reported_waiting_s_completed",
                    "reported_travel_s_completed", "reported_time_loss_s_completed"):
            row[key] = self.metrics.get(key)
        for event_id, state in sorted(self.metrics.get("responses", {}).items()):
            row[f"response_{event_id}_s"] = state["value_s"]
            row[f"response_{event_id}_status"] = state["status"]
        row["reroutes"] = self.reroutes
        return row


def normalize_weights(raw_weights: Iterable[float]) -> Dict[str, float]:
    """Normalize five non-negative values so their sum is exactly one."""
    values = [float(value) for value in raw_weights]
    if len(values) != len(WEIGHT_NAMES):
        raise ValueError(f"Expected {len(WEIGHT_NAMES)} weights")
    if any(value < 0.0 for value in values):
        raise ValueError("Weights must be non-negative")
    total = sum(values)
    if total <= 0.0:
        raise ValueError("At least one weight must be greater than zero")
    normalized = {name: value / total for name, value in zip(WEIGHT_NAMES, values)}
    normalized[WEIGHT_NAMES[-1]] += 1.0 - sum(normalized.values())
    # Audit T8: the fix-up above can push a near-zero last weight a hair below zero.
    normalized[WEIGHT_NAMES[-1]] = max(0.0, normalized[WEIGHT_NAMES[-1]])
    return normalized


def simplex_weights(sticks: Iterable[float]) -> Dict[str, float]:
    """Map four numbers in [0, 1] one-to-one onto five weights that sum to one (audit B7).

    Inverse-CDF stick-breaking: the k-th weight takes the fraction 1 - (1 - u_k)^(1/(n-1-k))
    of what is left, the Beta(1, n-1-k) marginal of a uniform Dirichlet(1, ..., 1). With the u_k
    independent and uniform, the weights are uniform on the simplex. Every weight set has exactly
    one parameter point (away from the boundary), so TPE models no redundant scale and no
    permutation copies. Sorting four uniforms would give copies.
    """
    values = [min(1.0, max(0.0, float(u))) for u in sticks]
    if len(values) != len(STICK_PARAMS):
        raise ValueError(f"Expected {len(STICK_PARAMS)} stick parameters")
    remaining = 1.0
    weights: Dict[str, float] = {}
    for k, (name, u) in enumerate(zip(WEIGHT_NAMES[:-1], values)):
        weights[name] = remaining * (1.0 - (1.0 - u) ** (1.0 / (len(WEIGHT_NAMES) - 1 - k)))
        remaining -= weights[name]
    weights[WEIGHT_NAMES[-1]] = max(0.0, remaining)
    return weights


def trial_weights(params: Dict[str, float]) -> Optional[Dict[str, float]]:
    """The weights a stored trial used: stick parameters since B7, five raw values before it."""
    if all(name in params for name in STICK_PARAMS):
        return simplex_weights(params[name] for name in STICK_PARAMS)
    if all(name in params for name in WEIGHT_NAMES):
        return normalize_weights(params[name] for name in WEIGHT_NAMES)
    return None


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _route_files(seed_args: List[str]) -> List[Path]:
    """The route files the run really loads: the seed's frozen demand plus the other entries."""
    value = seed_args[seed_args.index("--route-files") + 1]
    return [Path(p) if Path(p).is_absolute() else Path.cwd() / p for p in value.split(",")]


def _sumo_command(sumo_cfg: Path, sim_time: int, out_dir: Path, seed_args: List[str]) -> List[str]:
    """Outputs go to the run's own folder. Emission and queue output, which simulation.sumocfg
    switches on and sends to results/baseline_smoke_test/, are switched off: they cost wall time,
    change no dynamics, and would otherwise be overwritten by every trial."""
    return [
        "sumo", "-c", str(sumo_cfg), "--no-step-log", "true", "--end", str(sim_time),
        "--tripinfo-output", str(out_dir / "tripinfo.xml"),
        "--tripinfo-output.write-unfinished", "true",
        "--summary-output", str(out_dir / "summary.xml"),
        "--error-log", str(out_dir / "errors.log"),
        "--message-log", str(out_dir / "messages.log"),
        "--emission-output", os.devnull,
        "--queue-output", os.devnull,
        *seed_args,
    ]


def build_reference(sumo_cfg: Path, seed: int, sim_time: int, out_dir: Path) -> Dict[str, Any]:
    """The no-rerouting baseline every trial is measured against (D2).

    The same run as the baseline arm of scripts/16: no coordinator, no rerouting; incidents close
    roads and responder arrivals are measured, because those are the scenario and the metric.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    seed_args = sumo_seed_args(seed, sumo_cfg)
    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    traci.start(_sumo_command(sumo_cfg, sim_time, out_dir, seed_args))
    try:
        while traci.simulation.getTime() < sim_time:
            traci.simulationStep()
            t = traci.simulation.getTime()
            manager.update(t)
            manager.record_responder_arrivals(
                t, vehicle_ids=traci.vehicle.getIDList(), arrived_ids=traci.simulation.getArrivedIDList())
    finally:
        try:
            traci.close()
        except Exception:
            pass
    _require_ambient_switch(manager)
    population = scheduled_population(_route_files(seed_args), sim_time)
    metrics = run_metrics(out_dir / "tripinfo.xml", out_dir / "summary.xml", population, sim_time,
                          manager.get_response_summary(), sumo_log=out_dir / "errors.log")
    reference = {
        "kind": "e3_tuning_reference (no-rerouting baseline)",
        "seed": int(seed), "sim_time": float(sim_time),
        "frozen_demand": str(frozen_demand_paths(seed)[0]),
        "frozen_demand_sha256": _sha256(frozen_demand_paths(seed)[0]),
        "scenario_sha256": _sha256(SCENARIO_FILE),
        "objective_weights": dict(OBJECTIVE_WEIGHTS),
        # PD-1 question 2: the baseline's own traffic depends on who may enter a closed road.
        "road_permissions": ROAD_PERMISSION_POLICY,
        "ambient_road_classes": dict(manager.ambient_road_classes),
        "metrics": metrics,
    }
    (out_dir / "reference.json").write_text(json.dumps(reference, indent=1), encoding="utf-8")
    return reference


def load_reference(path: Path, seed: int, sim_time: int) -> Dict[str, Any]:
    reference = json.loads(Path(path).read_text(encoding="utf-8"))
    check_reference(reference, seed, sim_time, _sha256(frozen_demand_paths(seed)[0]), _sha256(SCENARIO_FILE))
    # PD-1 question 2 changed the baseline's traffic (ambient vehicles now stop at closures), and
    # no file hash notices that, so a reference built before it is refused by name.
    if reference.get("road_permissions") != ROAD_PERMISSION_POLICY:
        raise ValueError(f"{path}: built under other road permissions "
                         f"({reference.get('road_permissions')!r}); rebuild it with --build-reference")
    return reference


def _require_ambient_switch(manager) -> None:
    """Every run must have switched all four ambient emergency types (PD-1 question 2)."""
    classes = dict(manager.ambient_road_classes)
    if sorted(classes) != sorted(AMBIENT_EMERGENCY_TYPES) or set(classes.values()) != {"authority"}:
        raise RuntimeError(f"ambient emergency types not switched to authority in this run: {classes}")


def evaluator_label(emergency_layer: bool = True) -> str:
    """The evaluator string recorded with a run. With the layer on it is the frozen TUNER_EVALUATOR."""
    return TUNER_EVALUATOR if emergency_layer else "p3_runtime.P3RuntimeCoordinator(emergency_layer=False)"


def evaluate_weights(
    weights: Dict[str, float],
    sumo_cfg: Path,
    sim_time: int,
    seed: int,
    trial_dir: Optional[Path] = None,
    reference: Optional[Dict[str, Any]] = None,
    emergency_layer: bool = True,
) -> EvaluationResult:
    """Run one deterministic E3 trial and calculate its tuning objective.

    The trial is the production P3 treatment arm (see the module docstring). SUMO's tripinfo
    (with unfinished vehicles), summary, error and message logs and a `trial.json` of cross-checks
    are written to `trial_dir` (a temporary folder when none is given). Without `reference` the
    run is measured but not scored. A run that logs any exception raises `TrialRejected`.

    `emergency_layer` (2026-09-17, decision B2): True, the default, is the frozen treatment arm
    exactly as tuned, validated and evaluated. False runs the same coordinator with its emergency
    layer off - the ablation gates of `P3RuntimeCoordinator` (protocol term neutral, no yielding,
    no responder corridor claim) - for the like-for-like comparison with the other routing methods.
    The setting is recorded in `trial.json`. The coordinator's blockage samples (audit C2) are
    written to `blockage_samples.json` beside it.
    """
    # Imported here so `normalize_weights` and `EvaluationResult` stay importable in tests
    # without pulling in the whole runtime.
    from p3_runtime import P3RuntimeCoordinator

    if not sumo_cfg.is_file():
        raise FileNotFoundError(f"SUMO configuration not found: {sumo_cfg}")
    unknown = sorted(set(weights) - set(WEIGHT_NAMES))
    missing = sorted(set(WEIGHT_NAMES) - set(weights))
    if unknown or missing:
        raise ValueError(f"weights: unknown {unknown}, missing {missing}")
    if trial_dir is None:
        trial_dir = Path(tempfile.mkdtemp(prefix="e3_trial_"))
    trial_dir.mkdir(parents=True, exist_ok=True)

    # Resolved before anything starts, so a missing or edited frozen file stops the trial here.
    seed_args = sumo_seed_args(seed, sumo_cfg)
    emergency_manager = get_emergency_manager()
    emergency_manager.load_canonical_scenario_file()
    emergency_manager.generate_emergencies()
    # A fresh coordinator is the per-trial reset: it clears the pheromone/PSO tables, the BCO
    # memory, the outcome tracker and the route-validity counters, and seeds route choice.
    coordinator = P3RuntimeCoordinator(emergency_layer=bool(emergency_layer),
                                       route_choice_seed=route_choice_seed(seed))
    weights_source_overridden = coordinator.e3.weights_source
    coordinator.e3.router.weights = {name: float(weights[name]) for name in WEIGHT_NAMES}
    exceptions: List[str] = []
    log_exception = coordinator._log_exception

    def counted_log_exception(context: str, exc: Exception) -> None:
        exceptions.append(f"{context}: {type(exc).__name__}: {exc}")
        log_exception(context, exc)

    coordinator._log_exception = counted_log_exception

    traci.start(_sumo_command(sumo_cfg, sim_time, trial_dir, seed_args))
    speed_factors: Dict[str, float] = {}
    try:
        while traci.simulation.getTime() < sim_time:
            traci.simulationStep()
            current_time = float(traci.simulation.getTime())
            emergency_manager.update(current_time)
            # The whole treatment arm: broadcasts, the reroute trigger, E3 decisions and
            # outcome learning, yielding, responder tracking. Nothing below changes a vehicle.
            coordinator.update_step(current_time)
            for vehicle_id in traci.simulation.getDepartedIDList():
                try:
                    speed_factors[vehicle_id] = traci.vehicle.getSpeedFactor(vehicle_id)
                except traci.TraCIException:
                    pass
    finally:
        try:
            traci.close()
        except (KeyError, traci.TraCIException, traci.exceptions.FatalTraCIError):
            pass

    _require_ambient_switch(emergency_manager)
    population = scheduled_population(_route_files(seed_args), sim_time)
    metrics = run_metrics(trial_dir / "tripinfo.xml", trial_dir / "summary.xml", population, sim_time,
                          emergency_manager.get_response_summary(), sumo_log=trial_dir / "errors.log")
    scored = objective(metrics, reference["metrics"]) if reference is not None else None

    frozen = json.loads(frozen_demand_paths(seed)[1].read_text(encoding="utf-8"))["speed_factors"]
    compared = [v for v in frozen if v in speed_factors]
    checks = {
        "evaluator": evaluator_label(emergency_layer),
        "emergency_layer": bool(emergency_layer),
        "road_permissions": ROAD_PERMISSION_POLICY,
        "ambient_road_classes": dict(emergency_manager.ambient_road_classes),
        "seed": seed,
        "frozen_demand": str(frozen_demand_paths(seed)[0]),
        "sim_time": sim_time,
        "weights_requested": {name: float(weights[name]) for name in WEIGHT_NAMES},
        # Read back after the run: proves the router scored with the trial's weights throughout.
        "weights_used": dict(coordinator.e3.router.weights),
        "weights_source_overridden": weights_source_overridden,
        "exceptions_logged": exceptions,
        "coordinator": {
            "reroute_decisions": coordinator.metrics.reroute_decisions,
            "reroute_executed": coordinator.metrics.reroute_executed,
            "candidate_edges_scored": coordinator.metrics.candidate_edges_scored,
            "yield_actions": coordinator.metrics.yield_actions_performed,
            "emergency_reroutes": coordinator.metrics.emergency_reroutes,
        },
        "learning": {
            "pheromone_table_entries": len(aco_pso.pheromone),
            "outcomes": dict(coordinator.e3.outcomes.counters),
            # A2, "mean protected" (2026-09-14): every measured outcome updates PSO fitness, so the
            # first equals outcomes_reported; the second leaves out settled_by_new_decision.
            "outcomes_applied_to_pso": aco_pso.outcomes_recorded,
            "outcomes_in_running_mean": aco_pso.outcomes_in_mean,
            "quality_mean_final": aco_pso.quality_mean,
        },
        "route_stats": {k: v for k, v in get_route_stats().items() if isinstance(v, (int, float))},
        "metrics": metrics,
        "objective": scored,
        # Counters that must agree before any number is quoted (standing rule).
        "finished_equals_summary_arrived": metrics["finished"] == metrics["summary_arrived"],
        "no_tripinfo_record_outside_population": metrics["records_outside_population"] == 0,
        "pairing": {
            "ordinary_vehicles_compared": len(compared),
            "speed_factor_differences": sum(1 for v in compared if speed_factors[v] != float(frozen[v])),
        },
    }
    (trial_dir / "trial.json").write_text(json.dumps(checks, indent=1, default=str), encoding="utf-8")
    (trial_dir / "blockage_samples.json").write_text(
        json.dumps(coordinator.blockage_samples, default=str), encoding="utf-8")
    if exceptions:
        raise TrialRejected(f"{len(exceptions)} exception(s) logged, first: {exceptions[0]}")
    if checks["weights_used"] != checks["weights_requested"]:
        raise TrialRejected("router weights changed during the trial")
    if checks["pairing"]["speed_factor_differences"]:
        raise TrialRejected("speed factors differ from the seed's frozen demand file")
    if not (checks["finished_equals_summary_arrived"] and checks["no_tripinfo_record_outside_population"]):
        raise TrialRejected("tripinfo and summary disagree on who finished")

    return EvaluationResult(
        objective=scored["J"] if scored else None,
        relative=scored["relative"] if scored else {},
        metrics=metrics,
        reroutes=coordinator.metrics.reroute_executed,
        checks=checks,
    )


COMPARATOR_EVALUATOR = "algorithms.comparator_routing.RoutingLoop"


def evaluate_comparator(
    method: str,
    sumo_cfg: Path,
    sim_time: int,
    seed: int,
    trial_dir: Optional[Path] = None,
    reference: Optional[Dict[str, Any]] = None,
) -> EvaluationResult:
    """Run one deterministic comparison-method trial, measured exactly as `evaluate_weights` measures E3.

    Decision B5 (2026-09-17): every method of the V1 comparison is measured with the same code. So this uses the
    same SUMO command (`_sumo_command`, tripinfo with unfinished vehicles), the same scheduled population, the same
    censored `run_metrics`, the same `objective` against the seed's own no-rerouting reference, the same
    road-permission check and the same speed-factor pairing check as `evaluate_weights`, and writes a `trial.json`
    with the same cross-check fields. The routing itself is `algorithms.comparator_routing.RoutingLoop`, the loop
    `metrics/metrics_logger.py` runs, in the same order within a step. Completed-only averages are added as a
    supplementary block and are never part of J.
    """
    from algorithms.comparator_routing import COMPARATORS, RoutingLoop
    from algorithms.route_utils import reset_route_stats, seed_route_choice
    from metrics.trip_metrics import build_report
    from p3_simulation_config import EMERGENCY_VEHICLE_TYPES

    if method not in COMPARATORS:
        raise ValueError(f"unknown comparison method {method!r}; expected one of {COMPARATORS}")
    if not sumo_cfg.is_file():
        raise FileNotFoundError(f"SUMO configuration not found: {sumo_cfg}")
    if trial_dir is None:
        trial_dir = Path(tempfile.mkdtemp(prefix=f"comparison_{method}_"))
    trial_dir.mkdir(parents=True, exist_ok=True)

    # The same order as metrics_logger: seed arguments, route-choice seed, fresh route counters, SUMO,
    # the routing state, then the incidents.
    seed_args = sumo_seed_args(seed, sumo_cfg)
    route_seed = seed_route_choice(route_choice_seed(seed))
    reset_route_stats()
    traci.start(_sumo_command(sumo_cfg, sim_time, trial_dir, seed_args))
    speed_factors: Dict[str, float] = {}
    try:
        loop = RoutingLoop(method)
        emergency_manager = get_emergency_manager()
        emergency_manager.load_canonical_scenario_file()
        emergency_manager.generate_emergencies()
        while traci.simulation.getTime() < sim_time:
            traci.simulationStep()
            current_time = float(traci.simulation.getTime())
            emergency_manager.update(current_time)
            loop.before_step(current_time, emergency_manager.blocked_edges)
            emergency_manager.record_responder_arrivals(
                current_time, vehicle_ids=traci.vehicle.getIDList(),
                arrived_ids=traci.simulation.getArrivedIDList())
            loop.route_vehicles(current_time)
            for vehicle_id in traci.simulation.getDepartedIDList():
                try:
                    speed_factors[vehicle_id] = traci.vehicle.getSpeedFactor(vehicle_id)
                except traci.TraCIException:
                    pass
    finally:
        try:
            traci.close()
        except (KeyError, traci.TraCIException, traci.exceptions.FatalTraCIError):
            pass

    _require_ambient_switch(emergency_manager)
    population = scheduled_population(_route_files(seed_args), sim_time)
    metrics = run_metrics(trial_dir / "tripinfo.xml", trial_dir / "summary.xml", population, sim_time,
                          emergency_manager.get_response_summary(), sumo_log=trial_dir / "errors.log")
    scored = objective(metrics, reference["metrics"]) if reference is not None else None
    completed_only = build_report(trial_dir / "tripinfo.xml", trial_dir / "summary.xml",
                                  emergency_types=EMERGENCY_VEHICLE_TYPES,
                                  applied_reroutes=loop.reroutes_applied, finished_only=True)

    frozen = json.loads(frozen_demand_paths(seed)[1].read_text(encoding="utf-8"))["speed_factors"]
    compared = [v for v in frozen if v in speed_factors]
    route_stats = get_route_stats()
    checks = {
        "evaluator": f"{COMPARATOR_EVALUATOR}({method!r})",
        "method": method,
        "emergency_layer": None,
        "road_permissions": ROAD_PERMISSION_POLICY,
        "ambient_road_classes": dict(emergency_manager.ambient_road_classes),
        "seed": seed,
        "frozen_demand": str(frozen_demand_paths(seed)[0]),
        "sim_time": sim_time,
        "route_choice_seed": route_seed,
        # The comparison loop swallows per-vehicle TraCI errors, as metrics_logger always did; any other error
        # ends the process with a traceback, which invalidates the run.
        "exceptions_logged": [],
        "routing": loop.counters(),
        "route_stats": {k: v for k, v in route_stats.items() if isinstance(v, (int, float))},
        "route_build_failure_reasons": route_stats.get("route_build_failure_reasons", {}),
        "metrics": metrics,
        "objective": scored,
        "completed_only_supplementary": completed_only,
        "finished_equals_summary_arrived": metrics["finished"] == metrics["summary_arrived"],
        "no_tripinfo_record_outside_population": metrics["records_outside_population"] == 0,
        "pairing": {
            "ordinary_vehicles_compared": len(compared),
            "speed_factor_differences": sum(1 for v in compared if speed_factors[v] != float(frozen[v])),
        },
    }
    (trial_dir / "trial.json").write_text(json.dumps(checks, indent=1, default=str), encoding="utf-8")
    if checks["pairing"]["speed_factor_differences"]:
        raise TrialRejected("speed factors differ from the seed's frozen demand file")
    if not (checks["finished_equals_summary_arrived"] and checks["no_tripinfo_record_outside_population"]):
        raise TrialRejected("tripinfo and summary disagree on who finished")

    return EvaluationResult(
        objective=scored["J"] if scored else None,
        relative=scored["relative"] if scored else {},
        metrics=metrics,
        reroutes=loop.reroutes_applied,
        checks=checks,
    )


# ============================================================ the campaign (2026-09-13)
# Persistent SQLite study, deterministic per-trial sampler, crash-safe resume. See the module
# docstring for the rules; design choices.md ("PERSISTENT, RESUMABLE TUNING CAMPAIGN") for why.

TrialState = optuna.trial.TrialState


def storage_url(output_dir: Path) -> str:
    return f"sqlite:///{(Path(output_dir).resolve() / STORAGE_FILE).as_posix()}"


def trial_folder(output_dir: Path, number: int) -> Path:
    return Path(output_dir) / "trials" / f"trial_{int(number):03d}"


def _utc_now() -> str:
    from datetime import datetime, timezone

    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _write_atomic(path: Path, text: str) -> None:
    """Write, flush to disk, then rename over the target: a power cut leaves the old file or the
    new one, never half of one."""
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8", newline="") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def _log_event(output_dir: Path, event: str, **fields: Any) -> None:
    """Append-only campaign log, one JSON line per start, stop, trial and repair."""
    with open(Path(output_dir) / "campaign_events.jsonl", "a", encoding="utf-8") as handle:
        handle.write(json.dumps({"utc": _utc_now(), "event": event, **fields}, default=str) + "\n")
        handle.flush()
        os.fsync(handle.fileno())


class CampaignLock:
    """An OS file lock held for the life of the process. A second process on the same campaign is
    refused; a crash or power cut releases the lock with the process. While it is held, a trial the
    database still shows as RUNNING belongs to a process that no longer exists."""

    def __init__(self, output_dir: Path) -> None:
        self.path = Path(output_dir) / "campaign.lock"
        self.handle = None

    def __enter__(self) -> "CampaignLock":
        self.handle = open(self.path, "a+")
        try:
            if os.name == "nt":
                import msvcrt

                self.handle.seek(0)
                msvcrt.locking(self.handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(self.handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.handle.close()
            raise SystemExit(f"another tuner process is running this campaign ({self.path}); "
                             "let it finish or stop it before resuming")
        return self

    def __exit__(self, *exc: Any) -> None:
        try:
            if os.name == "nt":
                import msvcrt

                self.handle.seek(0)
                msvcrt.locking(self.handle.fileno(), msvcrt.LK_UNLCK, 1)
        except OSError:
            pass
        self.handle.close()


def _net_file(sumo_cfg: Path) -> Path:
    import xml.etree.ElementTree as ET

    value = ET.parse(sumo_cfg).getroot().find(".//net-file").get("value")
    return (Path(sumo_cfg).resolve().parent / value).resolve()


def evaluation_fingerprint(seed: int, sumo_cfg: Path) -> Dict[str, str]:
    """SHA-256 of every file a trial's result depends on: each project module this process loads
    to run a trial (tests and diagnostics excluded), the configuration, the incident scenario, the
    network and every route file the run loads. A resume compares it with the stored one."""
    import p3_runtime  # noqa: F401  - loads every module a trial runs

    files = set()
    for module in list(sys.modules.values()):
        path = getattr(module, "__file__", None)
        if not path or not str(path).endswith(".py"):
            continue
        resolved = Path(path).resolve()
        if REPO_ROOT in resolved.parents:
            parts = resolved.relative_to(REPO_ROOT).parts
            if "tests" not in parts and "diagnostics" not in parts:
                files.add(resolved)
    files |= {(REPO_ROOT / "config" / "scenario.yaml").resolve(), Path(sumo_cfg).resolve(),
              SCENARIO_FILE.resolve(), _net_file(Path(sumo_cfg))}
    files |= {p.resolve() for p in _route_files(sumo_seed_args(seed, Path(sumo_cfg)))}
    return {p.relative_to(REPO_ROOT).as_posix(): _sha256(p) for p in sorted(files)}


def _digest(mapping: Dict[str, str]) -> str:
    return hashlib.sha256(json.dumps(mapping, sort_keys=True).encode("utf-8")).hexdigest()[:16]


def campaign_settings(seed: int, sim_time: int, trials: int, startup_trials: int, sumo_cfg: Path,
                      reference_path: Path) -> Dict[str, Any]:
    """Everything that defines a campaign. Fixed at --new, stored in the study, checked on resume."""
    return {
        "seed": int(seed), "sim_time": int(sim_time), "trials_target": int(trials),
        "startup_trials": int(startup_trials),
        "sampler": SAMPLER_DESCRIPTION,
        "sumo_cfg": str(sumo_cfg), "reference": str(Path(reference_path).resolve()),
        "reference_sha256": _sha256(reference_path),
        "frozen_demand": str(frozen_demand_paths(seed)[0]),
        "frozen_demand_sha256": _sha256(frozen_demand_paths(seed)[0]),
        "scenario_sha256": _sha256(SCENARIO_FILE),
        "objective_weights": dict(OBJECTIVE_WEIGHTS),
        "evaluator": TUNER_EVALUATOR,
        "road_permissions": ROAD_PERMISSION_POLICY,
        "code_fingerprint": evaluation_fingerprint(seed, sumo_cfg),
    }


def evaluated_trials(study: "optuna.Study") -> list:
    """Trials that count toward the target: COMPLETE, or FAIL because the run was rejected (a
    deterministic outcome of its weights, as `catch=(TrialRejected,)` counted it before). A trial
    that was interrupted or hit an error was not evaluated; its weights come round again."""
    return [t for t in study.trials if t.state == TrialState.COMPLETE
            or (t.state == TrialState.FAIL and t.user_attrs.get("fail_reason") == "rejected")]


def _write_trial_state(output_dir: Path, trial: "optuna.trial.FrozenTrial") -> None:
    folder = trial_folder(output_dir, trial.number)
    if not folder.is_dir():
        return        # the process died before the trial created its folder
    _write_atomic(folder / "TRIAL_STATE.json", json.dumps({
        "trial": trial.number,
        "state": trial.state.name,
        "counts_as_a_result": trial.state == TrialState.COMPLETE,
        "value": trial.value,
        "fail_reason": trial.user_attrs.get("fail_reason"),
        "fail_detail": trial.user_attrs.get("fail_detail"),
        "note": "mirrors optuna_study.db, which is authoritative; only a COMPLETE trial is a result",
    }, indent=1))


def reconcile_interrupted(study: "optuna.Study", output_dir: Path) -> List[int]:
    """Mark every trial left RUNNING by a dead process as FAIL ("interrupted"). Call only while
    holding the CampaignLock. Also re-mirrors every trial's state into its folder, repairing a
    marker lost to a crash between the database commit and the marker write."""
    repaired = []
    for trial in study.trials:
        if trial.state == TrialState.RUNNING:
            study._storage.set_trial_user_attr(trial._trial_id, "fail_reason", "interrupted")
            study._storage.set_trial_user_attr(
                trial._trial_id, "fail_detail",
                f"its process ended before the trial finished (stopped, crashed or power lost); "
                f"marked on resume at {_utc_now()}")
            study.tell(trial.number, state=TrialState.FAIL)
            repaired.append(trial.number)
            _log_event(output_dir, "trial_marked_interrupted", trial=trial.number)
    for trial in study.trials:
        _write_trial_state(output_dir, trial)
    return repaired


def _objective(trial: "optuna.Trial", args: argparse.Namespace) -> float:
    raw = [trial.suggest_float(name, 0.0, 1.0) for name in STICK_PARAMS]
    weights = simplex_weights(raw)
    trial.set_user_attr("weights", weights)
    trial_dir = trial_folder(args.output_dir, trial.number)
    # Trial numbers are never reused, so this only fires if a folder was copied in by hand.
    if trial_dir.exists() and any(trial_dir.iterdir()):
        raise RuntimeError(f"{trial_dir} already holds files; a trial folder is never reused")
    trial.set_user_attr("trial_dir", str(trial_dir))
    result = evaluate_weights(weights, args.sumo_cfg, args.sim_time, args.seed,
                              trial_dir=trial_dir, reference=args.reference_data)
    trial.set_user_attr("metrics", result.to_dict())
    trial.set_user_attr("coordinator", result.checks.get("coordinator", {}))
    return result.objective


def _trial_rows(study: "optuna.Study") -> list:
    rows = []
    counted = {t.number for t in evaluated_trials(study)}
    for trial in study.trials:
        row = {"trial": trial.number, "state": trial.state.name,
               "counts_toward_target": trial.number in counted,
               "fail_reason": trial.user_attrs.get("fail_reason"), "value": trial.value,
               "evaluation_index": trial.user_attrs.get("evaluation_index"),
               "sampler_seed": trial.user_attrs.get("sampler_seed"),
               "trial_dir": trial.user_attrs.get("trial_dir")}
        weights = trial_weights(trial.params)
        if weights is not None:
            row.update(weights)
        metrics = dict(trial.user_attrs.get("metrics", {}))
        metrics.pop("objective", None)
        row.update(metrics)
        rows.append(row)
    return rows


def write_campaign_outputs(study: "optuna.Study", args: argparse.Namespace,
                           settings: Dict[str, Any]) -> Dict[str, Any]:
    """Refresh e3_trials.csv and progress.json from the database; write best_e3_weights.json only
    once the target has been evaluated. Every file is written atomically."""
    import io

    rows = _trial_rows(study)
    fieldnames = ["trial", "state", "counts_toward_target", "fail_reason", "value", *WEIGHT_NAMES]
    for row in rows:
        fieldnames += [k for k in row if k not in fieldnames]
    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
    _write_atomic(args.output_dir / "e3_trials.csv", buffer.getvalue())

    evaluated = evaluated_trials(study)
    counted = {t.number for t in evaluated}
    complete = [t for t in study.trials if t.state == TrialState.COMPLETE]
    not_counted = [t for t in study.trials if t.state == TrialState.FAIL and t.number not in counted]
    best = min(complete, key=lambda t: t.value) if complete else None
    finished = len(evaluated) >= settings["trials_target"]
    progress = {
        "study_name": study.study_name, "storage": storage_url(args.output_dir),
        "trials_target": settings["trials_target"], "trials_evaluated": len(evaluated),
        "trials_complete": len(complete),
        "trials_rejected": sum(1 for t in evaluated if t.state == TrialState.FAIL),
        "trials_not_counted (interrupted or error; their weights come round again)":
            {t.number: t.user_attrs.get("fail_reason") for t in not_counted},
        "campaign_complete": finished,
        "provisional_best": None if best is None else {
            "trial": best.number, "value": best.value,
            "weights": trial_weights(best.params)},
        "note": "PROVISIONAL until campaign_complete; never load these weights. The tuned weight set "
                "is best_e3_weights.json, written only when the target is reached.",
        "resume_command": f"python scripts/tune_e3_weights.py --resume --output-dir {args.output_dir}",
        "updated_utc": _utc_now(),
    }
    _write_atomic(args.output_dir / "progress.json", json.dumps(progress, indent=1))
    if not finished or best is None:
        return progress

    summary = {
        "study_name": study.study_name,
        "direction": "minimize",
        "seed": settings["seed"],
        "frozen_demand": settings["frozen_demand"],
        "evaluator": settings["evaluator"],
        "objective": "metrics/tuning_objective.py (D1-D8, contract 13.17)",
        "objective_weights": settings["objective_weights"],
        "road_permissions": settings["road_permissions"],
        "reference": settings["reference"],
        "contract_tuning_duration": contract_tuning_duration(),
        "storage": storage_url(args.output_dir),
        "sampler": settings["sampler"],
        "startup_trials": settings["startup_trials"],
        "code_fingerprint_digest": _digest(settings["code_fingerprint"]),
        "campaign_complete": True,
        "trials_target": settings["trials_target"],
        "trials_evaluated": len(evaluated),
        "trials_complete": len(complete),
        "failed_trials": sum(1 for t in study.trials if t.state == TrialState.FAIL),
        "sumo_cfg": settings["sumo_cfg"],
        "sim_time": settings["sim_time"],
        "trials": settings["trials_target"],
        "best_value": best.value,
        "best_trial_number": best.number,
        "best_weights": trial_weights(best.params),
        "best_metrics": best.user_attrs.get("metrics", {}),
    }
    _write_atomic(args.output_dir / "best_e3_weights.json", json.dumps(summary, indent=2))
    return progress


def run_campaign(study: "optuna.Study", args: argparse.Namespace, settings: Dict[str, Any]) -> str:
    """Evaluate trials until the target is reached, --stop-after is used up, or a STOP file appears.
    Returns why it stopped. Each trial is committed to the database before the next one starts."""
    ran = 0
    digest = _digest(settings["code_fingerprint"])
    while True:
        done = len(evaluated_trials(study))
        if done >= settings["trials_target"]:
            return "target reached"
        if args.stop_after is not None and ran >= args.stop_after:
            return f"--stop-after {args.stop_after} reached"
        if (args.output_dir / STOP_FILE).exists():
            return "STOP file found"
        study.sampler = optuna.samplers.TPESampler(seed=settings["seed"] + done,
                                                   n_startup_trials=settings["startup_trials"])
        trial = study.ask()
        trial.set_user_attr("evaluation_index", done)
        trial.set_user_attr("sampler_seed", settings["seed"] + done)
        trial.set_user_attr("code_fingerprint_digest", digest)
        _log_event(args.output_dir, "trial_started", trial=trial.number, evaluation_index=done)
        print(f"[campaign] trial {trial.number} started (evaluation {done + 1} of {settings['trials_target']})",
              flush=True)
        try:
            value = _objective(trial, args)
        except TrialRejected as exc:
            trial.set_user_attr("fail_reason", "rejected")
            trial.set_user_attr("fail_detail", str(exc))
            study.tell(trial, state=TrialState.FAIL)
            event, value = "trial_rejected", None
        except BaseException as exc:                    # KeyboardInterrupt included
            reason = "interrupted" if isinstance(exc, KeyboardInterrupt) else "error"
            trial.set_user_attr("fail_reason", reason)
            trial.set_user_attr("fail_detail", f"{type(exc).__name__}: {exc}")
            study.tell(trial, state=TrialState.FAIL)
            _write_trial_state(args.output_dir, study._storage.get_trial(trial._trial_id))
            _log_event(args.output_dir, f"trial_{reason}", trial=trial.number, detail=repr(exc))
            write_campaign_outputs(study, args, settings)
            raise
        else:
            study.tell(trial, value)
            event = "trial_complete"
        ran += 1
        _write_trial_state(args.output_dir, study._storage.get_trial(trial._trial_id))
        _log_event(args.output_dir, event, trial=trial.number, value=value)
        write_campaign_outputs(study, args, settings)
        print(f"[campaign] trial {trial.number}: {event}, value {value}", flush=True)


def _open_campaign(args: argparse.Namespace, parser: argparse.ArgumentParser):
    """--new creates the study (refusing a folder that already holds one); --resume loads it
    (refusing to create one). Returns (study, settings) with args filled from the settings."""
    url = storage_url(args.output_dir)
    db = args.output_dir / STORAGE_FILE
    if args.new:
        if db.exists():
            parser.error(f"{args.output_dir} already holds a campaign; continue it with --resume, "
                         "or choose a new --output-dir")
        trials_dir = args.output_dir / "trials"
        if trials_dir.exists() and any(trials_dir.iterdir()):
            parser.error(f"{trials_dir} already holds trial folders; choose a new --output-dir")
        args.sumo_cfg = args.sumo_cfg or Path("simulation.sumocfg")
        args.sim_time = args.sim_time if args.sim_time is not None else contract_tuning_duration()
        args.trials = args.trials if args.trials is not None else DEFAULT_TRIALS
        args.seed = args.seed if args.seed is not None else DEFAULT_SEED
        args.startup_trials = args.startup_trials if args.startup_trials is not None else DEFAULT_STARTUP_TRIALS
        if args.sim_time <= 0 or args.trials <= 0 or args.startup_trials < 0:
            parser.error("--sim-time and --trials must be positive, --startup-trials not negative")
        # Decision 13.15: tuning on a seed the evaluation reports would be tuning on the test set.
        if args.seed in evaluation_seeds() and not args.allow_evaluation_seed:
            parser.error(f"seed {args.seed} is an evaluation seed; tune on the held-out seed "
                         f"{tuning_seed()}, or pass --allow-evaluation-seed deliberately")
        reference_path = args.reference or default_reference_path(args.seed, args.sim_time)
        if not reference_path.is_file():
            parser.error(f"no baseline reference at {reference_path}; build it first with --build-reference")
        load_reference(reference_path, args.seed, args.sim_time)
        settings = campaign_settings(args.seed, args.sim_time, args.trials, args.startup_trials,
                                     args.sumo_cfg, reference_path)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        study = optuna.create_study(storage=url, study_name=args.study_name or "e3_weight_tuning",
                                    direction="minimize", load_if_exists=False)
        study.set_user_attr("campaign", settings)
        study.set_user_attr("created_utc", _utc_now())
        _write_atomic(args.output_dir / "campaign.json", json.dumps(
            {"study_name": study.study_name, "storage": url, **settings}, indent=1))
    else:
        if not db.exists():
            parser.error(f"no campaign in {args.output_dir} (no {STORAGE_FILE}); start one with --new")
        names = optuna.get_all_study_names(url)
        name = args.study_name or (names[0] if len(names) == 1 else None)
        if name not in names:
            parser.error(f"study {name!r} not in {db}; it holds {names}")
        study = optuna.load_study(study_name=name, storage=url)
        settings = dict(study.user_attrs["campaign"])
        given = {"seed": args.seed, "sim_time": args.sim_time, "trials_target": args.trials,
                 "startup_trials": args.startup_trials,
                 "sumo_cfg": None if args.sumo_cfg is None else str(args.sumo_cfg),
                 "reference": None if args.reference is None else str(args.reference.resolve())}
        clash = {k: (v, settings[k]) for k, v in given.items() if v is not None and v != settings[k]}
        if clash:
            parser.error(f"a resume continues the campaign as it was created; these differ "
                         f"(given, stored): {clash}")
        args.seed, args.sim_time, args.trials = settings["seed"], settings["sim_time"], settings["trials_target"]
        args.startup_trials, args.sumo_cfg = settings["startup_trials"], Path(settings["sumo_cfg"])
        reference_path = Path(settings["reference"])
        load_reference(reference_path, args.seed, args.sim_time)
        current = campaign_settings(args.seed, args.sim_time, args.trials, args.startup_trials,
                                    args.sumo_cfg, reference_path)
        changed = {k: (settings[k], current[k]) for k in settings
                   if k != "code_fingerprint" and settings[k] != current[k]}
        if changed:
            parser.error(f"the campaign's inputs changed since it was created; it cannot continue: {changed}")
        old, new = settings["code_fingerprint"], current["code_fingerprint"]
        files = sorted(k for k in set(old) | set(new) if old.get(k) != new.get(k))
        if files:
            if not args.accept_code_change:
                parser.error("code or input files a trial runs have changed since this campaign was "
                             f"created or last accepted: {files}. Trials run on different code are not "
                             "comparable. Restore them, or pass --accept-code-change deliberately")
            history = list(study.user_attrs.get("code_changes", []))
            history.append({"utc": _utc_now(), "files": files, "previous_digest": _digest(old)})
            study.set_user_attr("code_changes", history)
            settings["code_fingerprint"] = new
            study.set_user_attr("campaign", settings)
            _log_event(args.output_dir, "code_change_accepted", files=files)
    args.reference_data = load_reference(Path(settings["reference"]), args.seed, args.sim_time)
    return study, settings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--new", action="store_true", help="start a campaign in --output-dir")
    mode.add_argument("--resume", action="store_true", help="continue the campaign in --output-dir")
    mode.add_argument("--build-reference", action="store_true",
                      help="run the no-rerouting baseline for this seed and duration, write it, and stop")
    parser.add_argument("--output-dir", type=Path, default=None, help="the campaign folder")
    parser.add_argument("--trials", type=int, default=None,
                        help=f"trials the campaign evaluates in total (default {DEFAULT_TRIALS}; fixed at --new)")
    parser.add_argument("--stop-after", type=int, default=None,
                        help="run at most this many trials in this invocation, then stop cleanly")
    parser.add_argument("--sumo-cfg", type=Path, default=None, help="default simulation.sumocfg")
    parser.add_argument("--sim-time", type=int, default=None,
                        help="default: tuning.duration_seconds in config/scenario.yaml (3600)")
    parser.add_argument("--seed", type=int, default=None, help=f"default {DEFAULT_SEED}")
    parser.add_argument("--startup-trials", type=int, default=None,
                        help=f"random trials before TPE models the history (default {DEFAULT_STARTUP_TRIALS})")
    parser.add_argument("--study-name", default=None, help="default e3_weight_tuning")
    parser.add_argument("--reference", type=Path, default=None,
                        help="baseline reference.json (default: results/tuning_reference/seed_<seed>_<T>s/)")
    parser.add_argument("--allow-evaluation-seed", action="store_true",
                        help="tune on an evaluation seed anyway (decision 13.15 says do not)")
    parser.add_argument("--accept-code-change", action="store_true",
                        help="resume although code or input files changed (recorded in the study)")
    args = parser.parse_args()

    if args.build_reference:
        seed = args.seed if args.seed is not None else DEFAULT_SEED
        sim_time = args.sim_time if args.sim_time is not None else contract_tuning_duration()
        reference_path = args.reference or default_reference_path(seed, sim_time)
        if reference_path.exists():
            parser.error(f"{reference_path} exists; move it aside first (a reference is evidence)")
        reference = build_reference(args.sumo_cfg or Path("simulation.sumocfg"), seed, sim_time,
                                    reference_path.parent)
        print(json.dumps({k: v for k, v in reference["metrics"].items()}, indent=1))
        print(f"reference written: {reference_path}")
        return 0
    if not (args.new or args.resume):
        parser.error("choose --new (first launch of a campaign) or --resume (continue one)")
    if args.output_dir is None:
        parser.error("--output-dir is required: it is the campaign folder")
    if args.stop_after is not None and args.stop_after <= 0:
        parser.error("--stop-after must be positive")
    args.output_dir = Path(args.output_dir)
    if args.resume and not (args.output_dir / STORAGE_FILE).exists():
        parser.error(f"no campaign in {args.output_dir} (no {STORAGE_FILE}); start one with --new")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    with CampaignLock(args.output_dir):
        study, settings = _open_campaign(args, parser)
        repaired = reconcile_interrupted(study, args.output_dir)
        stop_file = args.output_dir / STOP_FILE
        if args.resume and stop_file.exists():
            stop_file.unlink()
            print(f"[campaign] removed {stop_file} left from the last stop")
        _log_event(args.output_dir, "created" if args.new else "resumed", pid=os.getpid(),
                   evaluated=len(evaluated_trials(study)), target=settings["trials_target"],
                   marked_interrupted=repaired, stop_after=args.stop_after)
        if repaired:
            print(f"[campaign] trial(s) {repaired} were interrupted; marked FAIL, not counted, their "
                  "weights are evaluated again")
        write_campaign_outputs(study, args, settings)
        reason = run_campaign(study, args, settings)
        progress = write_campaign_outputs(study, args, settings)
        _log_event(args.output_dir, "stopped", reason=reason, evaluated=progress["trials_evaluated"])

    print(json.dumps(progress, indent=1))
    if progress["campaign_complete"]:
        print(f"[campaign] complete: {args.output_dir / 'best_e3_weights.json'}")
    else:
        print(f"[campaign] stopped ({reason}) after {progress['trials_evaluated']} of "
              f"{progress['trials_target']} trials. Continue with:\n  {progress['resume_command']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
