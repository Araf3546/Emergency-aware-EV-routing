#!/usr/bin/env python3
"""Re-run one earlier tuning trial's weights on today's code and compare it with that trial (2026-09-14).

    python scripts/diagnostics/staged_trial_check.py --weights-from <old trial.json> --out DIR
    python scripts/diagnostics/staged_trial_check.py --default-weights --a2-variant dropped --out DIR

A staged-validation tool for the post-audit fixes (A1, A5, A2, then B7 and S5). It runs the real
tuning-trial function, `tune_e3_weights.evaluate_weights`, for 3,600 s at the tuning seed against
the canonical baseline. The weights are the ones the old trial used; the code is today's. It writes
the new `trial.json` under DIR/trial, plus `learning.json`, which holds what `trial.json` does not
record: outcomes actually applied to learning (`aco_pso.outcomes_recorded`) and the PSO table
sizes. It then prints and saves a before/after table against the old trial.

The A2 diagnosis switches (2026-09-14) change only how an outcome settled by a new decision (the
vehicle rerouted again inside its 60 s window) is learned, and never touch production code. The
learning channels are F (PSO edge fitness), M (the running mean quality that normalises every
deposit) and D (a pheromone deposit):

    production         whatever the production code does, not emulated (the default). Since the
                       user adopted "mean protected" (2026-09-14) that is F + D, M untouched
    as_implemented     F + M + D on the planned route (the first A2 implementation)
    pso_only           F + M
    travelled_deposit  F + M + D on the edges actually driven
    dropped            nothing (before A2)
    fitness_only       F
    mean_only          M
    mean_protected     F + D on the planned route, M untouched (emulated; the diagnosis that chose it)

Every run also records, for the diagnosis: outcome quality by how the window closed; deposits by
source (horizon or leaving, against new decision), with their strength; and a timeline of the running
mean and pheromone saturation every 300 s. `--butterfly-time T` refuses exactly one reroute, the first
at or after T. It is a chaos control for deterministic E3: how far does one different decision move
J?

It reads the old trial and never writes to any campaign. Nothing here is a thesis result.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from contextlib import contextmanager
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
os.chdir(REPO_ROOT)

import traci  # noqa: E402

import tune_e3_weights as tuner  # noqa: E402
from algorithms import aco_pso, e3_core, route_outcomes  # noqa: E402

SEED, T = 20260721, 3600
CANONICAL = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"
VARIANTS = ("production", "as_implemented", "pso_only", "travelled_deposit", "dropped", "fitness_only",
            "mean_only", "mean_protected")


def row(record: dict, learning: dict | None) -> dict:
    m, o = record["metrics"], record["learning"]["outcomes"]
    c = record["coordinator"]
    settled_by_new = o["settled_by_new_decision"]
    return {
        "J": (record.get("objective") or {}).get("J"),
        "finished": m["finished"], "unfinished_in_network": m["unfinished_in_network"],
        "never_inserted": m["never_inserted"], "teleports": m["teleports"],
        "travel_s": round(m["travel_s"], 4), "time_loss_s": round(m["time_loss_s"], 4),
        "responses_s": {e: r["value_s"] for e, r in m["responses"].items()},
        "reroute_decisions": c["reroute_decisions"], "reroute_executed": c["reroute_executed"],
        "candidate_edges_scored": c["candidate_edges_scored"], "yield_actions": c["yield_actions"],
        "decisions_watched": o["decisions_watched"], "outcomes_reported": o["outcomes_reported"],
        "settled_by_new_decision": settled_by_new, "settled_by_horizon": o["settled_by_horizon"],
        "settled_by_leaving": o["settled_by_leaving"],
        "outcomes_applied_to_learning": (learning or {}).get("outcomes_applied_to_learning",
            f"not recorded; at most {o['outcomes_reported'] - settled_by_new + o.get('discarded_too_short', 0)} "
            "(reported minus those settled by a new decision, which the old code dropped)"),
        "pheromone_table_entries": record["learning"]["pheromone_table_entries"],
        "pso_velocity_entries": (learning or {}).get("pso_velocity_entries", "not recorded"),
        "edge_fitness_entries": (learning or {}).get("edge_fitness_entries", "not recorded"),
    }


# ------------------------------------------------------------------ diagnosis instrumentation

PROBE = {"quality_by_reason": {}, "deposits": {}, "timeline": [], "butterfly": None,
         "early_outcomes": 0, "early_channels": {"fitness": 0, "mean": 0, "deposit": 0}}
CONTEXT = {"source": "other"}


@contextmanager
def source(name):
    previous, CONTEXT["source"] = CONTEXT["source"], name
    try:
        yield
    finally:
        CONTEXT["source"] = previous


def fitness_update(edges, quality):
    """Channel F alone: exactly `record_outcome`'s per-edge update, without the running mean."""
    q = max(0.0, min(1.0, float(quality)))
    for edge in edges:
        if edge in aco_pso.edge_fitness:
            aco_pso.edge_fitness[edge] += aco_pso.FITNESS_ALPHA * (q - aco_pso.edge_fitness[edge])
        else:
            aco_pso.edge_fitness[edge] = q


def mean_update(quality):
    """Channel M alone: exactly `record_outcome`'s running-mean update."""
    q = max(0.0, min(1.0, float(quality)))
    aco_pso.quality_mean += aco_pso.QUALITY_MEAN_ALPHA * (q - aco_pso.quality_mean)


def instrument(variant: str, butterfly_time: float | None) -> None:
    real_settle_one = route_outcomes.RouteOutcomeTracker._settle

    def settle_one(self, pending, reason):
        outcome = real_settle_one(self, pending, reason)
        if outcome is not None:
            entry = PROBE["quality_by_reason"].setdefault(reason, [0, 0.0])
            entry[0] += 1
            entry[1] += outcome.quality
        return outcome

    route_outcomes.RouteOutcomeTracker._settle = settle_one

    real_deposit = e3_core.deposit

    def recording_deposit(route, quality=None):
        entry = PROBE["deposits"].setdefault(CONTEXT["source"], {"count": 0, "strength_sum": 0.0, "edges_sum": 0})
        entry["count"] += 1
        entry["strength_sum"] += aco_pso.deposit_strength(quality)       # at the mean it is scaled by
        entry["edges_sum"] += len(route)
        real_deposit(route, quality=quality)

    e3_core.deposit = recording_deposit

    real_settle_outcomes = e3_core.E3Core.settle_outcomes

    def settle_outcomes(self, sim_time):
        with source("horizon_or_leaving"):
            real_settle_outcomes(self, sim_time)

    e3_core.E3Core.settle_outcomes = settle_outcomes

    real_note_applied_route = e3_core.E3Core.note_applied_route

    def note_applied_route(self, vehicle_id, sim_time, route):
        if variant == "production":                     # the real method; only observed
            recorded, in_mean = aco_pso.outcomes_recorded, aco_pso.outcomes_in_mean
            deposits = PROBE["deposits"].get("new_decision", {}).get("count", 0)
            with source("new_decision"):
                real_note_applied_route(self, vehicle_id, sim_time, route)
            if aco_pso.outcomes_recorded > recorded:
                PROBE["early_outcomes"] += 1
                PROBE["early_channels"]["fitness"] += 1
                PROBE["early_channels"]["mean"] += aco_pso.outcomes_in_mean - in_mean
                PROBE["early_channels"]["deposit"] += PROBE["deposits"].get("new_decision", {}).get("count", 0) - deposits
            return
        settled = self.outcomes.record_decision(vehicle_id, sim_time, route)
        if settled is None:
            return
        PROBE["early_outcomes"] += 1
        with source("new_decision"):
            if variant == "as_implemented":
                self._learn_from(settled)                       # the first A2 implementation: the mean moves
                channels = ("fitness", "mean", "deposit")
            elif variant == "pso_only":
                e3_core.record_outcome(settled.travelled_edges, settled.quality)
                channels = ("fitness", "mean")
            elif variant == "travelled_deposit":
                e3_core.record_outcome(settled.travelled_edges, settled.quality)
                if settled.travelled_edges:
                    e3_core.deposit(settled.travelled_edges, quality=settled.quality)
                channels = ("fitness", "mean", "deposit")
            elif variant == "fitness_only":
                fitness_update(settled.travelled_edges, settled.quality)
                channels = ("fitness",)
            elif variant == "mean_only":
                mean_update(settled.quality)
                channels = ("mean",)
            elif variant == "mean_protected":
                fitness_update(settled.travelled_edges, settled.quality)
                target = settled.planned_route if self.deposit_scope == "planned" else settled.travelled_edges
                if target:
                    e3_core.deposit(target, quality=settled.quality)
                channels = ("fitness", "deposit")
            else:                                               # dropped
                channels = ()
        for channel in channels:
            PROBE["early_channels"][channel] += 1

    e3_core.E3Core.note_applied_route = note_applied_route

    real_decay = e3_core.E3Core.decay

    def decay(self, seconds=1.0):
        real_decay(self, seconds)
        t = traci.simulation.getTime()
        if int(t) % 300 == 0:
            PROBE["timeline"].append({
                "t": t, "quality_mean": aco_pso.quality_mean, "pheromone_entries": len(aco_pso.pheromone),
                "pheromone_at_ceiling": sum(1 for v in aco_pso.pheromone.values() if v >= aco_pso.TAU_MAX - 1e-9)})

    e3_core.E3Core.decay = decay

    if butterfly_time is not None:
        real_set_route = e3_core.set_route_checked

        def refuse_once(vehicle_id, new_route, current_route):
            if PROBE["butterfly"] is None and traci.simulation.getTime() >= butterfly_time:
                PROBE["butterfly"] = {"vehicle": vehicle_id, "time": traci.simulation.getTime()}
                return False
            return real_set_route(vehicle_id, new_route, current_route)

        e3_core.set_route_checked = refuse_once


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--weights-from", type=Path, help="the old trial's trial.json")
    source_group.add_argument("--default-weights", action="store_true", help="run DEFAULT_WEIGHTS (no old trial)")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--a2-variant", default="production", choices=VARIANTS,
                        help="how an outcome settled by a new decision is learned (see the module docstring)")
    parser.add_argument("--emulate-before-a2", action="store_true", help="same as --a2-variant dropped")
    parser.add_argument("--emulate-before-s5", action="store_true",
                        help="a PSO iteration, with random coefficients, at every candidate scoring and none on "
                             "the clock, as before audit S5")
    parser.add_argument("--route-choice-offset", type=int, default=0,
                        help="shift the seed of Python's random. Since E3's PSO became deterministic "
                             "(2026-09-14) nothing in the P3 path draws from it, so this is a no-op there")
    parser.add_argument("--butterfly-time", type=float, default=None,
                        help="chaos control: refuse exactly one reroute, the first at or after this time")
    parser.add_argument("--sim-time", type=int, default=T, help="default 3600 (smoke tests only)")
    parser.add_argument("--reference", type=Path, default=CANONICAL, help="baseline reference for --sim-time")
    args = parser.parse_args()
    variant = "dropped" if args.emulate_before_a2 else args.a2_variant
    if args.emulate_before_s5:
        e3_core.velocity_of = aco_pso.update_velocity
        e3_core.advance_velocities = lambda: 0
    instrument(variant, args.butterfly_time)
    if args.route_choice_offset:
        original_seed = tuner.route_choice_seed
        tuner.route_choice_seed = lambda seed: original_seed(seed) + args.route_choice_offset
    if args.default_weights:
        from p3_hybrid_router import DEFAULT_WEIGHTS

        old = None
        weights = {name: float(DEFAULT_WEIGHTS[name]) for name in tuner.WEIGHT_NAMES}
    else:
        old = json.loads(args.weights_from.read_text(encoding="utf-8"))
        weights = old["weights_requested"]
    trial_dir = args.out / "trial"
    if trial_dir.exists() and any(trial_dir.iterdir()):
        raise SystemExit(f"{trial_dir} already holds files; evidence is never overwritten")
    reference = tuner.load_reference(args.reference, SEED, args.sim_time)
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), args.sim_time, SEED,
                               trial_dir=trial_dir, reference=reference)
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    learning = {
        "a2_variant": variant, "emulate_before_s5": args.emulate_before_s5,
        "route_choice_offset": args.route_choice_offset, "butterfly_time": args.butterfly_time,
        "e3_pso_coefficients": "random (emulated before S5)" if args.emulate_before_s5 else aco_pso.E3_PSO_R,
        "outcomes_applied_to_learning": aco_pso.outcomes_recorded,
        "outcomes_in_running_mean": aco_pso.outcomes_in_mean,
        "pso_velocity_entries": len(aco_pso.velocity),
        "pso_personal_best_entries": len(aco_pso.personal_best),
        "edge_fitness_entries": len(aco_pso.edge_fitness),
        "quality_mean": aco_pso.quality_mean,
        "pheromone_entries": len(aco_pso.pheromone),
        "pheromone_at_ceiling": sum(1 for v in aco_pso.pheromone.values() if v >= aco_pso.TAU_MAX - 1e-9),
        "pheromone_mean": (sum(aco_pso.pheromone.values()) / len(aco_pso.pheromone)) if aco_pso.pheromone else None,
        "probe": PROBE,
    }
    (args.out / "learning.json").write_text(json.dumps(learning, indent=1), encoding="utf-8")
    new = json.loads((trial_dir / "trial.json").read_text(encoding="utf-8"))
    after = row(new, learning)
    before = row(old, None) if old is not None else {k: None for k in after}
    table = {k: {"before": before[k], "after": after[k]} for k in after}
    result = {"source_trial": str(args.weights_from) if old is not None else "DEFAULT_WEIGHTS",
              "weights": weights, "a2_variant": variant, "butterfly": PROBE["butterfly"],
              "rejected": rejected, "exceptions_logged": new["exceptions_logged"],
              "finished_equals_summary_arrived": new["finished_equals_summary_arrived"],
              "speed_factor_differences": new["pairing"]["speed_factor_differences"], "table": table}
    (args.out / "comparison.json").write_text(json.dumps(result, indent=1, default=str), encoding="utf-8")
    print(f"weights from {result['source_trial']}; A2 variant {variant}; butterfly {PROBE['butterfly']}; "
          f"rejected: {rejected}; exceptions {len(new['exceptions_logged'])}")
    for k, v in table.items():
        mark = "" if v["before"] == v["after"] else "   <- changed"
        print(f"  {k:30} {str(v['before']):>40}   {str(v['after']):>40}{mark}")
    return 0 if rejected is None and not new["exceptions_logged"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
