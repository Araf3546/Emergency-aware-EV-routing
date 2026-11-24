"""The V1 comparison infrastructure (2026-09-17, decisions B2 and B5).

Pins: the emergency-layer switch of the frozen evaluator, the shared comparator loop's interface, the finished-only
tripinfo option, the comparison runner's plan, validation, aggregation, report and figures, and that nothing is
overwritten.
"""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import inspect
import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts"), str(REPO_ROOT / "scripts" / "diagnostics")]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cmp = _load("comparison_evaluation", "scripts/diagnostics/comparison_evaluation.py")


# ---------------------------------------------------------------- emergency_layer switch

def test_evaluate_weights_defaults_to_the_frozen_emergency_layer_on():
    import tune_e3_weights as tuner
    parameter = inspect.signature(tuner.evaluate_weights).parameters["emergency_layer"]
    assert parameter.default is True


def test_the_layer_on_label_is_the_frozen_evaluator_string():
    import tune_e3_weights as tuner
    from p3_hybrid_router import TUNER_EVALUATOR
    assert tuner.evaluator_label(True) == TUNER_EVALUATOR == "p3_runtime.P3RuntimeCoordinator(emergency_layer=True)"
    assert tuner.evaluator_label(False) == "p3_runtime.P3RuntimeCoordinator(emergency_layer=False)"


class _Stop(Exception):
    pass


@pytest.mark.parametrize("kwargs, expected", [({}, True), ({"emergency_layer": True}, True),
                                              ({"emergency_layer": False}, False)])
def test_the_switch_reaches_the_coordinator(monkeypatch, tmp_path, kwargs, expected):
    import p3_runtime
    import tune_e3_weights as tuner
    seen = {}

    def fake(**kw):
        seen.update(kw)
        raise _Stop()

    monkeypatch.setattr(p3_runtime, "P3RuntimeCoordinator", fake)
    weights = dict(zip(tuner.WEIGHT_NAMES, (0.2,) * 5))
    with pytest.raises(_Stop):
        tuner.evaluate_weights(weights, REPO_ROOT / "simulation.sumocfg", 60, 20260722, trial_dir=tmp_path, **kwargs)
    assert seen["emergency_layer"] is expected


def test_the_comparator_evaluator_refuses_an_unknown_method(tmp_path):
    import tune_e3_weights as tuner
    with pytest.raises(ValueError):
        tuner.evaluate_comparator("e3", REPO_ROOT / "simulation.sumocfg", 60, 20260722, trial_dir=tmp_path)


# ---------------------------------------------------------------- shared comparator loop

def test_the_shared_loop_covers_exactly_the_four_comparators():
    from algorithms.comparator_routing import COMPARATORS, METHODS, RoutingLoop
    assert COMPARATORS == ("dijkstra", "astar", "aco", "aco_pso")
    assert set(COMPARATORS) < set(METHODS)
    with pytest.raises(ValueError):
        RoutingLoop("bco")


def test_the_baseline_loop_routes_nothing_and_counts_nothing():
    from algorithms.comparator_routing import RoutingLoop
    loop = RoutingLoop("baseline")
    loop.route_vehicles(30.0)          # returns before any TraCI call
    assert loop.counters() == {"vehicles_seen_in_network": 0, "vehicles_needing_reroute": 0,
                               "vehicles_rerouted": 0, "reroute_attempts": 0, "reroutes_applied": 0}
    assert loop.outcome_tracker is None
    assert RoutingLoop("aco_pso").outcome_tracker is not None


def test_metrics_logger_uses_the_shared_loop_and_keeps_no_copy():
    source = (REPO_ROOT / "metrics/metrics_logger.py").read_text(encoding="utf-8")
    assert "RoutingLoop(algo_name)" in source
    for name in ("dijkstra_select", "astar_select", "aco_select", "aco_pso_select", "should_reroute"):
        assert name not in source


# ---------------------------------------------------------------- finished_only (B5)

TRIPINFO = """<tripinfos>
<tripinfo id="a" depart="0.00" departDelay="0.00" arrival="100.00" duration="100.00" routeLength="10.00"
 waitingTime="0.00" waitingCount="0" stopTime="0.00" timeLoss="5.00" rerouteNo="0" vType="normal_passenger"/>
<tripinfo id="b" depart="10.00" departDelay="0.00" arrival="-1.00" duration="3590.00" routeLength="5.00"
 waitingTime="100.00" waitingCount="3" stopTime="0.00" timeLoss="900.00" rerouteNo="0" vType="normal_passenger"/>
</tripinfos>"""


def test_finished_only_drops_unfinished_records_and_the_default_is_unchanged(tmp_path):
    from metrics.trip_metrics import build_report, load_tripinfo
    path = tmp_path / "tripinfo.xml"
    path.write_text(TRIPINFO, encoding="utf-8")
    assert sorted(load_tripinfo(path)) == ["a", "b"]
    assert sorted(load_tripinfo(path, finished_only=True)) == ["a"]
    assert build_report(path, finished_only=True)["completed_trips"] == 1


# ---------------------------------------------------------------- plan

def test_the_plan_runs_25_new_runs_and_reuses_the_rest():
    runs = cmp.run_matrix(cmp.SEEDS)
    assert len(runs) == 25
    assert {r["method"] for r in runs} == {"dijkstra", "astar", "aco", "aco_pso", "e3_off_t16"}
    assert len({r["out"] for r in runs}) == 25
    assert all(r["out"].startswith(cmp.OUT_ROOT + "/runs/") for r in runs)
    reused = cmp.reused_rows(cmp.SEEDS)
    assert {r["method"] for r in reused} == {"baseline", "e3_on_t16"} and len(reused) == 10
    for row in reused:
        assert (REPO_ROOT / (row.get("reference") or row["run"])).exists()


def test_the_core_comparison_has_six_rows_and_the_layer_on_row_is_separate():
    assert cmp.CORE == ("baseline", "dijkstra", "astar", "aco", "aco_pso", "e3_off_t16")
    assert cmp.EXTENSION == ("e3_on_t16",)
    assert "e3_on_t16" not in cmp.CORE
    assert cmp.SEEDS == (20260722, 20260723, 20260724, 20260725, 20260726)


def test_evidence_is_never_overwritten(tmp_path):
    target = tmp_path / "x.json"
    cmp.write_new(target, "{}")
    with pytest.raises(SystemExit):
        cmp.write_new(target, "{}")


# ---------------------------------------------------------------- validation

PLAN = {"code_fingerprint": {"code_digest": "abc"}, "e3_weights": {"pheromone": 0.1, "heuristic": 0.2,
        "protocol": 0.3, "bco": 0.2, "velocity": 0.2}, "e3_weights_file_sha256": "w"}


def _fake_run(root: Path, method: str, seed: int = 20260722) -> dict:
    spec = {"id": f"{method}_seed_{seed}", "method": method, "seed": seed, "out": f"runs/{method}_seed_{seed}"}
    ref = root / f"results/tuning_reference/seed_{seed}_3600s/reference.json"
    ref.parent.mkdir(parents=True, exist_ok=True)
    ref.write_text("{}", encoding="utf-8")
    out = root / spec["out"]
    (out / "trial").mkdir(parents=True)
    trial = {"seed": seed, "sim_time": 3600, "exceptions_logged": [], "finished_equals_summary_arrived": True,
             "no_tripinfo_record_outside_population": True,
             "pairing": {"ordinary_vehicles_compared": 3500, "speed_factor_differences": 0},
             "ambient_road_classes": {"a": "authority", "b": "authority"},
             "metrics": {"responses": {"404": {}, "401": {}, "405": {}}}, "objective": {"J": 0.9},
             "route_stats": {k: 0 for k in cmp.C1_COUNTERS}}
    meta = {"method": method, "seed": seed, "sim_time": 3600, "rejected": None, "code_digest": "abc",
            "code_matches_plan_except_seed_demand": True,
            "reference": f"results/tuning_reference/seed_{seed}_3600s/reference.json",
            "reference_sha256": hashlib.sha256(b"{}").hexdigest()}
    if method == "e3_off_t16":
        trial.update({"emergency_layer": False, "weights_requested": PLAN["e3_weights"],
                      "weights_used": PLAN["e3_weights"]})
        meta.update({"emergency_layer": False, "weights": PLAN["e3_weights"], "weights_file_sha256": "w"})
        (out / "trial" / "blockage_samples.json").write_text("[]", encoding="utf-8")
    else:
        trial["method"] = method
    (out / "trial" / "trial.json").write_text(json.dumps(trial), encoding="utf-8")
    (out / "run.json").write_text(json.dumps(meta), encoding="utf-8")
    return spec


@pytest.mark.parametrize("method", ["dijkstra", "e3_off_t16"])
def test_a_clean_run_is_valid(tmp_path, method):
    spec = _fake_run(tmp_path, method)
    assert cmp.new_run_problems(spec, PLAN, tmp_path) == []


def _mutate(root, spec, file, change):
    path = root / spec["out"] / file
    doc = json.loads(path.read_text(encoding="utf-8"))
    change(doc)
    path.write_text(json.dumps(doc), encoding="utf-8")


@pytest.mark.parametrize("file, change, expected", [
    ("trial/trial.json", lambda d: d.update(emergency_layer=True), "emergency layer is not OFF"),
    ("trial/trial.json", lambda d: d["pairing"].update(speed_factor_differences=2), "not the seed's frozen"),
    ("trial/trial.json", lambda d: d.update(exceptions_logged=["boom"]), "exception"),
    ("trial/trial.json", lambda d: d["route_stats"].pop("destination_violations"), "C1"),
    ("run.json", lambda d: d.update(seed=20260723), "wrong seed"),
    ("run.json", lambda d: d.update(code_digest="zzz"), "fingerprint"),
    ("run.json", lambda d: d.update(rejected="x"), "rejected"),
])
def test_each_integrity_failure_is_reported(tmp_path, file, change, expected):
    spec = _fake_run(tmp_path, "e3_off_t16")
    _mutate(tmp_path, spec, file, change)
    assert any(expected in p for p in cmp.new_run_problems(spec, PLAN, tmp_path))


def test_missing_blockage_samples_and_tracebacks_invalidate_a_run(tmp_path):
    spec = _fake_run(tmp_path, "e3_off_t16")
    (tmp_path / spec["out"] / "trial" / "blockage_samples.json").unlink()
    (tmp_path / spec["out"] / "run.err").write_text("Traceback (most recent call last):", encoding="utf-8")
    problems = cmp.new_run_problems(spec, PLAN, tmp_path)
    assert "C2 blockage samples missing" in problems and "traceback" in problems


def test_a_run_without_run_json_is_not_valid(tmp_path):
    spec = {"id": "x", "method": "aco", "seed": 20260722, "out": "runs/none"}
    assert cmp.new_run_problems(spec, PLAN, tmp_path) == ["no run.json (not run, or interrupted)"]


def test_a_changed_reused_row_is_refused(tmp_path):
    ref = tmp_path / "ref.json"
    ref.write_text("{}", encoding="utf-8")
    row = {"method": "baseline", "reference": "ref.json", "reference_sha256": hashlib.sha256(b"{}").hexdigest()}
    assert cmp.reused_row_problems(row, row, tmp_path) == []
    ref.write_text("{ }", encoding="utf-8")
    assert cmp.reused_row_problems(row, row, tmp_path) == ["reference changed since the plan"]


# ---------------------------------------------------------------- aggregation, report, figures (C4)

def _row(value, censored=False, j=0.9):
    status = "not_reached" if censored else "valid"
    return {"J": j, "terms": {t: j / 5 for t in ("travel", "response", "time_loss", "incomplete", "teleports")},
            "raw": {"travel_s": value, "time_loss_s": value / 2, "finished": 3000, "vehicles_scheduled": 4003,
                    "completion_share": 3000 / 4003, "incomplete_share": 0.25, "teleports": 10,
                    "responses": {i: {"value_s": value, "status": status} for i in cmp.INCIDENTS}},
            "normalized_response": {"mean": 0.8},
            "completed_only_supplementary": {"travel_s": value - 50},
            "c1_route_validity": {k: 1 for k in cmp.C1_COUNTERS}, "c2_blockage": None,
            "responders": {i: {"dispatch_delay_s": 60.0, "free_flow_s": 50.0, "delay_s": value - 110}
                           for i in cmp.INCIDENTS}}


def _synthetic_per_seed():
    per_seed = {}
    for k, s in enumerate(cmp.SEEDS):
        block = {r: _row(400.0 - 10 * i, censored=(r == "aco" and k == 0)) for i, r in enumerate(cmp.CORE + cmp.EXTENSION)}
        block["baseline"]["J"] = 1.0
        per_seed[str(s)] = block
    return per_seed


def test_aggregation_signs_and_censoring():
    agg = cmp.aggregate_rows(_synthetic_per_seed(), list(cmp.CORE + cmp.EXTENSION))
    # dijkstra is 10 s faster than baseline on every seed
    assert agg["dijkstra"]["baseline_minus_row"]["response_404"]["mean"] == pytest.approx(10.0)
    assert agg["dijkstra"]["baseline_minus_row"]["response_404_percent"]["mean"] == pytest.approx(2.5)
    # the censored ACO measurement on one seed is excluded, never substituted
    assert agg["aco"]["responses"]["404"]["n_defined"] == 4
    # row minus E3 OFF: dijkstra (390) - E3 OFF (350) = +40 in travel
    assert agg["dijkstra"]["row_minus_e3_off"]["travel_s"]["mean"] == pytest.approx(40.0)
    assert "row_minus_e3_off" not in agg["e3_off_t16"]


def _synthetic_report():
    per_seed = _synthetic_per_seed()
    return {"written_utc": "t", "seeds": list(cmp.SEEDS), "code_digest": "abc",
            "core_six_method_comparison": {m: cmp.LABELS[m] for m in cmp.CORE},
            "emergency_aware_extension": {m: cmp.LABELS[m] for m in cmp.EXTENSION},
            "measurement": "m", "planning_horizon_note": "p", "c2_note": "c",
            "per_seed": per_seed, "aggregate": cmp.aggregate_rows(per_seed, list(cmp.CORE + cmp.EXTENSION))}


def test_the_report_keeps_the_extension_out_of_the_six_method_table():
    text = cmp.report_markdown(_synthetic_report())
    core_section = text.split("## Core six-method comparison")[1].split("## Separate emergency-aware extension")[0]
    assert "layer ON" not in core_section
    assert core_section.count("\n| ") - 1 == 6          # header row plus six method rows
    assert "## Separate emergency-aware extension (not part of the six-method comparison)" in text
    assert "1.0 by construction" in text


def test_figures_are_written_once_from_the_report_only(tmp_path):
    pytest.importorskip("matplotlib")
    import comparison_figures
    report = _synthetic_report()
    written = comparison_figures.make_all(copy.deepcopy(report), tmp_path / "figures")
    assert len(written) == 7 and all(p.is_file() for p in written)
    with pytest.raises(SystemExit):
        comparison_figures.make_all(copy.deepcopy(report), tmp_path / "figures")
