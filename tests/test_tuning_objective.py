"""The tuning objective decided on 2026-09-13 (D1-D8, contract decision 13.17).

Every rule is pinned on small synthetic SUMO files, so a later edit cannot quietly re-open one of
the loopholes the design audit found (results/objective_audit_2026-09-13): stranding slow trips,
dropping a failed emergency response, a dead waiting term, teleports counted as a maximum.
"""

import math

import pytest

from metrics.tuning_objective import (
    EMERGENCY_SHARE,
    OBJECTIVE_WEIGHTS,
    check_reference,
    objective,
    run_metrics,
    scheduled_population,
)

T = 1000.0


def write_run(tmp_path, trips, teleports=2, arrived=None, name="run"):
    """trips: id -> dict(depart, arrival (-1 = unfinished), timeLoss, waitingTime, departDelay)."""
    d = tmp_path / name
    d.mkdir()
    rows = []
    for vid, t in trips.items():
        rows.append(
            f'<tripinfo id="{vid}" depart="{t["depart"]}" departDelay="{t.get("departDelay", 0)}" '
            f'arrival="{t["arrival"]}" duration="{t.get("duration", 0)}" timeLoss="{t["timeLoss"]}" '
            f'waitingTime="{t["waitingTime"]}" routeLength="100" vType="car" vaporized=""/>')
    (d / "tripinfo.xml").write_text("<tripinfos>" + "".join(rows) + "</tripinfos>", encoding="utf-8")
    if arrived is None:
        arrived = sum(1 for t in trips.values() if t["arrival"] >= 0)
    (d / "summary.xml").write_text(
        f'<summary><step time="{T}" loaded="5" inserted="4" running="1" arrived="{arrived}" '
        f'teleports="{teleports}"/></summary>', encoding="utf-8")
    return d


POP = {"a": 0.0, "b": 100.0, "c": 200.0, "d": 900.0, "e": 950.0}   # e is never inserted
TRIPS = {
    "a": dict(depart=0, arrival=300, duration=300, timeLoss=100, waitingTime=40),
    "b": dict(depart=110, departDelay=10, arrival=500, duration=390, timeLoss=150, waitingTime=60),
    "c": dict(depart=200, arrival=700, duration=500, timeLoss=200, waitingTime=90),
    "d": dict(depart=900, arrival=-1, duration=100, timeLoss=30, waitingTime=5),   # unfinished at T
}
RESPONSES = {
    404: {"start_time": 600.0, "response_time_s": 250.0, "arrived_after_incident_end": True},
    401: {"start_time": 800.0, "response_time_s": None, "responder_teleported": True},
    405: {"start_time": 2100.0, "response_time_s": None},      # starts after T: not in this run
}


def metrics_for(tmp_path, trips=TRIPS, responses=RESPONSES, teleports=2, name="run"):
    d = write_run(tmp_path, trips, teleports=teleports, name=name)
    return run_metrics(d / "tripinfo.xml", d / "summary.xml", POP, T, responses)


def test_weights_are_exactly_the_d8_decision():
    assert OBJECTIVE_WEIGHTS == {"travel": 0.35, "response": 0.30, "time_loss": 0.2625,
                                 "incomplete": 0.07, "teleports": 0.0175}
    assert math.isclose(sum(OBJECTIVE_WEIGHTS.values()), 1.0, abs_tol=1e-12)
    assert EMERGENCY_SHARE == OBJECTIVE_WEIGHTS["response"]
    # The 70% traffic share keeps the original travel : delay : completion : gridlock = 40:30:8:2.
    traffic = 1.0 - EMERGENCY_SHARE
    for name, original in (("travel", 0.40), ("time_loss", 0.30), ("incomplete", 0.08), ("teleports", 0.02)):
        assert math.isclose(OBJECTIVE_WEIGHTS[name], traffic * original / 0.80, abs_tol=1e-12)
    assert "waiting" not in OBJECTIVE_WEIGHTS          # D7


def test_travel_is_censored_at_T_for_unfinished_and_never_inserted(tmp_path):
    m = metrics_for(tmp_path)
    # a: 300-0; b: 500-100 (insertion delay counts); c: 700-200; d: T-900; e: T-950 (never inserted)
    assert m["travel_s"] == pytest.approx((300 + 400 + 500 + 100 + 50) / 5)
    assert (m["finished"], m["unfinished_in_network"], m["never_inserted"]) == (3, 1, 1)
    assert m["incomplete_share"] == pytest.approx(2 / 5)


def test_time_loss_covers_every_vehicle_that_entered_the_network(tmp_path):
    m = metrics_for(tmp_path)
    assert m["time_loss_s"] == pytest.approx((100 + 150 + 200 + 30) / 4)


def test_teleports_are_the_total_per_scheduled_vehicle(tmp_path):
    m = metrics_for(tmp_path, teleports=3)
    assert m["teleports"] == 3 and m["teleports_per_vehicle"] == pytest.approx(3 / 5)


def test_responses_valid_post_window_teleported_and_not_started(tmp_path):
    r = metrics_for(tmp_path)["responses"]
    assert r["404"] == {"start_time": 600.0, "value_s": 250.0, "status": "valid_post_window"}
    assert r["401"] == {"start_time": 800.0, "value_s": T - 800.0, "status": "teleported"}
    assert "405" not in r                               # never activated before T


def test_reference_scores_exactly_one(tmp_path):
    m = metrics_for(tmp_path)
    assert objective(m, m)["J"] == pytest.approx(1.0, abs=1e-12)


def test_waiting_is_reported_but_does_not_move_J(tmp_path):
    ref = metrics_for(tmp_path, name="ref")
    changed = {k: dict(v, waitingTime=v["waitingTime"] * 10) for k, v in TRIPS.items()}
    m = metrics_for(tmp_path, trips=changed, name="more_waiting")
    assert m["reported_waiting_s_completed"] == pytest.approx(10 * ref["reported_waiting_s_completed"])
    assert objective(m, ref)["J"] == pytest.approx(objective(ref, ref)["J"])


def test_stranding_a_slow_trip_never_pays(tmp_path):
    """The survivorship loophole: finishing, however late, must score better than not finishing."""
    ref = metrics_for(tmp_path, name="ref")
    late = dict(TRIPS, d=dict(depart=900, arrival=990, duration=90, timeLoss=60, waitingTime=5))
    stranded = dict(TRIPS, d=dict(depart=900, arrival=-1, duration=100, timeLoss=60, waitingTime=5))
    j_late = objective(metrics_for(tmp_path, trips=late, name="late"), ref)["J"]
    j_stranded = objective(metrics_for(tmp_path, trips=stranded, name="stranded"), ref)["J"]
    assert j_late < j_stranded


def test_a_failed_response_is_never_better_than_a_valid_one(tmp_path):
    ref = metrics_for(tmp_path, name="ref",
                      responses={404: {"start_time": 600.0, "response_time_s": 250.0}})
    slowest_valid = {404: {"start_time": 600.0, "response_time_s": T - 600.0 - 1}}
    failed = {404: {"start_time": 600.0, "response_time_s": None}}
    j_valid = objective(metrics_for(tmp_path, responses=slowest_valid, name="valid"), ref)["J"]
    j_failed = objective(metrics_for(tmp_path, responses=failed, name="failed"), ref)["J"]
    assert j_failed > j_valid


def test_response_term_is_the_mean_of_per_incident_ratios(tmp_path):
    base = {404: {"start_time": 600.0, "response_time_s": 200.0},
            401: {"start_time": 800.0, "response_time_s": 100.0}}
    run = {404: {"start_time": 600.0, "response_time_s": 100.0},    # ratio 0.5
           401: {"start_time": 800.0, "response_time_s": 150.0}}    # ratio 1.5
    ref = metrics_for(tmp_path, responses=base, name="ref")
    m = metrics_for(tmp_path, responses=run, name="run")
    assert objective(m, ref)["relative"]["response"] == pytest.approx(1.0)


def test_population_comes_from_the_route_files(tmp_path):
    f = tmp_path / "r.rou.xml"
    f.write_text('<routes><vehicle id="x" depart="10"/><vehicle id="y" depart="2000"/></routes>',
                 encoding="utf-8")
    assert scheduled_population([f], T) == {"x": 10.0}


def test_reference_for_another_seed_or_duration_is_refused():
    ref = {"seed": 20260721, "sim_time": 3600.0, "frozen_demand_sha256": "a", "scenario_sha256": "b"}
    check_reference(ref, 20260721, 3600, "a", "b")
    for args in ((20260722, 3600, "a", "b"), (20260721, 1800, "a", "b"),
                 (20260721, 3600, "x", "b"), (20260721, 3600, "a", "x")):
        with pytest.raises(ValueError):
            check_reference(ref, *args)
