"""The "teleports beyond arrival edge" rule in the tuning objective (2026-09-14).

Chunk 1 of the real campaign (results/e3_tuning_campaign_2026-09-14, archived since as
results/_archive/e3_tuning_campaign_2026-09-14_pre_fix) rejected trials 1, 3 and 4.
In each, SUMO removed one car whose teleport carried it past its arrival edge. The summary counted
the car in `arrived`; tripinfo wrote it with vaporized="teleport", which the objective counted as
unfinished. The rule pinned here counts only that event, confirmed by SUMO's own log line, as
arrived at its removal time. Its teleport stays in P, and every other kind of record is classified
as before.
"""

import json
import os
from pathlib import Path

import pytest

from metrics.tuning_objective import (
    ended_beyond_arrival,
    objective,
    run_metrics,
    teleports_beyond_arrival,
)

T = 1000.0
REPO_ROOT = Path(__file__).resolve().parent.parent
POP = {"a": 0.0, "b": 100.0, "c": 200.0, "d": 300.0, "e": 900.0}      # e is never inserted
RECORDS = {
    # a: a normal arrival
    "a": dict(depart=0, arrival=400, arrivalLane="A1_0", timeLoss=50, waitingTime=10, vaporized=""),
    # b: an ordinary teleport, then a normal arrival
    "b": dict(depart=100, arrival=600, arrivalLane="B1_0", timeLoss=250, waitingTime=90, vaporized=""),
    # c: an ordinary teleport, still driving at T
    "c": dict(depart=200, arrival=-1, arrivalLane="", timeLoss=300, waitingTime=200, vaporized="end"),
    # d: teleported beyond its arrival edge D9 at t = 700
    "d": dict(depart=300, arrival=700, arrivalLane="D9_1", timeLoss=350, waitingTime=150, vaporized="teleport"),
}
LOG = [
    "Warning: Teleporting vehicle 'b'; waited too long (jam), lane='B0_0', time=450.00.",
    "Warning: Vehicle 'b' ends teleporting on edge 'B1', time=455.00.",
    "Warning: Teleporting vehicle 'c'; waited too long (yield), lane='C0_0', time=900.00.",
    "Warning: Teleporting vehicle 'd'; waited too long (jam), lane='D7_0', time=690.00.",
    "Warning: Vehicle 'd' teleports beyond arrival edge 'D9', time=700.00.",
]


def write_run(tmp_path, records=RECORDS, log=LOG, arrived=3, teleports=3, name="run"):
    d = tmp_path / name
    d.mkdir()
    rows = "".join(
        f'<tripinfo id="{vid}" depart="{r["depart"]}" departDelay="0" arrival="{r["arrival"]}" '
        f'arrivalLane="{r["arrivalLane"]}" duration="0" timeLoss="{r["timeLoss"]}" '
        f'waitingTime="{r["waitingTime"]}" routeLength="100" vType="car" vaporized="{r["vaporized"]}"/>'
        for vid, r in records.items())
    (d / "tripinfo.xml").write_text(f"<tripinfos>{rows}</tripinfos>", encoding="utf-8")
    (d / "summary.xml").write_text(
        f'<summary><step time="{T}" loaded="5" inserted="4" running="1" ended="{arrived}" '
        f'arrived="{arrived}" teleports="{teleports}"/></summary>', encoding="utf-8")
    if log is not None:
        (d / "errors.log").write_bytes(("\r\n".join(log) + "\r\n").encode("utf-8"))   # SUMO on Windows
    return d


def metrics(d, with_log=True, responses=None):
    return run_metrics(d / "tripinfo.xml", d / "summary.xml", POP, T, responses or {},
                       sumo_log=(d / "errors.log") if with_log else None)


def test_the_log_parser_reads_only_the_beyond_arrival_report(tmp_path):
    d = write_run(tmp_path)
    assert teleports_beyond_arrival(d / "errors.log") == {"d": ("D9", 700.0)}
    assert teleports_beyond_arrival(None) == {}


def test_ordinary_teleport_still_driving_at_T_stays_unfinished_and_censored(tmp_path):
    m = metrics(write_run(tmp_path))
    assert (m["finished"], m["unfinished_in_network"], m["never_inserted"]) == (3, 1, 1)
    # c counts T - 200 = 800 s, exactly as before the rule
    assert m["travel_s"] == pytest.approx((400 + 500 + 800 + 400 + 100) / 5)


def test_ordinary_teleport_that_arrives_normally_is_finished_as_before(tmp_path):
    d = write_run(tmp_path)
    for m in (metrics(d), metrics(d, with_log=False)):
        assert m["finished"] >= 2                 # a and b in both
    assert metrics(d)["finished_by_teleport_beyond_arrival"] == ["d"]   # b is not in it


def test_teleport_beyond_arrival_is_finished_at_its_removal_time(tmp_path):
    m = metrics(write_run(tmp_path))
    assert m["finished_by_teleport_beyond_arrival"] == ["d"]
    assert m["incomplete_share"] == pytest.approx(2 / 5)            # c and e
    # d counts 700 - 300 = 400 s instead of T - 300 = 700 s
    without = metrics(write_run(tmp_path, name="same"), with_log=False)
    assert without["travel_s"] - m["travel_s"] == pytest.approx((T - 700) / 5)
    assert without["incomplete_share"] - m["incomplete_share"] == pytest.approx(1 / 5)


def test_teleport_beyond_arrival_still_counts_as_a_teleport(tmp_path):
    d = write_run(tmp_path)
    with_log, without = metrics(d), metrics(d, with_log=False)
    assert with_log["teleports"] == without["teleports"] == 3          # b, c and d
    assert with_log["teleports_per_vehicle"] == without["teleports_per_vehicle"] == pytest.approx(3 / 5)


def test_summary_and_tripinfo_agree_only_after_the_classification(tmp_path):
    d = write_run(tmp_path)
    assert metrics(d)["finished"] == metrics(d)["summary_arrived"] == 3
    # Without SUMO's report the event is not recognised, the counters disagree, the tuner rejects.
    assert metrics(d, with_log=False)["finished"] == 2 != metrics(d, with_log=False)["summary_arrived"]


@pytest.mark.parametrize("log_line", [
    "Warning: Vehicle 'd' teleports beyond arrival edge 'D9', time=701.00.",    # another time
    "Warning: Vehicle 'd' teleports beyond arrival edge 'D8', time=700.00.",    # another edge
    "Warning: Vehicle 'x' teleports beyond arrival edge 'D9', time=700.00.",    # another vehicle
    "Warning: Vehicle 'd' ends teleporting on edge 'D9', time=700.00.",         # an ordinary teleport
])
def test_a_record_without_the_matching_report_stays_unfinished(tmp_path, log_line):
    m = metrics(write_run(tmp_path, log=LOG[:3] + [log_line]))
    assert m["finished"] == 2 and m["finished_by_teleport_beyond_arrival"] == []


@pytest.mark.parametrize("reason", ["collision", "traci", "calibrator", "vaporizer", "gui", "end"])
def test_no_other_removal_counts_as_arrived_even_with_the_report(tmp_path, reason):
    records = dict(RECORDS, d=dict(RECORDS["d"], vaporized=reason))
    m = metrics(write_run(tmp_path, records=records))
    assert m["finished"] == 2 and m["finished_by_teleport_beyond_arrival"] == []
    assert not ended_beyond_arrival(dict(records["d"], id="d"), {"d": ("D9", 700.0)})


def test_time_loss_and_responses_do_not_depend_on_the_classification(tmp_path):
    d = write_run(tmp_path)
    responses = {404: {"start_time": 600.0, "response_time_s": 250.0}}
    with_log, without = metrics(d, responses=responses), metrics(d, with_log=False, responses=responses)
    assert with_log["time_loss_s"] == without["time_loss_s"] == pytest.approx((50 + 250 + 300 + 350) / 4)
    assert with_log["responses"] == without["responses"]


def test_objective_moves_only_through_travel_and_incomplete(tmp_path):
    d = write_run(tmp_path)
    ref = metrics(write_run(tmp_path, name="ref"), with_log=False)
    new, old = objective(metrics(d), ref), objective(metrics(d, with_log=False), ref)
    for term in ("time_loss", "teleports", "response"):
        assert new["relative"][term] == old["relative"][term]
    assert new["relative"]["travel"] < old["relative"]["travel"]
    assert new["relative"]["incomplete"] < old["relative"]["incomplete"]
    assert new["J"] == pytest.approx(sum(new["weights"][k] * new["relative"][k] for k in new["weights"]))


# ------------------------------------------------------------------ the real saved outputs

CANONICAL = REPO_ROOT / "results" / "tuning_reference" / "seed_20260721_3600s"
# The chunk-1 campaign was moved to the archive on 2026-09-14 (it ran on the pre-fix accounting rule). These tests
# only read its saved trial outputs; nothing in the archive is written (user decision 2026-09-15).
CAMPAIGN = REPO_ROOT / "results" / "_archive" / "e3_tuning_campaign_2026-09-14_pre_fix"
REPORTED = {1: "normal_02007", 3: "normal_02496", 4: "normal_02777"}


@pytest.fixture(scope="module")
def population():
    import scripts.tune_e3_weights as tuner
    from evaluation_seeds import sumo_seed_args

    here = os.getcwd()
    os.chdir(REPO_ROOT)
    try:
        return tuner.scheduled_population(
            tuner._route_files(sumo_seed_args(20260721, Path("simulation.sumocfg"))), 3600)
    finally:
        os.chdir(here)


def recompute(folder, population, stored):
    return run_metrics(folder / "tripinfo.xml", folder / "summary.xml", population, 3600,
                       {}, sumo_log=folder / "errors.log") | {"responses": stored["responses"]}


@pytest.mark.skipif(not CANONICAL.is_dir(), reason="canonical baseline outputs not present")
def test_canonical_baseline_is_unchanged(population):
    stored = json.loads((CANONICAL / "reference.json").read_text(encoding="utf-8"))["metrics"]
    new = recompute(CANONICAL, population, stored)
    assert {k: new[k] for k in stored} == stored
    assert new["finished_by_teleport_beyond_arrival"] == []


@pytest.mark.skipif(not CAMPAIGN.is_dir(), reason="chunk-1 trial outputs not present")
@pytest.mark.parametrize("trial", [0, 1, 2, 3, 4])
def test_chunk1_trials_reconcile_and_complete_trials_are_unchanged(population, trial):
    folder = CAMPAIGN / "trials" / f"trial_{trial:03d}"
    stored = json.loads((folder / "trial.json").read_text(encoding="utf-8"))["metrics"]
    new = recompute(folder, population, stored)
    assert new["finished"] == new["summary_arrived"]
    for key in ("teleports", "teleports_per_vehicle", "time_loss_s", "never_inserted", "vehicles_scheduled",
                "summary_arrived", "records_outside_population"):
        assert new[key] == stored[key]
    if trial not in REPORTED:                                 # trials 0 and 2: nothing changes
        assert {k: new[k] for k in stored} == stored
        assert new["finished_by_teleport_beyond_arrival"] == []
    else:                                                     # exactly the one car SUMO reported
        assert new["finished_by_teleport_beyond_arrival"] == [REPORTED[trial]]
        assert (new["finished"], new["unfinished_in_network"]) == (stored["finished"] + 1,
                                                                   stored["unfinished_in_network"] - 1)
