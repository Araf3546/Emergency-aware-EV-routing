"""The robustness check's rules (contract 13.20), pinned before any robustness result exists (2026-09-16).

`scripts/diagnostics/robustness_nudge.py`:
- the standardized perturbation rule and its one-nudge hook;
- the pre-registered finalist selection;
- the ranking, with every tie-break.

No SUMO: the hook is driven with a fake TraCI.
"""

import importlib.util
from pathlib import Path

import pytest

_PATH = Path(__file__).resolve().parent.parent / "scripts" / "diagnostics" / "robustness_nudge.py"
_spec = importlib.util.spec_from_file_location("robustness_nudge", _PATH)
rn = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rn)


class FakeVehicle:
    def __init__(self, speeds):
        self.speeds = dict(speeds)
        self.calls = []
        self.slowed = []

    def getIDList(self):
        self.calls.append("getIDList")
        return list(self.speeds)

    def getSpeed(self, v):
        self.calls.append(("getSpeed", v))
        return self.speeds[v]

    def slowDown(self, v, speed, duration):
        self.slowed.append((v, speed, duration))

    def getRoadID(self, v):
        return "road"

    def getLaneID(self, v):
        return "road_0"

    def getLanePosition(self, v):
        return 12.5


class FakeTraci:
    def __init__(self, speeds):
        self.vehicle = FakeVehicle(speeds)


# ---------------------------------------------------------------- the perturbation rule

def test_the_lowest_id_ordinary_car_above_5_ms_is_chosen():
    speeds = {"normal_00120": 9.0, "normal_00031": 4.9, "emergency_00001": 12.0, "normal_00077": 5.1,
              "normal_00200": 13.0}
    assert rn.choose_nudge_vehicle(speeds, speeds.get) == ("normal_00077", 5.1)


def test_exactly_5_ms_does_not_qualify_and_no_candidate_gives_none():
    speeds = {"normal_00001": 5.0, "emergency_00002": 20.0}
    assert rn.choose_nudge_vehicle(speeds, speeds.get) is None


def test_zero_padded_ids_sort_numerically():
    speeds = {"normal_00100": 8.0, "normal_00099": 8.0}
    assert rn.choose_nudge_vehicle(speeds, speeds.get)[0] == "normal_00099"


def test_without_a_nudge_the_hook_makes_no_traci_call():
    traci = FakeTraci({"normal_00001": 9.0})
    hook = rn.NudgeHook(None, traci)
    for t in range(1, 3601):
        hook.before_step(float(t))
    assert traci.vehicle.calls == [] and traci.vehicle.slowed == [] and hook.record is None


def test_one_nudge_at_the_declared_time_to_half_speed_over_2_s():
    traci = FakeTraci({"normal_00031": 10.0, "normal_00040": 12.0})
    hook = rn.NudgeHook(250.0, traci)
    for t in range(1, 600):
        hook.before_step(float(t))
    assert traci.vehicle.slowed == [("normal_00031", 5.0, 2.0)]
    assert hook.record["applied_at"] == 250.0 and hook.record["declared_time"] == 250.0
    assert hook.record["vehicle"] == "normal_00031" and hook.record["steps_tried"] == 1


def test_nothing_is_queried_before_the_declared_time():
    traci = FakeTraci({"normal_00031": 10.0})
    hook = rn.NudgeHook(250.0, traci)
    for t in range(1, 250):
        hook.before_step(float(t))
    assert traci.vehicle.calls == []


def test_a_nudge_that_finds_no_car_at_t_is_applied_later_and_recorded_as_late():
    traci = FakeTraci({"normal_00031": 3.0})
    hook = rn.NudgeHook(250.0, traci)
    hook.before_step(250.0)
    traci.vehicle.speeds["normal_00031"] = 8.0
    hook.before_step(251.0)
    assert hook.record["applied_at"] == 251.0 and hook.record["steps_tried"] == 2


def test_the_hook_wraps_update_step_and_passes_the_call_on():
    seen = []

    class Coordinator:
        def update_step(self, sim_time):
            seen.append(sim_time)
            return "done"

    traci = FakeTraci({"normal_00031": 10.0})
    rn.NudgeHook(2.0, traci).install(Coordinator)
    c = Coordinator()
    assert [c.update_step(1.0), c.update_step(2.0), c.update_step(3.0)] == ["done"] * 3
    assert seen == [1.0, 2.0, 3.0] and traci.vehicle.slowed == [("normal_00031", 5.0, 2.0)]


# ---------------------------------------------------------------- finalists

def _trial(n, value, state="COMPLETE", w=None):
    return {"trial": n, "state": state, "value": value,
            "weights": w or {k: (n + 1) / 100 + i for i, k in enumerate(rn.WEIGHT_NAMES)}}


def test_finalists_are_the_five_lowest_complete_trials():
    trials = [_trial(0, 0.95), _trial(1, 1.12), _trial(2, 0.84), _trial(3, 0.91, state="FAIL"),
              _trial(4, 0.85), _trial(5, 0.93), _trial(6, 0.90), _trial(7, 0.99)]
    assert rn.select_finalists(trials) == [2, 4, 6, 5, 0]


def test_a_duplicate_weight_set_is_replaced_by_the_next_trial():
    same = {k: 0.2 for k in rn.WEIGHT_NAMES}
    trials = [_trial(0, 0.80, w=same), _trial(1, 0.81, w=dict(same)), _trial(2, 0.82), _trial(3, 0.83),
              _trial(4, 0.84), _trial(5, 0.85)]
    assert rn.select_finalists(trials) == [0, 2, 3, 4, 5]


def test_the_campaign_rule_selects_the_expected_finalists():
    if not (rn.CAMPAIGN / "e3_trials.csv").is_file():
        pytest.skip("campaign not present")
    assert rn.select_finalists(rn.campaign_trials()) == [10, 16, 11, 17, 12]


# ---------------------------------------------------------------- ranking

T = rn.NUDGE_TIMES


def _js(*values):
    return dict(zip(T, values))


def test_lowest_mean_nudge_j_wins_and_default_is_a_full_candidate():
    ranking = rn.rank_configurations({"trial_010": _js(0.9, 1.0, 1.1, 1.0), "DEFAULT": _js(0.8, 0.9, 1.0, 0.9)},
                                     {"trial_010": 10, "DEFAULT": None})
    assert [e["configuration"] for e in ranking] == ["DEFAULT", "trial_010"]
    assert ranking[0]["score"] == pytest.approx(0.9) and ranking[0]["decided_by"] == "lowest score"


def test_a_tie_is_broken_first_by_paired_wins():
    ranking = rn.rank_configurations({"trial_010": _js(1.0, 1.0, 1.0, 1.2), "trial_016": _js(1.1, 1.1, 1.1, 0.9)},
                                     {"trial_010": 10, "trial_016": 16})
    assert ranking[0]["configuration"] == "trial_010"          # wins 3 of 4 paired times
    assert ranking[0]["decided_by"].endswith("paired wins")


def test_then_by_the_lower_worst_case():
    ranking = rn.rank_configurations({"trial_010": _js(0.8, 1.2, 1.0, 1.0), "trial_016": _js(0.9, 1.1, 1.1, 0.9)},
                                     {"trial_010": 10, "trial_016": 16})
    assert ranking[0]["configuration"] == "trial_016"          # 2-2 paired wins; worst 1.1 against 1.2
    assert ranking[0]["decided_by"].endswith("worst-case J")


def test_then_default_then_the_lowest_trial_number():
    same = _js(1.0, 1.0, 1.0, 1.0)
    ranking = rn.rank_configurations({"trial_012": dict(same), "DEFAULT": dict(same), "trial_011": dict(same)},
                                     {"trial_012": 12, "DEFAULT": None, "trial_011": 11})
    assert [e["configuration"] for e in ranking] == ["DEFAULT", "trial_011", "trial_012"]
    assert ranking[0]["decided_by"].endswith("DEFAULT preference")
    assert ranking[1]["decided_by"].endswith("lowest trial number")


def test_a_difference_above_1e_6_is_not_a_tie():
    ranking = rn.rank_configurations({"trial_010": _js(1.0, 1.0, 1.0, 1.0),
                                      "trial_016": _js(1.0, 1.0, 1.0, 1.0 + 8e-6)},
                                     {"trial_010": 10, "trial_016": 16})
    assert ranking[0]["configuration"] == "trial_010" and ranking[0]["decided_by"] == "lowest score"


def test_the_protocol_constants_are_the_contracted_ones():
    assert rn.NUDGE_TIMES == (250.0, 350.0, 450.0, 550.0)
    assert (rn.FINALISTS, rn.MIN_SPEED_MS, rn.SLOW_FACTOR, rn.SLOW_DURATION_S, rn.TIE_TOLERANCE) == \
        (5, 5.0, 0.5, 2.0, 1e-6)
    assert (rn.SEED, rn.SIM_TIME) == (20260721, 3600)
