"""Audit A2 (2026-09-14): an outcome settled by a new decision reaches learning.

`RouteOutcomeTracker.record_decision` closes a vehicle's open observation window when it is rerouted
again, and returns that window's measured outcome. Both E3 production paths threw the return value
away: `E3Core.decide` (the P3 runtime and the tuner) and `algorithms/e3.note_applied_route` (the
six-algorithm table). The measurement was counted and never learned from.
- In the five 3,600 s trials of campaign 2026-09-14b this was 72-183 outcomes per trial, 17-23% of
  all measured outcomes.
- Such an outcome now goes through `E3Core._learn_from`, the same step a horizon or leaving
  settlement goes through: one PSO fitness update and one pheromone deposit.
- "Mean protected" (user decision 2026-09-14): that outcome stays out of the running mean quality.
  The reroute trigger picks slow cars, so these windows are a congested subsample, and in the mean
  they inflated every deposit normalised by it. Horizon and leaving settlements are unchanged.
- The ACO-PSO comparator keeps its own copy of the pattern (`metrics/metrics_logger.py`); it is
  frozen by the user's decision and not changed here.
"""

import inspect

import pytest

from algorithms import aco_pso, e3, e3_core, route_outcomes


class FakeTraci:
    """The TraCI readings the outcome tracker uses."""

    class TraCIException(Exception):
        pass

    def __init__(self):
        self.state = {}          # vehicle -> (road, odometer)
        outer = self

        class Vehicle:
            @staticmethod
            def getIDList():
                return tuple(outer.state)

            @staticmethod
            def getRoadID(vid):
                return outer.state[vid][0]

            @staticmethod
            def getDistance(vid):
                return outer.state[vid][1]

        class Lane:
            @staticmethod
            def getMaxSpeed(lane_id):
                return 10.0

        self.vehicle, self.lane = Vehicle, Lane


@pytest.fixture
def fake(monkeypatch):
    stub = FakeTraci()
    monkeypatch.setattr(route_outcomes, "traci", stub)
    aco_pso.reset_pheromone()
    yield stub
    aco_pso.reset_pheromone()


@pytest.fixture
def spies(monkeypatch):
    calls = {"record_outcome": [], "deposit": []}
    real_record, real_deposit = e3_core.record_outcome, e3_core.deposit

    def record(edges, quality, update_mean=True):
        calls["record_outcome"].append((list(edges), quality, update_mean))
        real_record(edges, quality, update_mean=update_mean)

    def dep(route, quality=None):
        calls["deposit"].append((list(route), quality))
        real_deposit(route, quality=quality)

    monkeypatch.setattr(e3_core, "record_outcome", record)
    monkeypatch.setattr(e3_core, "deposit", dep)
    return calls


def drive(core, fake, vehicle, start, seconds, speed, road=None):
    for step in range(1, seconds + 1):
        r, odo = fake.state[vehicle]
        fake.state[vehicle] = (road or r, odo + speed)
        core.settle_outcomes(start + step)


def test_a_new_decision_closes_the_old_window_and_that_outcome_is_learned_once(fake, spies):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 30, 5.0)                     # 30 s at half the speed limit
    assert spies["record_outcome"] == []                     # nothing settled yet

    core.note_applied_route("v", 30.0, ["a", "c"])           # rerouted again: closes the window

    assert core.outcomes.counters["settled_by_new_decision"] == 1
    assert len(spies["record_outcome"]) == 1                 # PSO fitness: exactly once
    edges, quality, update_mean = spies["record_outcome"][0]
    assert edges == ["a"] and quality == pytest.approx(0.5, abs=0.01)
    assert update_mean is False                              # mean protected
    assert spies["deposit"] == [(["a", "b"], quality)]       # pheromone on the replaced plan
    assert aco_pso.outcomes_recorded == 1 and "a" in aco_pso.edge_fitness
    assert aco_pso.outcomes_in_mean == 0 and aco_pso.quality_mean == aco_pso.FITNESS_INIT
    assert aco_pso.pheromone["b"] == pytest.approx(aco_pso.TAU_INIT + aco_pso.deposit_strength(quality))


def test_the_replaced_window_is_never_learned_twice(fake, spies):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 30, 5.0)
    core.note_applied_route("v", 30.0, ["a", "c"])
    drive(core, fake, "v", 30.0, 60, 8.0, road="c")         # the new window reaches its horizon

    assert len(spies["record_outcome"]) == 2                 # the old window once, the new one once
    assert core.outcomes.counters["settled_by_new_decision"] == 1
    assert core.outcomes.counters["settled_by_horizon"] == 1
    assert aco_pso.outcomes_recorded == core.outcomes.counters["outcomes_reported"] == 2
    assert [call[2] for call in spies["record_outcome"]] == [False, True]
    assert aco_pso.outcomes_in_mean == 1                     # only the full window


def test_horizon_settlement_is_exactly_what_it_was(fake, spies):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 60, 2.5)
    assert len(spies["record_outcome"]) == 1
    edges, quality, update_mean = spies["record_outcome"][0]
    assert quality == pytest.approx(0.25, abs=0.01)
    assert update_mean is True
    assert spies["deposit"] == [(["a", "b"], quality)]
    assert core.outcomes.counters["settled_by_horizon"] == 1
    assert aco_pso.quality_mean == pytest.approx(
        aco_pso.FITNESS_INIT + aco_pso.QUALITY_MEAN_ALPHA * (quality - aco_pso.FITNESS_INIT))


def test_leaving_settlement_still_moves_the_mean(fake, spies):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 20, 2.0)                     # quality 0.2, away from the initial mean
    del fake.state["v"]                                      # arrived
    core.settle_outcomes(21.0)
    assert core.outcomes.counters["settled_by_leaving"] == 1
    assert [call[2] for call in spies["record_outcome"]] == [True]
    assert aco_pso.outcomes_in_mean == 1 and aco_pso.quality_mean < aco_pso.FITNESS_INIT


def test_mean_protected_is_the_fitness_update_and_deposit_with_the_mean_untouched(fake):
    """The exact arithmetic of the diagnosis variant `mean_protected`: `record_outcome`'s per-edge
    update, then the deposit, normalised by the mean as it stood before."""
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    aco_pso.record_outcome(["a"], 0.30)                      # some history: fitness and mean both set
    aco_pso.record_outcome(["x"], 0.20)
    mean_before, fitness_a_before = aco_pso.quality_mean, aco_pso.edge_fitness["a"]
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 30, 1.0)                     # 30 s at a tenth of the speed limit
    core.note_applied_route("v", 30.0, ["a", "c"])

    q = 0.1
    assert aco_pso.quality_mean == mean_before               # bit for bit
    assert aco_pso.edge_fitness["a"] == pytest.approx(fitness_a_before + aco_pso.FITNESS_ALPHA * (q - fitness_a_before),
                                                      abs=1e-3)
    expected = aco_pso.TAU_INIT + aco_pso.Q * min(aco_pso.DEPOSIT_SCALE_CAP, q / mean_before)
    assert aco_pso.pheromone["b"] == pytest.approx(expected, abs=1e-2)
    assert aco_pso.outcomes_recorded == 3 and aco_pso.outcomes_in_mean == 2


def test_an_early_window_cannot_inflate_the_next_normal_deposit(fake):
    """The mechanism the rule removes: a congested early window used to lower the mean, which
    scaled up the next full window's deposit."""
    def next_normal_deposit(core):
        fake.state["w"] = ("p", 0.0)
        core.note_applied_route("w", 31.0, ["p", "q"])
        drive(core, fake, "w", 31.0, 60, 5.0, road="p")
        return aco_pso.pheromone["q"]

    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 30, 0.5)                     # badly congested
    core.note_applied_route("v", 30.0, ["a", "c"])          # early close
    del fake.state["v"]
    with_early = next_normal_deposit(core)

    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    without_early = next_normal_deposit(core)

    assert with_early == without_early


def test_record_outcome_default_is_unchanged_for_every_other_caller():
    aco_pso.reset_pheromone()
    aco_pso.record_outcome(["a"], 0.1)
    assert aco_pso.quality_mean == aco_pso.FITNESS_INIT + aco_pso.QUALITY_MEAN_ALPHA * (0.1 - aco_pso.FITNESS_INIT)
    assert aco_pso.outcomes_recorded == aco_pso.outcomes_in_mean == 1
    aco_pso.record_outcome(["a"], 0.1, update_mean=False)
    assert aco_pso.outcomes_recorded == 2 and aco_pso.outcomes_in_mean == 1
    aco_pso.reset_pheromone()
    assert aco_pso.outcomes_recorded == aco_pso.outcomes_in_mean == 0


def test_the_frozen_aco_pso_comparator_still_moves_the_mean():
    # 2026-09-17: the comparators' routing loop moved unchanged from metrics_logger into
    # algorithms/comparator_routing.py (decisions B2/B5), which metrics_logger now calls.
    from algorithms import comparator_routing
    from metrics import metrics_logger

    source = inspect.getsource(comparator_routing)
    assert "aco_pso_record_outcome(outcome.travelled_edges, outcome.quality)" in source
    assert "update_mean" not in source
    assert "RoutingLoop(algo_name)" in inspect.getsource(metrics_logger)
    assert "update_mean" not in inspect.getsource(metrics_logger)


def test_a_first_decision_learns_nothing(fake, spies):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    assert spies["record_outcome"] == [] and spies["deposit"] == []


def test_decide_uses_the_same_path_when_it_applies_a_route(monkeypatch):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    noted = []

    class Vehicle:
        @staticmethod
        def getRoadID(v):
            return "a"

        @staticmethod
        def getRoute(v):
            return ("a", "b", "dest")

    class Traci:
        TraCIException = Exception
        vehicle = Vehicle

    monkeypatch.setattr(e3_core, "traci", Traci)
    monkeypatch.setattr(e3_core, "get_outgoing_edges", lambda edge, veh_id=None: ["b", "c"])
    monkeypatch.setattr(e3_core, "build_destination_valid_route", lambda v, r, cur, sel: ["a", sel, "dest"])
    monkeypatch.setattr(e3_core, "set_route_checked", lambda v, new, old: True)
    monkeypatch.setattr(core, "select_edge", lambda *a, **k: ("c", 2))
    monkeypatch.setattr(core, "note_applied_route", lambda v, t, r: noted.append((v, t, list(r))))

    report = core.decide("v", 30.0)

    assert report["applied"] is True
    assert noted == [("v", 30.0, ["a", "c", "dest"])]


def test_the_six_algorithm_path_learns_from_it_too(fake, spies, monkeypatch):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    monkeypatch.setattr(e3, "get_core", lambda: core)
    fake.state["v"] = ("a", 0.0)
    e3.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 30, 5.0)
    e3.note_applied_route("v", 30.0, ["a", "c"])
    assert len(spies["record_outcome"]) == 1 and len(spies["deposit"]) == 1
    assert spies["record_outcome"][0][2] is False            # the same rule: kept out of the mean
    assert aco_pso.quality_mean == aco_pso.FITNESS_INIT


def test_state_summary_reports_both_counters(fake):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    fake.state["v"] = ("a", 0.0)
    core.note_applied_route("v", 0.0, ["a", "b"])
    drive(core, fake, "v", 0.0, 30, 5.0)
    core.note_applied_route("v", 30.0, ["a", "c"])
    summary = core.state_summary()
    assert summary["outcomes_recorded"] == 1 and summary["outcomes_in_mean"] == 0
