"""Work order item 1: pheromone and PSO both learn from realised outcomes.

Two things were wrong and they were the same thing. `deposit()` laid a flat Q on every edge
of every applied route the instant it was applied, so pheromone recorded how often an edge
was chosen rather than how well it worked - contradicting `design_phase.md` 2.5, "pheromone
updates from **successful** route choices". PSO's fitness was then read out of that same
table, so it could not carry information pheromone did not.

These tests pin the three properties that fix depends on: a deposit scales with realised
quality, an average outcome still deposits what the flat rule deposited, and PSO's fitness
is no longer a function of pheromone.
"""

import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from algorithms import aco_pso
from algorithms import route_outcomes


@pytest.fixture(autouse=True)
def clean_state():
    aco_pso.reset_pheromone()
    yield
    aco_pso.reset_pheromone()


# ---------------------------------------------------------------------------
# quality-weighted deposit
# ---------------------------------------------------------------------------

def test_a_good_route_is_reinforced_more_than_a_bad_one():
    """The whole point. Under the flat rule these two deposits were identical."""
    # Both edges start neutral; the mean quality is the yardstick, so seed it first.
    aco_pso.record_outcome(["seed"], 0.30)

    aco_pso.deposit(["fast"], quality=0.60)
    aco_pso.deposit(["slow"], quality=0.05)

    assert aco_pso.pheromone["fast"] > aco_pso.pheromone["slow"]


def test_an_average_route_deposits_what_the_flat_rule_deposited():
    """The normalisation is what keeps the change a change of rule, not of scale.

    Raw `Q * quality` in a network running at 12-19x free-flow would deposit about a tenth
    of the old amount on everything, the table would flatten, and pheromone would stop
    telling candidates apart - which would look like the new rule failing when it was only
    mis-scaled.
    """
    for _ in range(50):
        aco_pso.record_outcome(["seed"], 0.12)   # drive the running mean to ~0.12

    aco_pso.deposit(["typical"], quality=aco_pso.quality_mean)

    assert aco_pso.pheromone["typical"] == pytest.approx(aco_pso.TAU_INIT + aco_pso.Q, abs=1e-6)


def test_one_lucky_vehicle_cannot_write_an_edge_to_the_ceiling():
    for _ in range(50):
        aco_pso.record_outcome(["seed"], 0.05)

    aco_pso.deposit(["lucky"], quality=1.0)

    assert aco_pso.pheromone["lucky"] <= aco_pso.TAU_INIT + aco_pso.Q * aco_pso.DEPOSIT_SCALE_CAP
    assert aco_pso.pheromone["lucky"] <= aco_pso.TAU_MAX


def test_deposit_without_a_quality_keeps_the_old_flat_behaviour():
    """`update_pheromone` still drives the six-algorithm comparison table (item 2)."""
    aco_pso.deposit(["a"])
    assert aco_pso.pheromone["a"] == pytest.approx(aco_pso.TAU_INIT + aco_pso.Q)


# ---------------------------------------------------------------------------
# PSO fitness is a realised outcome, not pheromone
# ---------------------------------------------------------------------------

def test_pheromone_alone_no_longer_moves_the_pso_velocity():
    """The redundancy this fix removes: velocity used to be computed from pheromone."""
    aco_pso.update_velocity("e")
    aco_pso.velocity["e"] = 0.0

    aco_pso.pheromone["e"] = aco_pso.TAU_MAX      # as high as the table allows
    high = aco_pso.update_velocity("e")

    aco_pso.velocity["e"] = 0.0
    aco_pso.pheromone["e"] = aco_pso.TAU_MIN      # as low as the table allows
    low = aco_pso.update_velocity("e")

    # Both are zero: with no outcome recorded, fitness equals the mean for both readings.
    assert high == 0.0 and low == 0.0


def test_an_observed_outcome_becomes_the_edges_fitness():
    aco_pso.record_outcome(["e"], 0.8)
    assert aco_pso.fitness_of("e") == pytest.approx(0.8)

    aco_pso.record_outcome(["e"], 0.2)          # the road has filled up
    assert 0.2 < aco_pso.fitness_of("e") < 0.8  # follows, but is not whipped around


def test_an_undriven_edge_reads_the_running_mean_not_a_fixed_constant():
    """No cliff at the boundary - the defect A10 removed from pheromone and A3 from the
    protocol term. If unknown edges read a fixed 0.5 while measured ones sat near 0.1,
    every road nobody had tried would look like the best in the network."""
    for _ in range(80):
        aco_pso.record_outcome(["measured"], 0.10)

    assert aco_pso.fitness_of("never_driven") == pytest.approx(aco_pso.quality_mean)
    assert aco_pso.fitness_of("never_driven") < 0.3


def test_the_bests_decay_so_an_edge_can_exceed_them_again():
    """Without this every fitness sits under a permanent maximum, every velocity is
    negative, and the term stops distinguishing anything."""
    aco_pso.record_outcome(["e"], 0.9)
    aco_pso.update_velocity("e")                # the bests are raised here, PSO ordering
    peak_personal = aco_pso.personal_best["e"]
    peak_global = aco_pso.global_best_score

    for _ in range(600):                     # two half-lives
        aco_pso.decay_fitness(seconds=1.0)

    assert aco_pso.global_best_score < peak_global
    assert aco_pso.personal_best.get("e", aco_pso.quality_mean) < peak_personal


def test_velocity_is_positive_on_improvement_and_negative_on_deterioration():
    """design_phase.md 2.6: "velocity updates from route improvement or deterioration"."""
    for _ in range(60):                          # a network running at about a fifth of
        aco_pso.record_outcome(["background"], 0.20)   # free-flow, which is this scenario
        aco_pso.update_velocity("background")

    aco_pso.record_outcome(["clear"], 0.9)       # a road doing far better than the network
    aco_pso.velocity["clear"] = 0.0
    assert aco_pso.update_velocity("clear") > 0.0

    aco_pso.record_outcome(["jam"], 0.02)        # and one doing far worse
    aco_pso.velocity["jam"] = 0.0
    assert aco_pso.update_velocity("jam") < 0.0


def test_an_absorbed_improvement_stops_producing_velocity():
    """Velocity is a rate of change, so a road that is merely still good decays back to
    zero rather than accumulating a permanent bonus - that is pheromone's job."""
    aco_pso.record_outcome(["a"], 0.9)
    first = abs(aco_pso.update_velocity("a"))
    for _ in range(20):
        aco_pso.update_velocity("a")            # nothing new observed

    assert abs(aco_pso.velocity["a"]) < first


# ---------------------------------------------------------------------------
# the outcome tracker itself
# ---------------------------------------------------------------------------

class FakeTraci:
    """The three TraCI readings the tracker uses, and nothing else."""

    class TraCIException(Exception):
        pass

    def __init__(self):
        self.state = {}          # vehicle -> (road, odometer)
        self.speed_limit = 10.0

        outer = self

        class Vehicle:
            @staticmethod
            def getIDList():
                return tuple(outer.state)

            @staticmethod
            def getRoadID(vid):
                if vid not in outer.state:
                    raise outer.TraCIException(vid)
                return outer.state[vid][0]

            @staticmethod
            def getDistance(vid):
                if vid not in outer.state:
                    raise outer.TraCIException(vid)
                return outer.state[vid][1]

        class Lane:
            @staticmethod
            def getMaxSpeed(lane_id):
                return outer.speed_limit

        self.vehicle = Vehicle
        self.lane = Lane


@pytest.fixture
def fake(monkeypatch):
    stub = FakeTraci()
    monkeypatch.setattr(route_outcomes, "traci", stub)
    return stub


def drive(tracker, fake, vehicle, seconds, metres_per_second, start_time=0.0, edges=None):
    """Advance the fake simulation one second at a time, polling the tracker."""
    collected = []
    road, odometer = fake.state[vehicle]
    for step in range(1, int(seconds) + 1):
        odometer += metres_per_second
        road = edges[min(step, len(edges) - 1)] if edges else road
        fake.state[vehicle] = (road, odometer)
        collected.extend(tracker.update(start_time + step))
    return collected


def test_quality_is_realised_speed_over_the_free_flow_speed(fake):
    tracker = route_outcomes.RouteOutcomeTracker()
    fake.state["v"] = ("e0", 0.0)
    tracker.record_decision("v", 0.0, ["e0", "e1"])

    outcomes = drive(tracker, fake, "v", 60, 2.5)      # 2.5 m/s against a 10 m/s limit

    assert len(outcomes) == 1
    assert outcomes[0].quality == pytest.approx(0.25, abs=0.01)
    assert outcomes[0].reason == "horizon"


def test_a_stuck_vehicle_reports_a_quality_of_zero(fake):
    tracker = route_outcomes.RouteOutcomeTracker()
    fake.state["v"] = ("e0", 0.0)
    tracker.record_decision("v", 0.0, ["e0"])

    outcomes = drive(tracker, fake, "v", 60, 0.0)

    assert outcomes[0].quality == 0.0


def test_only_the_roads_actually_driven_are_reported(fake):
    tracker = route_outcomes.RouteOutcomeTracker()
    fake.state["v"] = ("a", 0.0)
    tracker.record_decision("v", 0.0, ["a", "b", "c", "d", "e"])

    outcomes = drive(tracker, fake, "v", 60, 1.0, edges=["a", "a", "b", "c"])

    assert outcomes[0].travelled_edges == ["a", "b", "c"]
    assert outcomes[0].planned_route == ["a", "b", "c", "d", "e"]


def test_a_window_too_short_to_mean_anything_is_discarded(fake):
    tracker = route_outcomes.RouteOutcomeTracker()
    fake.state["v"] = ("e0", 0.0)
    tracker.record_decision("v", 0.0, ["e0"])

    outcomes = drive(tracker, fake, "v", 3, 5.0)
    del fake.state["v"]
    outcomes.extend(tracker.update(4.0))

    assert outcomes == []
    assert tracker.counters["discarded_too_short"] == 1


def test_a_teleport_cannot_manufacture_a_perfect_route(fake):
    """Audit M12 showed a teleport can manufacture a response time. The odometer jump would
    manufacture a route quality the same way."""
    tracker = route_outcomes.RouteOutcomeTracker()
    fake.state["v"] = ("e0", 0.0)
    tracker.record_decision("v", 0.0, ["e0"])

    for step in range(1, 30):
        fake.state["v"] = ("e0", step * 1.0)
        tracker.update(float(step))
    fake.state["v"] = ("e0", 5000.0)          # teleported across the map
    outcomes = tracker.update(30.0)
    del fake.state["v"]
    outcomes.extend(tracker.update(31.0))

    assert outcomes == []
    assert tracker.counters["discarded_implausible"] >= 1


def test_a_leaving_vehicle_settles_on_its_last_known_position(fake):
    tracker = route_outcomes.RouteOutcomeTracker()
    fake.state["v"] = ("e0", 0.0)
    tracker.record_decision("v", 0.0, ["e0"])

    drive(tracker, fake, "v", 20, 4.0)
    del fake.state["v"]
    outcomes = tracker.update(21.0)

    assert len(outcomes) == 1
    assert outcomes[0].reason == "left_network"
    assert outcomes[0].quality == pytest.approx(0.4, abs=0.01)


def test_a_new_reroute_closes_the_window_on_the_route_it_replaces(fake):
    """Otherwise the old choice is credited with the new one's driving."""
    tracker = route_outcomes.RouteOutcomeTracker()
    fake.state["v"] = ("e0", 0.0)
    tracker.record_decision("v", 0.0, ["e0"])
    drive(tracker, fake, "v", 30, 1.0)

    settled = tracker.record_decision("v", 30.0, ["e0", "e9"])

    assert settled is not None
    assert settled.reason == "new_decision"
    assert tracker.counters["settled_by_new_decision"] == 1


# ---------------------------------------------------------------------------
# the scorer must not throw PSO's information away again
# ---------------------------------------------------------------------------

def test_a_negative_velocity_still_separates_two_candidates():
    """`component_values` used to return max(0.0, velocity).

    That was harmless when velocity came from pheromone and was usually positive. Now that
    velocity is a rate of change, most roads at any moment are not improving, so the clamp
    flattened almost every candidate pair to a single 0.0: measured over 600 s, velocity
    differed between candidates in 83.3% of selections and in 1.7% after the clamp. Audit A3
    already made the clamp unnecessary - every component is min-max normalised across the
    candidate set, so the worst velocity in the set scores 0 whatever its sign.
    """
    from p3_hybrid_router import HybridRouter, HybridContext

    router = HybridRouter()
    worse = HybridContext(1.0, 0.5, 0.5, 0.5, velocity=-0.40)
    better = HybridContext(1.0, 0.5, 0.5, 0.5, velocity=-0.05)

    rows = router.normalise_candidates([("worse", worse), ("better", better)])

    assert rows[0]["velocity"] == 0.0
    assert rows[1]["velocity"] == 1.0
    assert router.select_best_edge([("worse", worse), ("better", better)])[0] == "better"
