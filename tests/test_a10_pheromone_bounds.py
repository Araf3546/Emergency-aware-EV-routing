"""Regression tests for audit A10: the pheromone table must be bounded and time-decayed.

Three defects are guarded here.

**Decay was tied to reroute events, not to the clock.** Evaporation lived inside
`update_pheromone`, which the P3 path calls only when a reroute is actually applied - about
480 times an hour rather than once per second. The swarm's memory therefore faded at a rate
set by how busy the router was.

**Decay ran to zero with no floor.** An edge not re-deposited within roughly 100 reroutes
retained 2.7e-05 of its value.

**A stale edge scored worse than an unknown one.** An absent edge defaulted to 1.0 while a
decayed edge sat near 0, so a road the router knew about but had not used recently looked
worse than a road it had never seen - a cliff in the middle of the score, the same defect
A3 removed from the protocol term.
"""

import pytest

from algorithms import aco_pso


@pytest.fixture(autouse=True)
def clean_table():
    aco_pso.reset_pheromone()
    yield
    aco_pso.reset_pheromone()


def test_a_deposit_raises_an_edge_above_neutral():
    aco_pso.deposit(["a", "b"])

    assert aco_pso.pheromone["a"] == aco_pso.TAU_INIT + aco_pso.Q
    assert aco_pso.pheromone["b"] == aco_pso.TAU_INIT + aco_pso.Q


def test_deposits_cannot_grow_without_bound():
    """The unbounded scheme reached 6.45 and was still climbing when the run ended."""
    for _ in range(50):
        aco_pso.deposit(["a"])

    assert aco_pso.pheromone["a"] == aco_pso.TAU_MAX


def test_decay_follows_the_clock_and_the_half_life():
    """One call per simulated second, so PHEROMONE_HALF_LIFE_S means what it says."""
    aco_pso.deposit(["a"])
    start = aco_pso.pheromone["a"]

    for _ in range(int(aco_pso.PHEROMONE_HALF_LIFE_S)):
        aco_pso.evaporate(seconds=1.0)

    halfway = aco_pso.TAU_INIT + (start - aco_pso.TAU_INIT) / 2
    assert aco_pso.pheromone["a"] == pytest.approx(halfway, abs=0.01)


def test_decay_does_not_depend_on_how_often_reroutes_happen():
    """The defect itself: 300 s of simulation must decay the same amount either way."""
    aco_pso.deposit(["a"])
    for _ in range(300):
        aco_pso.evaporate(seconds=1.0)
    busy_run = aco_pso.pheromone["a"]

    aco_pso.reset_pheromone()
    aco_pso.deposit(["a"])
    for _ in range(300):
        aco_pso.evaporate(seconds=1.0)
    quiet_run = aco_pso.pheromone["a"]

    assert busy_run == quiet_run


def test_a_stale_edge_becomes_neutral_not_worthless():
    """The cliff: stale used to mean "worse than never seen". Now it means "no information"."""
    aco_pso.deposit(["a"])
    for _ in range(4000):
        aco_pso.evaporate(seconds=1.0)

    # Dropped from the table entirely, and the default equals the decay target, so a stale
    # edge and an edge nobody has ever driven now score identically.
    assert "a" not in aco_pso.pheromone
    assert aco_pso.pheromone.get("a", aco_pso.TAU_INIT) == aco_pso.TAU_INIT
    assert aco_pso.pheromone.get("never_seen", aco_pso.TAU_INIT) == aco_pso.TAU_INIT


def test_evaporation_only_touches_edges_carrying_a_deposit():
    """Cost is proportional to what the router used, not to the 21,385 edges in the network."""
    aco_pso.deposit(["a"])
    assert aco_pso.evaporate(seconds=1.0) == 1

    for _ in range(4000):
        aco_pso.evaporate(seconds=1.0)

    assert aco_pso.evaporate(seconds=1.0) == 0


def test_values_stay_inside_the_bounds_under_any_mix_of_deposits_and_decay():
    for cycle in range(200):
        aco_pso.deposit(["a", "b"] if cycle % 3 else ["a"])
        for _ in range(7):
            aco_pso.evaporate(seconds=1.0)

    for value in aco_pso.pheromone.values():
        assert aco_pso.TAU_MIN <= value <= aco_pso.TAU_MAX


def test_the_floor_is_a_safety_bound_that_does_not_bind_today():
    """Stated explicitly so nobody assumes the floor is doing work.

    Deposits only add and decay only relaxes toward TAU_INIT, so a value can never fall
    below TAU_INIT and TAU_MIN is never reached. It is kept as a bound against a future
    rule that subtracts pheromone, in the same spirit as the A3 clamp that no longer binds.
    """
    aco_pso.deposit(["a"])
    for _ in range(10000):
        aco_pso.evaporate(seconds=1.0)

    assert aco_pso.pheromone.get("a", aco_pso.TAU_INIT) >= aco_pso.TAU_INIT
    assert aco_pso.TAU_MIN < aco_pso.TAU_INIT


def test_reset_clears_search_state_between_runs():
    aco_pso.deposit(["a"])
    # `record_selection` used to stand here. The bests are now on the realised-quality
    # scale and are raised by an observed outcome, not by an edge having been chosen.
    aco_pso.record_outcome(["a"], 0.9)
    aco_pso.update_velocity("a")
    assert aco_pso.pheromone and aco_pso.global_best_score > aco_pso.FITNESS_INIT
    assert aco_pso.edge_fitness and aco_pso.velocity

    aco_pso.reset_pheromone()

    assert aco_pso.pheromone == {}
    assert aco_pso.velocity == {}
    assert aco_pso.personal_best == {}
    assert aco_pso.edge_fitness == {}
    assert aco_pso.global_best_score == aco_pso.FITNESS_INIT
    assert aco_pso.quality_mean == aco_pso.FITNESS_INIT
