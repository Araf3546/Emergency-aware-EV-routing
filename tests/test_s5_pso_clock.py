"""Audit S5 (2026-09-14), with the user's decisions: PSO advances once per step for the edges
scored in the step before, and keeps its seeded random coefficients.

E3 used to run a full PSO iteration (`update_velocity`) every time it scored a candidate. An edge
scored by several vehicles on one 30 s tick then took several iterations (inertia 0.5 each, two
random draws each, its bests raised on the first), and which vehicle saw which value depended on
the order TraCI listed the vehicles in. Scoring now only reads; `advance_velocities` iterates
each scored edge exactly once per step, in sorted order.
"""

import random

import pytest

from algorithms import aco_pso, e3_core


@pytest.fixture(autouse=True)
def clean():
    aco_pso.reset_pheromone()
    yield
    aco_pso.reset_pheromone()


def seed_state():
    for edge, quality in (("a", 0.9), ("b", 0.2), ("c", 0.5)):
        aco_pso.record_outcome([edge], quality)


def snapshot():
    return (dict(aco_pso.velocity), dict(aco_pso.personal_best), aco_pso.global_best_score, random.getstate())


def test_scoring_reads_without_changing_pso_state_or_drawing_random_numbers():
    seed_state()
    random.seed(1)
    before = snapshot()
    for _ in range(10):
        for edge in ("a", "b", "c"):
            aco_pso.velocity_of(edge)
    assert snapshot() == before


def test_an_edge_scored_ten_times_in_a_step_advances_exactly_once(monkeypatch):
    seed_state()
    calls = []
    real = aco_pso.update_velocity
    monkeypatch.setattr(aco_pso, "update_velocity",
                        lambda edge, **coefficients: calls.append(edge) or real(edge, **coefficients))
    for _ in range(10):
        aco_pso.velocity_of("c")
    aco_pso.velocity_of("a")
    assert aco_pso.advance_velocities() == 2
    assert calls == ["a", "c"]                         # once each, in sorted order
    assert aco_pso.advance_velocities() == 0           # nothing scored since the last advance


def one_step(orders, seed=7):
    """Vehicles score their candidates in the given order; then the step's advance runs."""
    aco_pso.reset_pheromone()
    seed_state()
    random.seed(seed)
    seen = {}
    for edges in orders:
        for edge in edges:
            seen.setdefault(edge, set()).add(aco_pso.velocity_of(edge))
    aco_pso.advance_velocities()
    after = {edge: aco_pso.velocity_of(edge) for edge in ("a", "b", "c")}
    return seen, after, dict(aco_pso.personal_best), aco_pso.global_best_score


def test_vehicle_processing_order_and_repetition_change_nothing():
    forward = [["a", "b"], ["b", "c"], ["a", "c"]]
    reference = one_step(forward)
    assert one_step(list(reversed(forward))) == reference
    assert one_step([["c", "a"], ["b"], ["c", "b", "a"]] * 3) == reference      # repeated scorings
    assert all(len(values) == 1 for values in reference[0].values())          # all read one value


def test_what_s5_removed_the_per_scoring_update_depended_on_order():
    def per_scoring(order):
        aco_pso.reset_pheromone()
        seed_state()
        random.seed(7)
        for edge in order:
            aco_pso.update_velocity(edge)
        return dict(aco_pso.velocity)

    assert per_scoring(["a", "b", "c", "a"]) != per_scoring(["b", "a", "a", "c"])


def test_e3s_advance_uses_fixed_coefficients_and_draws_no_random_numbers():
    """User decision 2026-09-14: E3's PSO uses r1 = r2 = 0.5, the expected value of U(0, 1)."""
    assert one_step([["a", "b", "c"]], seed=11) == one_step([["a", "b", "c"]], seed=12)   # seed-free
    aco_pso.reset_pheromone()
    seed_state()
    aco_pso.velocity["a"] = 0.2
    pb, gbest, fitness = aco_pso.personal_best.get("a", aco_pso.quality_mean), aco_pso.global_best_score, \
        aco_pso.fitness_of("a")
    random.seed(5)
    state = random.getstate()
    aco_pso.velocity_of("a")
    aco_pso.advance_velocities()
    assert random.getstate() == state
    expected = aco_pso.W * 0.2 + aco_pso.C1 * 0.5 * (fitness - pb) + aco_pso.C2 * 0.5 * (fitness - gbest)
    assert aco_pso.velocity["a"] == pytest.approx(max(-aco_pso.VMAX, min(aco_pso.VMAX, expected)))
    assert aco_pso.E3_PSO_R == 0.5


def test_the_comparators_update_still_draws_its_random_coefficients():
    seed_state()
    random.seed(5)
    state = random.getstate()
    aco_pso.update_velocity("a")                        # the ACO-PSO comparator's call, unchanged
    assert random.getstate() != state


def test_e3_advances_once_per_clock_step_after_the_memories_decay(monkeypatch):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    order = []
    monkeypatch.setattr(e3_core, "evaporate", lambda seconds=1.0: order.append("evaporate"))
    monkeypatch.setattr(e3_core, "decay_fitness", lambda seconds=1.0: order.append("decay_fitness"))
    monkeypatch.setattr(e3_core, "advance_velocities", lambda: order.append("advance") or 0)
    core.decay(1.0)
    core.decay(1.0)
    assert order == ["evaporate", "decay_fitness", "advance"] * 2


def test_e3_scoring_no_longer_runs_a_pso_iteration(monkeypatch):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    seed_state()
    aco_pso.velocity.update({"a": 0.25, "b": -0.1})
    monkeypatch.setattr(e3_core.E3Core, "congestion_suitability", staticmethod(lambda edge: 0.5))
    monkeypatch.setattr(e3_core.E3Core, "progress_suitability", lambda self, edge, destination: 0.5)
    monkeypatch.setattr(aco_pso, "update_velocity", lambda edge: pytest.fail("scoring advanced PSO"))
    scored = dict(core.score_candidates("v", ["a", "b", "c"], "dest", []))
    assert [scored[e].velocity for e in ("a", "b", "c")] == [0.25, -0.1, 0.0]
