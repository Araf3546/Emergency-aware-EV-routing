"""Audit A6: the BCO term must reward exploration, not repetition.

`scout_memory` records how recently and often an edge has been used, and that part was
always correct. The defect was in how the score read it: the raw value was passed in as
`bco_exploration`, where a higher number makes an edge more attractive, so the edge just
chosen - which receives the largest increment - became the most attractive next time. That
is reinforcement, which is pheromone's job. `design_phase.md` section 2.1 asks BCO for the
opposite:

    BCO adds a more explicit exploration mechanism that is useful when traffic conditions
    change quickly... helps the system discover alternative routes that the other two
    methods may not explore aggressively enough.

The correction inverts the interpretation only; the counter is untouched.
"""

import pytest

from p3_hybrid_router import HybridRouter, ProtocolMessage


def test_a_less_used_edge_scores_higher_than_a_heavily_used_one():
    router = HybridRouter()
    router.scout_memory = {"busy": 0.9, "quiet": 0.1}

    busy = router.build_context(1.0, 0.5, "busy", [], 0.0).bco_exploration
    quiet = router.build_context(1.0, 0.5, "quiet", [], 0.0).bco_exploration

    assert quiet > busy


def test_an_untouched_edge_is_the_most_explorable():
    router = HybridRouter()
    router.scout_memory = {"used": 0.4}

    untouched = router.build_context(1.0, 0.5, "never_seen", [], 0.0).bco_exploration
    used = router.build_context(1.0, 0.5, "used", [], 0.0).bco_exploration

    assert untouched == 1.0
    assert untouched > used


def test_an_edge_that_decayed_out_reads_the_same_as_one_never_visited():
    """decay_scout_memory drops a faded edge, so both are absent - no cliff."""
    router = HybridRouter(bco_half_life_s=1.0)
    router.scout_memory = {"stale": 0.01}
    router.decay_scout_memory(seconds=10.0)

    assert "stale" not in router.scout_memory
    stale = router.build_context(1.0, 0.5, "stale", [], 0.0).bco_exploration
    fresh = router.build_context(1.0, 0.5, "never_seen", [], 0.0).bco_exploration
    assert stale == fresh == 1.0


def test_choosing_an_edge_makes_it_less_attractive_next_time():
    """The defect, stated as a behaviour: selection must not raise the BCO score."""
    router = HybridRouter()
    candidates = ["a", "b"]

    before = router.build_context(1.0, 0.5, "a", [], 0.0).bco_exploration
    router.update_exploration_memory(candidates, selected_edge="a")
    after = router.build_context(1.0, 0.5, "a", [], 0.0).bco_exploration

    assert after < before


def test_the_chosen_edge_is_discouraged_more_than_the_alternatives():
    """+0.15 for the winner and +0.05 for the rest now means the winner is damped most."""
    router = HybridRouter()
    router.update_exploration_memory(["a", "b"], selected_edge="a")

    chosen = router.build_context(1.0, 0.5, "a", [], 0.0).bco_exploration
    alternative = router.build_context(1.0, 0.5, "b", [], 0.0).bco_exploration

    assert alternative > chosen


def test_the_term_stays_inside_zero_to_one():
    router = HybridRouter()
    for _ in range(40):
        router.update_exploration_memory(["a", "b"], selected_edge="a")

    for edge in ("a", "b", "unseen"):
        value = router.build_context(1.0, 0.5, edge, [], 0.0).bco_exploration
        assert 0.0 <= value <= 1.0


def test_bco_stays_independent_of_pheromone_and_of_destination_progress():
    """BCO must not read either signal - it is the one term with its own memory."""
    router = HybridRouter()
    router.scout_memory = {"a": 0.3}

    low_pheromone = router.build_context(0.1, 0.1, "a", [], 0.0).bco_exploration
    high_pheromone = router.build_context(9.9, 0.9, "a", [], 0.0).bco_exploration

    assert low_pheromone == high_pheromone


def test_the_protocol_term_is_unaffected_by_the_change():
    router = HybridRouter()
    messages = [ProtocolMessage(vehicle_id="amb", emergency_flag=True, next_edge="a")]

    weight = router.protocol_weight_for_edge("a", messages, is_emergency_vehicle=False)

    assert weight < 0.5


def test_memory_fades_on_the_clock_at_the_stated_half_life():
    """Audit A6 follow-up: decay used to run per routing decision, so its real rate was
    5.77 x 0.02 = 0.115 per simulated second and a chosen edge was forgotten after 1.30 s -
    shorter than the 30 s reroute interval, so no vehicle reached its next decision with the
    memory intact. It now fades on the clock at a stated half-life."""
    router = HybridRouter(bco_half_life_s=120.0)
    router.scout_memory = {"a": 0.8}

    router.decay_scout_memory(seconds=120.0)

    assert router.scout_memory["a"] == pytest.approx(0.4, abs=1e-6)


def test_decay_no_longer_depends_on_how_busy_the_router_is():
    """The defect: the same simulated minute must fade the memory by the same amount
    whether the router made 6 decisions in it or 600."""
    quiet = HybridRouter(bco_half_life_s=60.0)
    quiet.scout_memory = {"a": 1.0}
    for _ in range(60):
        quiet.decay_scout_memory(seconds=1.0)

    busy = HybridRouter(bco_half_life_s=60.0)
    busy.scout_memory = {"a": 1.0}
    busy.decay_scout_memory(seconds=60.0)

    assert quiet.scout_memory["a"] == pytest.approx(busy.scout_memory["a"], abs=1e-6)


def test_a_chosen_edge_now_survives_past_one_reroute_interval():
    """30 s is one reroute interval; the old behaviour forgot after 1.3 s."""
    router = HybridRouter()
    router.update_exploration_memory(["a", "b"], selected_edge="a")

    for _ in range(30):
        router.decay_scout_memory(seconds=1.0)

    assert router.scout_memory.get("a", 0.0) > 0.0
