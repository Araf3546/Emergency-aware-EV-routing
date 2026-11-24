"""Regression tests for audit finding A3: the E3 score must not saturate.

Before this fix `score_edge` summed five components on their own scales and clamped the
total to [0, 1]. ACO pheromone grows without bound, so measured over a real 900 s run
36.9% of raw scores landed above the clamp (99th percentile 3.99) and 71% of selections
were made at it. At the clamp every candidate scores exactly 1.0, so the argmax is a tie
and the winner is whichever edge sorts first - the router was not choosing. It also meant
the emergency term could not move a saturated sum: the protocol weight changed the chosen
edge in 1 of 193 selections.

The fix normalises each component across the candidate set before weighting, and divides
by the total weight, so the score cannot leave [0, 1] and the clamp has nothing to do.
"""

from p3_hybrid_router import (
    CONSTANT_COMPONENT_VALUE,
    HybridContext,
    HybridRouter,
    NEUTRAL_WEIGHT,
    ORDINARY_AVOID_EMERGENCY_WEIGHT,
    ProtocolMessage,
)


def test_an_unbounded_pheromone_no_longer_saturates_the_score():
    """The defect itself: pheromone of 40 used to flatten every candidate onto 1.0."""
    router = HybridRouter()
    candidates = [
        ("edge_a", HybridContext(40.0, 0.9, 0.5, 0.8, 3.0)),
        ("edge_b", HybridContext(38.0, 0.8, 0.5, 0.7, 2.0)),
        ("edge_c", HybridContext(35.0, 0.7, 0.5, 0.6, 1.0)),
    ]

    scored = router.score_candidates(candidates)
    scores = [score for _, score in scored]

    assert all(0.0 <= score <= 1.0 for score in scores)
    # The old formula gave all three exactly 1.0. They must now be distinguishable.
    assert len(set(scores)) == 3
    assert router.select_best_edge(candidates)[0] == "edge_a"


def test_the_old_clamp_used_to_hide_the_ordering():
    """Shown side by side, so the difference is on the record rather than asserted."""
    router = HybridRouter()
    contexts = [
        HybridContext(40.0, 0.9, 0.5, 0.8, 3.0),
        HybridContext(38.0, 0.8, 0.5, 0.7, 2.0),
    ]

    old_style = [router.score_edge(context) for context in contexts]
    assert old_style == [1.0, 1.0]  # indistinguishable: the argmax was decided by order

    new_style = [score for _, score in router.score_candidates(list(zip("ab", contexts)))]
    assert new_style[0] > new_style[1]


def test_a_component_that_is_equal_everywhere_cannot_change_the_choice():
    """The property the emergency-layer OFF arm depends on.

    "Off" forces the protocol term to the neutral constant rather than to zero, because a
    constant added to every candidate cannot move the argmax. Normalisation must preserve
    that: a component with no variation is mapped to the same value for everyone.
    """
    router = HybridRouter()
    base = [
        ("edge_a", HybridContext(0.7, 0.6, 0.2, 0.1, 0.0)),
        ("edge_b", HybridContext(0.5, 0.5, 0.9, 0.2, 0.0)),
    ]
    flattened = [
        (edge, HybridContext(c.pheromone, c.heuristic, NEUTRAL_WEIGHT, c.bco_exploration, c.velocity))
        for edge, c in base
    ]

    normalised = router.normalise_candidates(flattened)
    assert {row["protocol"] for row in normalised} == {CONSTANT_COMPONENT_VALUE}

    # And the same holds for any constant, not just the neutral one.
    other = [
        (edge, HybridContext(c.pheromone, c.heuristic, 0.05, c.bco_exploration, c.velocity))
        for edge, c in base
    ]
    assert router.select_best_edge(flattened)[0] == router.select_best_edge(other)[0]


def test_scores_stay_in_range_whatever_weights_tuning_picks():
    router = HybridRouter(weights={"pheromone": 9.0, "heuristic": 0.0, "protocol": 4.0,
                                   "bco": 0.5, "velocity": 2.0})
    candidates = [
        ("edge_a", HybridContext(120.0, 0.9, 0.5, 0.8, 30.0)),
        ("edge_b", HybridContext(0.001, 0.0, 0.05, 0.0, 0.0)),
    ]

    for _, score in router.score_candidates(candidates):
        assert 0.0 <= score <= 1.0


def test_a_single_emergency_message_is_no_longer_diluted_by_ordinary_ones():
    """Audit A3: the weights used to be averaged, so the emergency signal was drowned.

    One responder saying "get off this road" among four routine broadcasts averaged to
    0.41, which is almost the neutral 0.5. The strongest signal must decide instead.
    """
    router = HybridRouter()
    messages = [
        ProtocolMessage(vehicle_id="amb", emergency_flag=True, priority_level=4, next_edge="edge_x"),
        ProtocolMessage(vehicle_id="v1", next_edge="edge_x"),
        ProtocolMessage(vehicle_id="v2", next_edge="edge_x"),
        ProtocolMessage(vehicle_id="v3", next_edge="edge_x"),
        ProtocolMessage(vehicle_id="v4", next_edge="edge_x"),
    ]

    weight = router.protocol_weight_for_edge("edge_x", messages, is_emergency_vehicle=False)

    assert weight == ORDINARY_AVOID_EMERGENCY_WEIGHT  # the old code returned about 0.41


def test_a_responder_keeps_its_corridor_despite_ordinary_chatter():
    """The mirror image: for the responder the strongest claim is the largest, not the mean."""
    router = HybridRouter()
    messages = [
        ProtocolMessage(vehicle_id="amb", emergency_flag=True, priority_level=4, next_edge="edge_x"),
        ProtocolMessage(vehicle_id="v1", next_edge="edge_x"),
        ProtocolMessage(vehicle_id="v2", next_edge="edge_x"),
    ]

    weight = router.protocol_weight_for_edge("edge_x", messages, is_emergency_vehicle=True)

    assert weight == 1.0


def test_silence_and_a_routine_message_agree():
    """Audit A3: no information used to score 0.0 while routine information scored ~0.5.

    Nothing physical happens at that boundary, so a road nobody mentioned looked like the
    worst road available purely because it was quiet.
    """
    router = HybridRouter()
    quiet = router.protocol_weight_for_edge("edge_x", [], is_emergency_vehicle=False)
    routine = router.protocol_weight_for_edge(
        "edge_x", [ProtocolMessage(vehicle_id="v1", next_edge="edge_x")], is_emergency_vehicle=False
    )

    assert quiet == NEUTRAL_WEIGHT
    assert routine == NEUTRAL_WEIGHT
