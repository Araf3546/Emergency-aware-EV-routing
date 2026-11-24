"""The heuristic must weigh congestion AND distance to destination.

`design_phase.md` section 2.3 defines the heuristic term as "short-term local suitability
such as lower congestion **and shorter distance**". Only the congestion half was ever
implemented, so E3 chose its next edge with no reference to where the vehicle was going and
`build_destination_valid_route` repaired a path from wherever that landed. Measured over
1,800 s: 87.8% of applied reroutes lengthened the remaining trip, by 229 m on average, with
a worst case of +1,061 m.

These tests cover the blending rule only; whether it actually reduces detours is a question
for `scripts/diagnostics/reroute_quality.py`, not for a unit test.
"""

from algorithms.e3_core import HEURISTIC_PROGRESS_BLEND, E3Core

blend = E3Core.blend_heuristic


def test_a_candidate_that_wins_both_halves_ranks_first():
    congestion = [1.0, 0.5, 0.2]   # first is emptiest
    progress = [0.9, 0.5, 0.1]     # first is also closest

    scores = blend(congestion, progress)

    assert scores[0] == max(scores)
    assert scores[2] == min(scores)


def test_distance_decides_when_congestion_says_nothing():
    """Every road equally empty, so only "closer to the destination" is left."""
    scores = blend([0.4, 0.4, 0.4], [0.1, 0.9, 0.5])

    assert scores[1] > scores[2] > scores[0]


def test_congestion_still_decides_when_every_road_is_equally_close():
    """The distance half must not silence the congestion half."""
    scores = blend([0.2, 0.9, 0.5], [0.3, 0.3, 0.3])

    assert scores[1] > scores[2] > scores[0]


def test_a_half_that_is_constant_contributes_a_constant():
    """A term identical for every candidate cannot move the choice - the A3 property."""
    constant_congestion = blend([0.4, 0.4], [0.2, 0.8])
    other_constant = blend([0.7, 0.7], [0.2, 0.8])

    # Different constant, same ranking and same spread: the constant half only shifts.
    assert constant_congestion == other_constant


def test_both_halves_constant_gives_a_neutral_score():
    assert blend([0.5, 0.5], [0.3, 0.3]) == [0.5, 0.5]


def test_the_two_halves_carry_equal_weight_by_default():
    """Documented consequence: with two candidates that disagree, the heuristic abstains.

    One road is emptier, the other is closer, and at an even blend the term genuinely has
    no opinion - the decision falls through to pheromone, BCO, PSO and the protocol. That
    is deliberate rather than a defect, but it does mean the distance half cannot outvote
    congestion on its own in a two-way choice, which is most junctions in this network.
    Raising HEURISTIC_PROGRESS_BLEND above 0.5 would let distance win those, at the cost of
    moving E3 toward shortest-path behaviour.
    """
    assert HEURISTIC_PROGRESS_BLEND == 0.5

    scores = blend([1.0, 0.0], [0.0, 1.0])

    assert scores[0] == scores[1] == 0.5


def test_scores_stay_within_range():
    scores = blend([0.0, 12.5, 3.0], [900.0, 0.001, 44.0])

    assert all(0.0 <= score <= 1.0 for score in scores)


def test_a_single_candidate_is_neutral_and_never_crashes():
    assert blend([0.3], [0.7]) == [0.5]
