# READ-ONLY diagnostic. Modifies no production file. Run from Mine/ with `python <path>`.
"""READ-ONLY diagnostic for audit A3: is the E3 scorer actually choosing anything?

Wraps `HybridRouter.select_best_edge` during a real P3 run and answers, per selection:

  Q1  does the score saturate, so that the argmax is decided by sorted order rather than
      by the scoring function?
  Q2  how often is the top score an exact tie between two or more candidates?
  Q3  how often does the protocol (emergency) term actually change the chosen edge?
      Measured by re-running the same selection with the protocol weight forced to its
      neutral constant, which is exactly what the emergency-layer OFF arm will do.

It also reports the *old* pre-A3 raw weighted sum for every candidate, so the before and
after are measured on the same run rather than compared across sessions. Before A3 that
sum was the score, and everything above 1.0 was flattened onto the clamp.

    python scripts/diagnostics/probe_e3_scores.py            # 900 s
    python scripts/diagnostics/probe_e3_scores.py 1800
"""

import collections
import json
import os
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)
os.chdir(REPO_ROOT)

import traci  # noqa: E402
import p3_hybrid_router as H  # noqa: E402

STATS = collections.Counter()
OLD_RAW_SCORES = []
NEW_SCORES = []
PROTOCOL_WEIGHTS = collections.Counter()
CANDIDATE_COUNTS = collections.Counter()
COMPONENT_VARIES = collections.Counter()

_orig_select = H.HybridRouter.select_best_edge


def old_raw_score(router, context):
    """The pre-A3 formula: raw components, weighted, before the [0, 1] clamp."""
    return sum(
        router.weight_of(name) * value
        for name, value in router.component_values(context).items()
    )


def proportional_scores(router, candidates):
    """An alternative to min-max: divide each component by its maximum in the set.

    Min-max is a rank transform when there are only two candidates - the worst candidate
    scores 0 on every component and the best scores 1, so the *size* of each difference is
    thrown away and the decision becomes a weighted vote. Most junctions here offer two
    edges, so that matters. Dividing by the maximum also bounds every component to [0, 1]
    and removes the saturation, but keeps the ratios. This measures which of the two makes
    fewer arbitrary decisions on the same run.
    """
    values = [router.component_values(context) for _, context in candidates]
    total_weight = sum(abs(router.weight_of(name)) for name in H.COMPONENT_NAMES) or 1.0
    maxima = {name: max(row[name] for row in values) for name in H.COMPONENT_NAMES}

    scores = []
    for row in values:
        score = 0.0
        for name in H.COMPONENT_NAMES:
            top = maxima[name]
            scaled = H.CONSTANT_COMPONENT_VALUE if top <= 0.0 else row[name] / top
            score += router.weight_of(name) * scaled
        scores.append(round(score / total_weight, 6))
    return scores


def select_best_edge(self, candidates):
    best, score = _orig_select(self, candidates)

    protocol_column = [c.protocol_weight for _, c in candidates]
    single = len(candidates) < 2

    for _, context in candidates:
        STATS["scored"] += 1
        raw = old_raw_score(self, context)
        OLD_RAW_SCORES.append(round(raw, 4))
        if raw > 1.0:
            STATS["old_formula_above_clamp"] += 1
        PROTOCOL_WEIGHTS[round(context.protocol_weight, 2)] += 1
        # An emergency broadcast that reaches a vehicle with no choice of edge cannot
        # change any route. Counting these separately is the difference between "the
        # emergency signal is inert" and "the emergency signal arrives where there is
        # nothing to decide".
        if context.protocol_weight <= H.ORDINARY_AVOID_EMERGENCY_WEIGHT:
            STATS["avoid_signal_in_single_candidate_call" if single
                  else "avoid_signal_in_real_selection"] += 1

    if single:
        STATS["single_candidate_calls"] += 1
        return best, score

    if len(candidates) > 1:
        STATS["selections"] += 1
        CANDIDATE_COUNTS[len(candidates)] += 1
        # Can the protocol term discriminate here at all? If every candidate carries the
        # same protocol weight it is a constant, and a constant cannot move an argmax.
        if max(protocol_column) - min(protocol_column) > 1e-9:
            STATS["selections_where_protocol_varies"] += 1
            if min(protocol_column) <= H.ORDINARY_AVOID_EMERGENCY_WEIGHT:
                STATS["selections_with_an_avoid_signal"] += 1

        # Which of the five terms can influence anything at all? A component with the same
        # value for every candidate is a constant, and a constant cannot move an argmax -
        # so its weight is unreachable by tuning. This is the decisive Step 5 check.
        rows = [self.component_values(c) for _, c in candidates]
        for name in H.COMPONENT_NAMES:
            column = [row[name] for row in rows]
            if max(column) - min(column) > 1e-9:
                COMPONENT_VARIES[name] += 1

        scored = self.score_candidates(candidates)
        top = max(s for _, s in scored)
        NEW_SCORES.append(round(top, 4))
        if sum(1 for _, s in scored if s == top) > 1:
            STATS["selection_was_a_tie"] += 1
        if top >= 1.0:
            STATS["selection_at_upper_bound"] += 1

        # Counterfactual: the emergency-layer OFF arm forces the protocol term to its
        # neutral constant for every candidate. If that changes the winner, the protocol
        # term decided this selection.
        neutral = [
            (edge, H.HybridContext(
                pheromone=c.pheromone,
                heuristic=c.heuristic,
                protocol_weight=H.NEUTRAL_WEIGHT,
                bco_exploration=c.bco_exploration,
                velocity=c.velocity,
            ))
            for edge, c in candidates
        ]
        neutral_best, _ = _orig_select(self, neutral)
        if neutral_best != best:
            STATS["protocol_changed_the_choice"] += 1
            if min(protocol_column) <= H.ORDINARY_AVOID_EMERGENCY_WEIGHT:
                STATS["avoid_signal_changed_the_choice"] += 1

        # The same three questions under proportional normalisation, for comparison.
        prop = proportional_scores(self, candidates)
        prop_top = max(prop)
        if sum(1 for s in prop if s == prop_top) > 1:
            STATS["proportional_selection_was_a_tie"] += 1
        prop_best = candidates[prop.index(prop_top)][0]
        prop_neutral = proportional_scores(self, neutral)
        if candidates[prop_neutral.index(max(prop_neutral))][0] != prop_best:
            STATS["proportional_protocol_changed_the_choice"] += 1
        if prop_best != best:
            STATS["proportional_picked_a_different_edge"] += 1

    return best, score


H.HybridRouter.select_best_edge = select_best_edge

from p3_runtime import P3RuntimeCoordinator  # noqa: E402
from emergency_manager import get_emergency_manager  # noqa: E402


def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 900
    out = tempfile.mkdtemp(prefix="probe_e3_")

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator = P3RuntimeCoordinator()

    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true", "--end", str(duration),
        "--tripinfo-output", os.path.join(out, "probe_tripinfo.xml"),
        "--error-log", os.path.join(out, "probe_err.log"),
    ])
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)
            coordinator.update_step(sim_time)
    finally:
        try:
            traci.close()
        except Exception:
            pass

    OLD_RAW_SCORES.sort()
    n = len(OLD_RAW_SCORES)
    selections = max(1, STATS["selections"])
    print(json.dumps({
        "duration_s": duration,
        "stats": dict(STATS),
        "pct_old_formula_above_clamp": round(100 * STATS["old_formula_above_clamp"] / max(1, STATS["scored"]), 2),
        "pct_selections_at_upper_bound": round(100 * STATS["selection_at_upper_bound"] / selections, 2),
        "pct_selections_tied": round(100 * STATS["selection_was_a_tie"] / selections, 2),
        "pct_selections_changed_by_protocol": round(100 * STATS["protocol_changed_the_choice"] / selections, 2),
        "pct_selections_where_each_component_varies": {
            name: round(100 * COMPONENT_VARIES[name] / selections, 2) for name in H.COMPONENT_NAMES
        },
        "protocol_reach": {
            "selections_where_protocol_varies": STATS["selections_where_protocol_varies"],
            "selections_with_an_avoid_signal": STATS["selections_with_an_avoid_signal"],
            "avoid_signal_changed_the_choice": STATS["avoid_signal_changed_the_choice"],
            "avoid_signal_reached_a_real_selection": STATS["avoid_signal_in_real_selection"],
            "avoid_signal_wasted_on_a_single_candidate_call": STATS["avoid_signal_in_single_candidate_call"],
            "single_candidate_calls": STATS["single_candidate_calls"],
        },
        "proportional_alternative": {
            "pct_selections_tied": round(100 * STATS["proportional_selection_was_a_tie"] / selections, 2),
            "pct_selections_changed_by_protocol": round(100 * STATS["proportional_protocol_changed_the_choice"] / selections, 2),
            "pct_selections_where_it_picks_a_different_edge": round(100 * STATS["proportional_picked_a_different_edge"] / selections, 2),
        },
        "old_raw_score_percentiles": {p: OLD_RAW_SCORES[int(n * p / 100)] for p in (5, 25, 50, 75, 95, 99)} if n else {},
        "candidates_per_selection": sorted(CANDIDATE_COUNTS.items()),
        "protocol_weight_distribution_top": PROTOCOL_WEIGHTS.most_common(8),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
