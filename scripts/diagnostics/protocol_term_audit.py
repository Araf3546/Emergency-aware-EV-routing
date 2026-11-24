#!/usr/bin/env python3
"""How much does E3's protocol term actually decide? A read-only breakdown of component-reachability runs.

Diagnostic only. Nothing is changed and nothing is simulated. It reads the `decisions.json` and `meta.json` of runs
made by `e3_component_reachability.py --run`, which records every E3 decision: every candidate's value of every
component, and the edge E3 chose. It re-scores them offline with the production scorer
(`HybridRouter.select_best_edge`), with the same contexts and freeze rule as that tool's `--analyse`, and splits
three things that are easy to conflate:

  A  the protocol value is non-neutral (not 0.5) on a candidate, or varies across a decision's candidates. After
     A3's min-max scaling only variation can matter: a component that is the same for every candidate becomes
     the constant 0.5 and cannot move the argmax;
  B  freezing the protocol term to a constant (the mean of its values, as the tool does) changes the winner;
  C  B, and the protocol is demonstrably the reason:
     - the offline re-score reproduces the edge E3 actually chose;
     - neither scoring is a tie at the top (margin above the scorer's 1e-6 rounding).
     Only the protocol term differs between the two scorings, so the change is caused by it.

For responder decisions it also says whether the protocol's winner is the responder's own planned next edge (the
corridor) - that is, whether without the term E3 would have diverted the responder. For ordinary cars it says
whether the edge given up is one an active responder had announced.

Not recorded by the tool, so not claimed here: whether each decision's new route was then applied by SUMO.

    python scripts/diagnostics/protocol_term_audit.py --out DIR RUN [RUN ...]
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

from algorithms.e3_core import E3Core  # noqa: E402
from p3_hybrid_router import NEUTRAL_WEIGHT, HybridContext, HybridRouter  # noqa: E402

EPS = 1e-12
TIE = 1e-6            # the production scorer rounds scores to 6 decimals


def contexts(d, protocol=None):
    """The same contexts the component tool builds (its `contexts`, without other overrides)."""
    values = d["protocol"] if protocol is None else protocol
    return [(edge, HybridContext(pheromone=d["pheromone"][i], heuristic=d["heuristic"][i], protocol_weight=values[i],
                                 bco_exploration=d["bco"][i], velocity=d["velocity"][i]))
            for i, edge in enumerate(d["edges"])]


def ranked(router, ctx):
    scored = router.score_candidates(ctx)
    best_edge, best = max(scored, key=lambda item: item[1])
    others = [s for e, s in scored if e != best_edge]
    return best_edge, (best - max(others)) if others else float("inf")


def audit(run: Path, until=None) -> dict:
    meta = json.loads((run / "meta.json").read_text(encoding="utf-8"))
    decisions = json.loads((run / "decisions.json").read_text(encoding="utf-8"))
    if until is not None:
        decisions = [d for d in decisions if d["t"] <= until]
    router = HybridRouter(weights=dict(meta["weights"]), emergency_layer_enabled=True)
    multi = [d for d in decisions if len(d["edges"]) >= 2]

    def non_neutral(values):
        return [v for v in values if abs(v - NEUTRAL_WEIGHT) > EPS]

    def varies(values):
        return max(values) - min(values) > EPS

    scorings = sum(len(d["edges"]) for d in decisions)
    nn = [v for d in decisions for v in non_neutral(d["protocol"])]
    nn_multi = [v for d in multi for v in non_neutral(d["protocol"])]
    varying = [d for d in multi if varies(d["protocol"])]
    reproduced = sum(ranked(router, contexts(d))[0] == d["chosen"] for d in multi)

    b_cases = []
    for d in varying:
        actual, margin = ranked(router, contexts(d))
        frozen = [statistics.fmean(d["protocol"])] * len(d["protocol"])
        counter, margin_frozen = ranked(router, contexts(d, frozen))
        if actual == counter:
            continue
        i_actual, i_counter = d["edges"].index(actual), d["edges"].index(counter)
        prov = d.get("protocol_provenance", {})
        case = {
            "t": d["t"], "vehicle": d["vehicle"], "responder": d["is_emergency_vehicle"],
            "edges": d["edges"], "protocol": d["protocol"], "chosen_in_run": d["chosen"],
            "winner_with_protocol": actual, "winner_without": counter,
            "margin_with": round(margin, 6), "margin_without": round(margin_frozen, 6),
            "reproduces_run": actual == d["chosen"],
            "tie": margin <= TIE or margin_frozen <= TIE,
            "protocol_of_winner": d["protocol"][i_actual], "protocol_of_edge_given_up": d["protocol"][i_counter],
            "corridor_edge": prov.get("corridor_edge_applied"),
            "sources_of_edge_given_up": (prov.get("sources") or [[]] * len(d["edges"]))[i_counter],
        }
        case["demonstrated"] = case["reproduces_run"] and not case["tie"]
        if case["responder"]:
            case["effect"] = ("kept the responder on its planned next edge" if actual == case["corridor_edge"]
                              else "responder: winner is not its corridor edge")
        else:
            case["effect"] = ("steered the car off an edge an active responder had announced"
                              if abs(case["protocol_of_edge_given_up"] - 0.05) < EPS
                              else "ordinary car: other")
        b_cases.append(case)

    def split(cases):
        return {"all": len(cases), "responder": sum(c["responder"] for c in cases),
                "ordinary": sum(not c["responder"] for c in cases)}

    return {
        "run": run.name, "weights": meta["weights"], "duration_s": until or meta["duration_s"],
        "protocol_weight_share": round(meta["weights"]["protocol"] / sum(abs(v) for v in meta["weights"].values()), 4),
        "coordinator_candidate_edges_scored": meta["counters"]["candidate_edges_scored"] if until is None else None,
        "decisions": len(decisions), "candidate_scorings": scorings,
        "candidate_scorings_non_neutral": len(nn),
        "non_neutral_values": {str(v): nn.count(v) for v in sorted(set(nn))},
        "pct_non_neutral": round(100 * len(nn) / scorings, 3) if scorings else None,
        "decisions_by_responders": sum(d["is_emergency_vehicle"] for d in decisions),
        "contested_decisions": len(multi), "contested_by_responders": sum(d["is_emergency_vehicle"] for d in multi),
        "contested_candidate_scorings": sum(len(d["edges"]) for d in multi),
        "contested_candidate_scorings_non_neutral": len(nn_multi),
        "contested_with_any_non_neutral": sum(1 for d in multi if non_neutral(d["protocol"])),
        "A_contested_where_protocol_varies": {"all": len(varying),
                                              "responder": sum(d["is_emergency_vehicle"] for d in varying),
                                              "ordinary": sum(not d["is_emergency_vehicle"] for d in varying)},
        "offline_scorer_reproduces_run": f"{reproduced} of {len(multi)}",
        "B_freezing_protocol_changes_winner": split(b_cases),
        "B_pct_of_contested": round(100 * len(b_cases) / len(multi), 3) if multi else None,
        "C_demonstrated": split([c for c in b_cases if c["demonstrated"]]),
        "ties_among_B": sum(c["tie"] for c in b_cases),
        "B_not_reproducing_the_run": sum(not c["reproduces_run"] for c in b_cases),
        "effects": {e: sum(c["effect"] == e for c in b_cases) for e in sorted({c["effect"] for c in b_cases})},
        "cases": b_cases,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True)
    parser.add_argument("--until", type=float, default=None, help="only decisions up to this time (s)")
    parser.add_argument("runs", nargs="+")
    args = parser.parse_args()
    reports = [audit(Path(r), args.until) for r in args.runs]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    name = "protocol_term_audit" + (f"_until_{int(args.until)}s" if args.until else "") + ".json"
    (out / name).write_text(json.dumps(reports, indent=1), encoding="utf-8")
    for r in reports:
        print(f"\n=== {r['run']} ({r['duration_s']} s) weights {r['weights']} (protocol share {r['protocol_weight_share']})")
        for k, v in r.items():
            if k not in ("run", "weights", "cases", "duration_s", "protocol_weight_share"):
                print(f"  {k}: {v}")
        for c in r["cases"]:
            print(f"    t={c['t']:6.0f} {c['vehicle']:24s} {'responder' if c['responder'] else 'ordinary '} "
                  f"with {c['winner_with_protocol']} / without {c['winner_without']}  protocol {c['protocol']}  "
                  f"margins {c['margin_with']}/{c['margin_without']}  reproduces {c['reproduces_run']}  "
                  f"tie {c['tie']}  {c['effect']}")
    print(f"\nwritten: {out / name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
