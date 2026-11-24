#!/usr/bin/env python3
"""Is every E3 scoring component reachable, varying and able to change the chosen edge?

Asked before weight tuning: tuning a component that is identical across the candidates of
every decision returns an arbitrary weight, because a constant added to every candidate cannot
move the argmax (audit A3 normalises each component across the candidate set).

ONE instrumented run records, for every routing decision, every candidate's value of every
component - pheromone, PSO velocity, BCO exploration, the heuristic's congestion and
destination-progress halves, and the protocol/emergency term - plus which edge E3 chose. All
the questions are then answered offline from that record, with the production scorer itself
(`HybridRouter.select_best_edge`), so no counterfactual re-implements the maths:

* does the component differ across the candidates of a decision, and by how much;
* would freezing it to a constant change the chosen edge (its reach at the committed weight);
* does it duplicate another component (does it prefer the same candidate).

The emergency layer is ON, because with it off the protocol term is the neutral constant by
construction and its reachability cannot be measured at all.

Nothing is changed: the run wraps E3's own methods, reads their values and passes them through.

Added 2026-09-14 for the audit's A1/A5. Each decision also records the protocol value every
candidate would have under the four combinations of two rules: with or without the deciding
vehicle's own message (A1), and with a message that names no next edge applied to every edge or
to none (A5). It also records who sent the emergency or yield messages naming each candidate. The
variants are computed on the same traffic state with the production `protocol_weight_for_edge`
on transformed message lists, so they attribute an effect without re-running anything. `meta.json`
also records the learning counters: outcomes settled and applied, and table sizes.

    python scripts/diagnostics/e3_component_reachability.py --run --seed 20260721 --duration 1800 --out DIR
    python scripts/diagnostics/e3_component_reachability.py --analyse DIR
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import statistics
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402

COMPONENTS = ("pheromone", "velocity", "bco", "congestion", "progress", "protocol")
EPSILON = 1e-12
# self_in/out: the deciding vehicle's own message included or not (A1).
# none_all/none_named: a message with no next edge counts for every edge, or for none (A5).
# The self_out variants include the explicit responder corridor rule (2026-09-14, the user's
# option 2), as the fixed code does; the self_in variants are the old code, which had no explicit
# rule. The last variant drops the corridor rule too (the audit's option 1).
PROTOCOL_VARIANTS = ("self_in_none_all", "self_out_none_all", "self_in_none_named", "self_out_none_named",
                     "self_out_none_named_no_corridor")


def protocol_variants(router, vehicle_id, edges, passed, is_emergency, own, corridor_edge=None):
    """Each candidate's protocol value under each rule combination, on this traffic state."""
    others = [m for m in passed if m.vehicle_id != vehicle_id]
    with_self = others + ([own] if own is not None else [])

    def value(edge, messages, none_to_all, corridor=None):
        if none_to_all:
            messages = [m if m.next_edge else dataclasses.replace(m, next_edge=edge) for m in messages]
        else:
            messages = [m for m in messages if m.next_edge]
        return router.protocol_weight_for_edge(edge, messages, is_emergency_vehicle=is_emergency,
                                               corridor_edge=corridor)

    variants = {
        "self_in_none_all": [value(e, with_self, True) for e in edges],
        "self_out_none_all": [value(e, others, True, corridor_edge) for e in edges],
        "self_in_none_named": [value(e, with_self, False) for e in edges],
        "self_out_none_named": [value(e, others, False, corridor_edge) for e in edges],
        "self_out_none_named_no_corridor": [value(e, others, False) for e in edges],
    }
    signalling = [m for m in with_self if m.emergency_flag or m.yield_request]
    sources = [sorted({m.vehicle_id for m in signalling if m.next_edge == e}) for e in edges]
    return variants, {
        "sources": sources,
        "own_message": None if own is None else {
            "emergency": bool(own.emergency_flag), "yield_request": bool(own.yield_request),
            "next_edge": own.next_edge},
        "signals_naming_no_edge": sum(1 for m in signalling if not m.next_edge),
    }


def run(args) -> int:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    import p3_runtime
    from algorithms import e3_core
    from emergency_manager import get_emergency_manager
    from evaluation_seeds import frozen_demand_paths, route_choice_seed, sumo_seed_args

    seed_args = sumo_seed_args(args.seed)
    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    from algorithms import aco_pso

    decisions = []
    buffer = {}
    holder = {}

    original_blend = e3_core.E3Core.blend_heuristic   # a staticmethod reads as a plain function
    original_score = e3_core.E3Core.score_candidates
    original_select = e3_core.E3Core.select_edge

    def blend_heuristic(congestion, progress):
        blended = original_blend(congestion, progress)
        buffer["congestion"] = list(congestion)
        buffer["progress"] = list(progress)
        return blended

    def score_candidates(self, vehicle_id, outgoing_edges, destination_edge, messages,
                         is_emergency_vehicle=False, **corridor):
        messages = list(messages)
        scored = original_score(self, vehicle_id, outgoing_edges, destination_edge, messages,
                                is_emergency_vehicle=is_emergency_vehicle, **corridor)
        buffer["edges"] = [edge for edge, _ in scored]
        buffer["pheromone"] = [c.pheromone for _, c in scored]
        buffer["heuristic"] = [c.heuristic for _, c in scored]
        buffer["protocol"] = [c.protocol_weight for _, c in scored]
        buffer["bco"] = [c.bco_exploration for _, c in scored]
        buffer["velocity"] = [c.velocity for _, c in scored]
        buffer["is_emergency_vehicle"] = bool(is_emergency_vehicle)
        entry = holder["coordinator"].protocol_state.neighbors.get(vehicle_id)
        corridor_edge = corridor.get("corridor_edge")
        # The corridor a responder would claim, whether or not the code under test applies it, so
        # the old code's run records it too and a --no-responder-corridor run can be compared.
        claim = holder["corridor_of"](vehicle_id) if is_emergency_vehicle and holder.get("corridor_of") else None
        variants, provenance = protocol_variants(
            self.router, vehicle_id, buffer["edges"], messages, is_emergency_vehicle,
            entry.message if entry is not None else None, corridor_edge=claim)
        buffer["protocol_variants"] = variants
        provenance["corridor_edge_applied"] = corridor_edge
        provenance["corridor_edge_claimed"] = claim
        buffer["protocol_provenance"] = provenance
        return scored

    def select_edge(self, vehicle_id, outgoing, destination, messages, is_emergency_vehicle=False,
                    **corridor):
        buffer.clear()
        chosen, scored = original_select(self, vehicle_id, outgoing, destination, messages,
                                         is_emergency_vehicle=is_emergency_vehicle, **corridor)
        if buffer.get("edges"):
            keys = ("edges", "pheromone", "heuristic", "protocol", "bco", "velocity",
                    "congestion", "progress", "protocol_variants", "protocol_provenance")
            record = {k: buffer[k] for k in keys if k in buffer}
            record["t"] = traci.simulation.getTime()
            record["vehicle"] = vehicle_id
            record["chosen"] = chosen
            record["is_emergency_vehicle"] = buffer.get("is_emergency_vehicle", False)
            decisions.append(record)
        return chosen, scored

    e3_core.E3Core.blend_heuristic = staticmethod(blend_heuristic)
    e3_core.E3Core.score_candidates = score_candidates
    e3_core.E3Core.select_edge = select_edge

    # The code's own corridor definition, kept before any patching so both runs record the same claim.
    holder["corridor_of"] = getattr(p3_runtime.P3RuntimeCoordinator, "_responder_corridor_edge", None)
    if args.no_responder_corridor:
        # Paired ablation of the explicit corridor rule (2026-09-14): everything else identical.
        p3_runtime.P3RuntimeCoordinator._responder_corridor_edge = staticmethod(lambda vehicle_id: None)
    coordinator = p3_runtime.P3RuntimeCoordinator(
        emergency_layer=True, route_choice_seed=route_choice_seed(args.seed))
    holder["coordinator"] = coordinator

    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true",
        "--end", str(args.duration),
        "--tripinfo-output", str(out / "tripinfo.xml"),
        "--summary-output", str(out / "summary.xml"),
        "--message-log", str(out / "messages.log"),
        "--error-log", str(out / "errors.log"),
        *seed_args,
    ])
    speed_factors = {}
    started = time.perf_counter()
    try:
        while traci.simulation.getTime() < args.duration:
            traci.simulationStep()
            t = traci.simulation.getTime()
            manager.update(t)
            coordinator.update_step(t)
            for vehicle_id in traci.simulation.getDepartedIDList():
                try:
                    speed_factors[vehicle_id] = traci.vehicle.getSpeedFactor(vehicle_id)
                except traci.TraCIException:
                    pass
    finally:
        try:
            traci.close()
        except Exception:
            pass

    frozen = {k: float(v) for k, v in json.loads(
        frozen_demand_paths(args.seed)[1].read_text(encoding="utf-8"))["speed_factors"].items()}
    common = [v for v in frozen if v in speed_factors]
    metrics = coordinator.metrics
    meta = {
        "seed": args.seed, "duration_s": args.duration, "emergency_layer": "on",
        "responder_corridor_rule": "off (--no-responder-corridor)" if args.no_responder_corridor else "as in the code",
        "responses": {str(k): v for k, v in manager.get_response_summary().items()},
        "weights": dict(coordinator.e3.router.weights),
        "wall_seconds": round(time.perf_counter() - started, 1),
        "counters": {
            "reroute_decisions": metrics.reroute_decisions,
            "reroute_executed": metrics.reroute_executed,
            "candidate_edges_scored": metrics.candidate_edges_scored,
            "yield_requests": metrics.yield_requests,
            "yield_actions": metrics.yield_actions_performed,
            "emergency_reroutes": metrics.emergency_reroutes,
        },
        "recorded_decisions": len(decisions),
        "pairing": {"ordinary_vehicles_compared": len(common),
                    "speed_factor_differences": sum(1 for v in common if speed_factors[v] != frozen[v])},
        # 2026-09-14: what reached learning (audit A2), read after the run.
        "learning": {
            "outcomes": dict(coordinator.e3.outcomes.counters),
            "outcomes_applied_to_learning": aco_pso.outcomes_recorded,
            "pheromone_table_entries": len(aco_pso.pheromone),
            "pso_velocity_entries": len(aco_pso.velocity),
            "pso_personal_best_entries": len(aco_pso.personal_best),
            "edge_fitness_entries": len(aco_pso.edge_fitness),
            "quality_mean": aco_pso.quality_mean,
        },
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    (out / "decisions.json").write_text(json.dumps(decisions), encoding="utf-8")
    print(json.dumps(meta))
    return 0


def analyse(directory: Path) -> int:
    from algorithms.e3_core import E3Core
    from p3_hybrid_router import DEFAULT_WEIGHTS, HybridContext, HybridRouter

    meta = json.loads((directory / "meta.json").read_text(encoding="utf-8"))
    decisions = json.loads((directory / "decisions.json").read_text(encoding="utf-8"))
    multi = [d for d in decisions if len(d["edges"]) >= 2]
    router = HybridRouter(weights=dict(meta["weights"]), emergency_layer_enabled=True)

    def contexts(d, overrides=None):
        overrides = overrides or {}
        congestion = overrides.get("congestion", d["congestion"])
        progress = overrides.get("progress", d["progress"])
        if "congestion" in overrides or "progress" in overrides:
            heuristic = E3Core.blend_heuristic(congestion, progress)
        else:
            heuristic = d["heuristic"]
        built = []
        for i, edge in enumerate(d["edges"]):
            built.append((edge, HybridContext(
                pheromone=overrides.get("pheromone", d["pheromone"])[i],
                heuristic=heuristic[i],
                protocol_weight=overrides.get("protocol", d["protocol"])[i],
                bco_exploration=overrides.get("bco", d["bco"])[i],
                velocity=overrides.get("velocity", d["velocity"])[i],
            )))
        return built

    def varies(values):
        return (max(values) - min(values)) > EPSILON

    def frozen_like(values):
        mean = statistics.fmean(values)
        return [mean] * len(values)

    print("")
    print(f"=== E3 component reachability - seed {meta['seed']}, {meta['duration_s']} s, "
          f"emergency layer {meta['emergency_layer']}")
    print(f"decisions recorded {len(decisions)}; {len(multi)} had 2+ candidates "
          f"(the only ones where any component can change the outcome)")
    print(f"committed weights: {meta['weights']}")
    print(f"counters: {meta['counters']}")
    print(f"pairing: {meta['pairing']}")

    reproduced = sum(1 for d in multi if router.select_best_edge(contexts(d))[0] == d["chosen"])
    print(f"offline scorer reproduces the run's chosen edge: {reproduced} of {len(multi)}")
    print("")

    rows = []
    for name in COMPONENTS:
        values = [d[name] for d in multi]
        non_constant = [v for v in values if varies(v)]
        spans = [max(v) - min(v) for v in non_constant]
        flat = [x for v in values for x in v]
        changed = 0
        for d in multi:
            if not varies(d[name]):
                continue
            before = router.select_best_edge(contexts(d))[0]
            after = router.select_best_edge(contexts(d, {name: frozen_like(d[name])}))[0]
            if before != after:
                changed += 1
        weight_name = {"congestion": "heuristic", "progress": "heuristic"}.get(name, name)
        rows.append({
            "component": name,
            "weight": meta["weights"].get(weight_name, DEFAULT_WEIGHTS.get(weight_name)),
            "pct_non_constant": round(100.0 * len(non_constant) / len(multi), 2) if multi else None,
            "value_min": round(min(flat), 4),
            "value_median": round(statistics.median(flat), 4),
            "value_max": round(max(flat), 4),
            "median_span_when_varying": round(statistics.median(spans), 4) if spans else None,
            "freezing_changes_winner": changed,
            "pct_of_all_decisions": round(100.0 * changed / len(multi), 2) if multi else None,
            "pct_of_varying_decisions": round(100.0 * changed / len(non_constant), 2) if non_constant else None,
        })

    header = "component    weight  %varies   value range              median span  freezing changes winner"
    print(header)
    print("-" * len(header))
    for r in rows:
        rng = str(r["value_min"]) + " .. " + str(r["value_max"])
        reach = (str(r["freezing_changes_winner"]) + " (" + str(r["pct_of_all_decisions"]) +
                 "% of all, " + str(r["pct_of_varying_decisions"]) + "% when it varies)")
        print(f"{r['component']:12s} {str(r['weight']):>6s} {str(r['pct_non_constant']):>7s}% "
              f"{rng:24s} {str(r['median_span_when_varying']):>11s}  {reach}")

    print("")
    print("pairwise agreement on the preferred candidate (only decisions where both vary):")
    pairs = [("velocity", "pheromone"), ("bco", "pheromone"), ("congestion", "progress"),
             ("bco", "velocity"), ("protocol", "pheromone")]
    agreements = {}
    for a, b in pairs:
        both = [d for d in multi if varies(d[a]) and varies(d[b])]
        if not both:
            print(f"  {a:10s} vs {b:10s}: never both vary")
            agreements[a + "_vs_" + b] = None
            continue
        same = 0
        for d in both:
            ia = max(range(len(d[a])), key=lambda i: d[a][i])
            ib = max(range(len(d[b])), key=lambda i: d[b][i])
            if d["edges"][ia] == d["edges"][ib]:
                same += 1
        pct = round(100.0 * same / len(both), 1)
        agreements[a + "_vs_" + b] = {"decisions": len(both), "agreement_pct": pct}
        print(f"  {a:10s} vs {b:10s}: {pct:5.1f}% of {len(both)} decisions prefer the same candidate")

    # 2026-09-14, audit A1/A5: the protocol term's reach under each rule, on the same states.
    variants_out = None
    if multi and "protocol_variants" in multi[0]:
        names = [n for n in PROTOCOL_VARIANTS if n in multi[0]["protocol_variants"]]
        running = [n for n in names if all(d["protocol_variants"][n] == d["protocol"] for d in multi)]
        variants_out = {"running_code_matches": running, "variants": {}}
        print("")
        print(f"protocol term under each rule (A1: own message in/out; A5: no-next-edge message counts for "
              f"all edges/none); running code matches {running}")

        def reach(d, values):
            if not varies(values):
                return False
            return (router.select_best_edge(contexts(d, {"protocol": values}))[0]
                    != router.select_best_edge(contexts(d, {"protocol": frozen_like(values)}))[0])

        for n in names:
            hits = [d for d in multi if reach(d, d["protocol_variants"][n])]
            observed = sorted({round(x, 4) for d in multi for x in d["protocol_variants"][n]
                               if varies(d["protocol_variants"][n])})
            row = {
                "protocol_varies_in": sum(1 for d in multi if varies(d["protocol_variants"][n])),
                "protocol_changes_winner": len(hits),
                "of_which_responder": sum(1 for d in hits if d["is_emergency_vehicle"]),
                "values_observed_when_varying": observed,
                "cases": [{"t": d["t"], "vehicle": d["vehicle"], "responder": d["is_emergency_vehicle"],
                           "chosen": d["chosen"], "edges": d["edges"], "values": d["protocol_variants"][n],
                           "sources": d["protocol_provenance"]["sources"],
                           "own_message": d["protocol_provenance"]["own_message"]} for d in hits],
            }
            variants_out["variants"][n] = row
            print(f"  {n:20s} varies in {row['protocol_varies_in']:3d}, changes the winner in "
                  f"{row['protocol_changes_winner']:3d} (responder {row['of_which_responder']}); values {observed}")

        old = [d for d in multi if reach(d, d["protocol_variants"]["self_in_none_all"])]
        survives = {n: sum(1 for d in old if reach(d, d["protocol_variants"][n])) for n in names}
        own_only = sum(1 for d in old if any(
            s == [d["vehicle"]] for s, v in zip(d["protocol_provenance"]["sources"], d["protocol_variants"]["self_in_none_all"])
            if v != 0.5))
        variants_out["old_rule_cases_surviving"] = survives
        variants_out["old_rule_cases_where_a_non_neutral_edge_was_named_by_the_vehicle_itself_only"] = own_only
        print(f"  of the {len(old)} decisions the term decides under the old rule: still decided with A1 only "
              f"{survives['self_out_none_all']}, with A5 only {survives['self_in_none_named']}, with both "
              f"{survives['self_out_none_named']}; a non-neutral edge named only by the deciding vehicle itself "
              f"in {own_only}")

    payload = {"meta": meta, "components": rows, "pairwise": agreements,
               "decisions_with_2plus_candidates": len(multi),
               "offline_scorer_reproduces_winner": reproduced,
               "protocol_rule_variants": variants_out}
    (directory / "analysis.json").write_text(json.dumps(payload, indent=1), encoding="utf-8")
    print("")
    print(f"written: {directory / 'analysis.json'}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--analyse", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=20260721)
    parser.add_argument("--duration", type=int, default=1800)
    parser.add_argument("--out", default=None)
    parser.add_argument("--no-responder-corridor", action="store_true",
                        help="run with the explicit responder corridor rule switched off (paired ablation)")
    args = parser.parse_args()
    if args.analyse:
        return analyse(args.analyse)
    if not args.run or not args.out:
        parser.error("use --run --out DIR, or --analyse DIR")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
