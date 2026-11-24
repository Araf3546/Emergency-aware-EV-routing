#!/usr/bin/env python3
"""The closure reachability filter in both arms of the emergency-layer ablation, on real SUMO runs (2026-09-15).

Diagnostic only: it changes no production code and writes only into the folder given by --out.

User decision 2026-09-15: the filter (`route_utils.reachable_candidates`) is a routing-feasibility rule, not
part of the V2V emergency mechanism, so it runs with the emergency layer ON and OFF. This checks, on the
tuner's own trial (`tune_e3_weights.evaluate_weights`: tuning seed, frozen demand, all three incidents):

- the filter acts in both arms, and what it did is right. Every call made while a closure is on is
  recorded, then re-checked offline on the network file (`check`), independently of SUMO's router:
  removed candidates must be dead ends, kept ones must reach the destination, a fallback must have had
  no reachable candidate;
- no ordinary car chooses a dead end during a closure, in either arm, unless nothing else was offered;
- the switch still gates only the emergency mechanisms: with the layer OFF every protocol value is the
  neutral 0.5 (no avoidance, no corridor), nobody yields, and nobody raises the yield flag;
- the OFF arm with every receiver silenced (`--deaf`) is the same run as the OFF arm, bit for bit
  (`compare`): with the layer OFF, V2V messages reach no behaviour at all.

The tuner writes `"evaluator": "...(emergency_layer=True)"` into every trial.json; it is its fixed label.
The arm this run actually had is read back from the coordinator after the run, into arm.json.

    python scripts/diagnostics/closure_filter_arms_check.py run --layer on  --out DIR/on_default
    python scripts/diagnostics/closure_filter_arms_check.py run --layer off --out DIR/off_default
    python scripts/diagnostics/closure_filter_arms_check.py run --layer off --deaf --out DIR/off_default_deaf
    python scripts/diagnostics/closure_filter_arms_check.py check --out DIR DIR/on_default DIR/off_default
    python scripts/diagnostics/closure_filter_arms_check.py compare DIR/off_default DIR/off_default_deaf
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Shared helpers: the offline road graph, and the exact record comparisons of `reproduce-check`.
from yield_signal_diagnosis import Graph, records, sumo_lines  # noqa: E402  (also chdirs to REPO_ROOT)

REFERENCE = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"
SEED = 20260721
EXEMPT_CLASS = "emergency"      # dispatched responders; closures do not apply to them (contract 13.2)


def live_run(args) -> int:
    import traci
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    import tune_e3_weights as tuner
    import p3_runtime
    import p3_hybrid_router
    from algorithms import e3_core, route_utils

    out = Path(args.out)
    trial_dir = out / "trial"
    if trial_dir.exists() and any(trial_dir.iterdir()):
        raise SystemExit(f"{trial_dir} already holds files; evidence is never overwritten")
    out.mkdir(parents=True, exist_ok=True)
    layer = args.layer == "on"
    holder = {"coordinator": None, "errors": []}
    filter_calls, closure_decisions = [], []
    protocol = {"values": 0, "non_neutral": 0, "non_neutral_for_responders": 0, "corridor": 0}
    responder_decisions = {"scored_as_emergency": 0, "scored_as_ordinary": 0}

    # -------- the arm: force the switch on the tuner's own coordinator
    real_init = p3_runtime.P3RuntimeCoordinator.__init__

    def init(self, *a, **k):
        k["emergency_layer"] = layer
        real_init(self, *a, **k)
        holder["coordinator"] = self
        if args.deaf:
            # Every receiver is silenced; broadcasting itself is untouched.
            self.protocol_state.get_fresh_neighbors = lambda *x, **y: []
            self.protocol_state.get_emergency_neighbors = lambda *x, **y: []

    p3_runtime.P3RuntimeCoordinator.__init__ = init

    # -------- observation only: every wrapper calls straight through and returns what it got
    real_filter = e3_core.reachable_candidates

    def observed_filter(veh_id, candidates, destination_edge):
        candidates = list(candidates)
        closed = route_utils.blocked_edges()
        fallbacks_before = route_utils.ROUTE_STATS["closure_filter_fallbacks"]
        kept = real_filter(veh_id, candidates, destination_edge)
        if closed and len(candidates) >= 2:
            try:
                filter_calls.append({
                    "t": traci.simulation.getTime(), "vehicle": veh_id,
                    "vtype": route_utils.vehicle_type_for_routing(veh_id),
                    "vclass": traci.vehicle.getVehicleClass(veh_id), "destination": destination_edge,
                    "closed": sorted(closed), "offered": candidates, "kept": list(kept),
                    "fallback": route_utils.ROUTE_STATS["closure_filter_fallbacks"] > fallbacks_before})
            except Exception as exc:
                holder["errors"].append(f"filter record {veh_id}: {exc!r}")
        return kept

    e3_core.reachable_candidates = observed_filter

    real_weight = p3_hybrid_router.HybridRouter.protocol_weight_for_edge

    def observed_weight(self, edge_id, messages, is_emergency_vehicle=False, corridor_edge=None):
        value = real_weight(self, edge_id, messages, is_emergency_vehicle=is_emergency_vehicle,
                            corridor_edge=corridor_edge)
        protocol["values"] += 1
        if value != p3_hybrid_router.NEUTRAL_WEIGHT:
            protocol["non_neutral"] += 1
            protocol["non_neutral_for_responders"] += bool(is_emergency_vehicle)
        if is_emergency_vehicle and corridor_edge is not None and edge_id == corridor_edge:
            protocol["corridor"] += 1
        return value

    p3_hybrid_router.HybridRouter.protocol_weight_for_edge = observed_weight

    real_select = e3_core.E3Core.select_edge

    def observed_select(self, vehicle_id, outgoing, destination, messages, is_emergency_vehicle=False,
                        corridor_edge=None):
        chosen, n = real_select(self, vehicle_id, outgoing, destination, messages,
                                is_emergency_vehicle=is_emergency_vehicle, corridor_edge=corridor_edge)
        try:
            coordinator = holder["coordinator"]
            if coordinator is not None and vehicle_id in coordinator._active_responders:
                responder_decisions["scored_as_emergency" if is_emergency_vehicle else "scored_as_ordinary"] += 1
            closed = route_utils.blocked_edges()
            if closed and len(outgoing) >= 2:
                closure_decisions.append({
                    "t": traci.simulation.getTime(), "vehicle": vehicle_id,
                    "vclass": traci.vehicle.getVehicleClass(vehicle_id), "destination": destination,
                    "closed": sorted(closed), "offered": list(outgoing), "chosen": chosen})
        except Exception as exc:
            holder["errors"].append(f"select record {vehicle_id}: {exc!r}")
        return chosen, n

    e3_core.E3Core.select_edge = observed_select

    # -------- the real trial
    weights = {name: float(p3_hybrid_router.DEFAULT_WEIGHTS[name]) for name in tuner.WEIGHT_NAMES}
    # `--unscored` is for a short tool check only: the reference is for 3,600 s, so a shorter run has no J.
    reference = None if args.unscored else tuner.load_reference(REFERENCE, SEED, args.sim_time)
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), args.sim_time, SEED,
                               trial_dir=trial_dir, reference=reference)
    except tuner.TrialRejected as exc:
        rejected = str(exc)

    c = holder["coordinator"]
    arm = {
        "layer_requested": args.layer, "deaf_receivers": bool(args.deaf), "weights": weights,
        "switch_read_back": {"coordinator.emergency_layer": c.emergency_layer,
                             "e3.emergency_layer": c.e3.emergency_layer,
                             "router.emergency_layer_enabled": c.e3.router.emergency_layer_enabled},
        "rejected": rejected, "instrumentation_errors": holder["errors"],
        "yield_actions": c.yield_actions, "yield_flags_broadcast": c.metrics.yield_requests,
        "protocol_values": protocol, "responder_decisions": responder_decisions,
        "filter_calls_during_closures": len(filter_calls),
        "filter_calls_that_removed": sum(len(f["kept"]) < len(f["offered"]) for f in filter_calls),
        "filter_fallbacks": sum(f["fallback"] for f in filter_calls),
        "contested_decisions_during_closures": len(closure_decisions),
    }
    (out / "filter_calls.json").write_text(json.dumps(filter_calls, indent=1), encoding="utf-8")
    (out / "closure_decisions.json").write_text(json.dumps(closure_decisions, indent=1), encoding="utf-8")
    (out / "arm.json").write_text(json.dumps(arm, indent=1), encoding="utf-8")
    # `reproduce-check` in yield_signal_diagnosis.py reads a summary.json; this one says the run is no emulation.
    (out / "summary.json").write_text(json.dumps({"variant": f"layer_{args.layer}" + ("_deaf" if args.deaf else ""),
                                                  "clean": None}), encoding="utf-8")
    print(json.dumps(arm))
    switch_ok = all(v is layer for v in arm["switch_read_back"].values())
    return 0 if rejected is None and not holder["errors"] and switch_ok else 1


class ClassGraph(Graph):
    """The offline road graph for one vehicle class (the base class is for passenger cars)."""

    def __init__(self, vclass: str, net=None) -> None:
        if net is None:
            super().__init__()
        else:
            self.net, self.cache = net, {}
        self.vclass = vclass

    def successors(self, edge_id: str) -> list:
        e = self.edge(edge_id)
        if e is None:
            return []
        return sorted(s.getID() for s in e.getOutgoing() if s.allows(self.vclass))


def check(args) -> int:
    """Re-check every recorded filter call and every choice made during a closure, on the network file."""
    graphs = {}

    def reaches(vclass, edge, destination, closed):
        if vclass not in graphs:
            graphs[vclass] = ClassGraph(vclass, next(iter(graphs.values())).net if graphs else None)
        return graphs[vclass].search(edge, destination, closed, True) != float("inf")

    report = {}
    for run in (Path(p) for p in args.runs):
        arm = json.loads((run / "arm.json").read_text(encoding="utf-8"))
        trial = json.loads((run / "trial" / "trial.json").read_text(encoding="utf-8"))
        calls = json.loads((run / "filter_calls.json").read_text(encoding="utf-8"))
        decisions = json.loads((run / "closure_decisions.json").read_text(encoding="utf-8"))
        r = {"calls": len(calls), "calls_by_exempt_responders": 0, "responder_calls_that_removed": 0,
             "removed_confirmed_dead_end": 0, "removed_disputed": [], "kept_confirmed_reachable": 0,
             "kept_disputed": [], "fallbacks": 0, "fallbacks_confirmed": 0, "fallbacks_disputed": []}
        for f in calls:
            removed = [e for e in f["offered"] if e not in f["kept"]]
            if f["vclass"] == EXEMPT_CLASS:
                # A responder may drive through the closure, so the closed edges are not closed for it.
                r["calls_by_exempt_responders"] += 1
                r["responder_calls_that_removed"] += bool(removed)
                continue
            if f["fallback"]:
                r["fallbacks"] += 1
                if any(reaches(f["vclass"], e, f["destination"], f["closed"]) for e in f["offered"]):
                    r["fallbacks_disputed"].append(f)
                else:
                    r["fallbacks_confirmed"] += 1
                continue
            for e in removed:
                if reaches(f["vclass"], e, f["destination"], f["closed"]):
                    r["removed_disputed"].append({**f, "edge": e})
                else:
                    r["removed_confirmed_dead_end"] += 1
            for e in f["kept"]:
                if reaches(f["vclass"], e, f["destination"], f["closed"]):
                    r["kept_confirmed_reachable"] += 1
                else:
                    r["kept_disputed"].append({**f, "edge": e})
        fallback_keys = {(f["t"], f["vehicle"]) for f in calls if f["fallback"]}
        chose_dead, ordinary = [], 0
        for d in decisions:
            if d["vclass"] == EXEMPT_CLASS or (d["t"], d["vehicle"]) in fallback_keys:
                continue
            ordinary += 1
            if d["chosen"] and not reaches(d["vclass"], d["chosen"], d["destination"], d["closed"]):
                chose_dead.append(d)
        r["ordinary_contested_decisions_during_closures"] = ordinary
        r["ordinary_chose_a_dead_end"] = chose_dead
        stats = trial["route_stats"]
        r["counters"] = {k: stats[k] for k in ("closure_filtered_decisions", "closure_candidates_removed",
                                               "closure_filter_fallbacks")}
        # The recorded calls must add up to the production counters (standing rule: cross-check counters).
        r["counters_agree_with_recorded_calls"] = (
            stats["closure_filtered_decisions"] == sum(len(f["kept"]) < len(f["offered"]) for f in calls)
            and stats["closure_candidates_removed"] == sum(len(f["offered"]) - len(f["kept"]) for f in calls)
            and stats["closure_filter_fallbacks"] == sum(f["fallback"] for f in calls))
        m = trial["metrics"]
        tracebacks = sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback")
                         for p in (run / "run.err", run / "trial" / "errors.log") if p.is_file())
        r["run"] = {"arm": arm["switch_read_back"], "deaf": arm["deaf_receivers"], "rejected": arm["rejected"],
                    "J": (trial["objective"] or {}).get("J"), "finished": m["finished"],
                    "scheduled": m["vehicles_scheduled"],
                    "completion_pct": round(100 * m["finished"] / m["vehicles_scheduled"], 2),
                    "teleports": m["teleports"], "travel_s_finished_trips": round(m["travel_s"], 1),
                    "time_loss_s": round(m["time_loss_s"], 1),
                    "responses_s": {e: (x["value_s"], x["status"]) for e, x in m["responses"].items()},
                    "reroutes_applied": trial["coordinator"]["reroute_executed"],
                    "yield_actions": arm["yield_actions"], "yield_flags_broadcast": arm["yield_flags_broadcast"],
                    "protocol_values": arm["protocol_values"], "responder_decisions": arm["responder_decisions"],
                    "exceptions": len(trial["exceptions_logged"]), "tracebacks": tracebacks,
                    "instrumentation_errors": len(arm["instrumentation_errors"]),
                    "finished_equals_summary": trial["finished_equals_summary_arrived"],
                    "speed_factor_differences": trial["pairing"]["speed_factor_differences"]}
        report[run.name] = r
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "check.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    failures = 0
    for name, r in report.items():
        bad = (len(r["removed_disputed"]) + len(r["kept_disputed"]) + len(r["fallbacks_disputed"])
               + len(r["ordinary_chose_a_dead_end"]) + (not r["counters_agree_with_recorded_calls"])
               + r["run"]["tracebacks"] + r["run"]["exceptions"] + r["run"]["instrumentation_errors"])
        failures += bad
        print(f"\n=== {name} === {'PASS' if not bad else 'FAIL'}")
        print(f"  arm {r['run']['arm']} deaf {r['run']['deaf']}")
        print(f"  filter calls during closures {r['calls']} (responders {r['calls_by_exempt_responders']}, "
              f"of which removed something {r['responder_calls_that_removed']})")
        print(f"  removed: {r['removed_confirmed_dead_end']} confirmed dead ends, {len(r['removed_disputed'])} disputed")
        print(f"  kept:    {r['kept_confirmed_reachable']} confirmed reachable, {len(r['kept_disputed'])} disputed")
        print(f"  fallbacks {r['fallbacks']}: {r['fallbacks_confirmed']} confirmed, {len(r['fallbacks_disputed'])} disputed")
        print(f"  ordinary contested decisions during closures {r['ordinary_contested_decisions_during_closures']}, "
              f"chose a dead end {len(r['ordinary_chose_a_dead_end'])}")
        print(f"  counters {r['counters']} agree with the recorded calls: {r['counters_agree_with_recorded_calls']}")
        x = r["run"]
        print(f"  J {x['J'] if x['J'] is None else round(x['J'], 4)}  finished {x['finished']}/{x['scheduled']} "
              f"({x['completion_pct']}%)  teleports "
              f"{x['teleports']}  travel {x['travel_s_finished_trips']} s (finished trips)  time loss {x['time_loss_s']} s")
        print(f"  responses {x['responses_s']}")
        print(f"  yields {x['yield_actions']}  yield flags {x['yield_flags_broadcast']}  protocol {x['protocol_values']}  "
              f"responder decisions {x['responder_decisions']}")
        print(f"  exceptions {x['exceptions']} tracebacks {x['tracebacks']} instrumentation errors "
              f"{x['instrumentation_errors']} finished=summary {x['finished_equals_summary']} "
              f"speed-factor differences {x['speed_factor_differences']}")
    print(f"\nwritten: {out / 'check.json'}")
    return 0 if not failures else 1


def compare(args) -> int:
    """Are two runs the same run? Every tripinfo record, summary step (SUMO's wall-clock `duration` left
    out), SUMO warning, the incident log, J with its terms, every metric and every counter."""
    a, b = Path(args.first), Path(args.second)
    failures = []

    def same(label, x, y):
        ok = x == y
        if not ok:
            failures.append(label)
        print(f"  {'same     ' if ok else 'DIFFERENT'} {label}" + ("" if ok else f": {str(x)[:100]} | {str(y)[:100]}"))

    ta, tb = (json.loads((r / "trial" / "trial.json").read_text(encoding="utf-8")) for r in (a, b))
    for key in ("objective", "metrics", "coordinator", "learning", "route_stats", "exceptions_logged", "pairing"):
        same(key, ta[key], tb[key])
    for tag, name, ignore in (("tripinfo", "tripinfo.xml", ()), ("step", "summary.xml", ("duration",))):
        ra, rb = (records(r / "trial" / name, tag, ignore=ignore) for r in (a, b))
        same(f"{name} records ({len(ra)} / {len(rb)})", ra, rb)
    wa, wb = (sumo_lines(r / "trial" / "errors.log") for r in (a, b))
    same(f"SUMO warning lines ({len(wa)} / {len(wb)})", wa, wb)

    def incident_log(run):
        path = run / "run.out"
        return [l for l in path.read_text(encoding="utf-8", errors="replace").splitlines()
                if l.startswith("[EMERGENCY]")] if path.is_file() else None

    same("incident activations and closures", incident_log(a), incident_log(b))
    for run in (a, b):
        tracebacks = sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback")
                         for p in (run / "run.err", run / "trial" / "errors.log") if p.is_file())
        same(f"tracebacks in {run.name}", 0, tracebacks)
    print("IDENTICAL" if not failures else "NOT IDENTICAL: " + ", ".join(failures))
    return 0 if not failures else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="one instrumented tuner trial, default weights, in the given arm")
    r.add_argument("--layer", choices=("on", "off"), required=True)
    r.add_argument("--deaf", action="store_true", help="silence every receiver (the no-protocol reference)")
    r.add_argument("--sim-time", type=int, default=3600)
    r.add_argument("--unscored", action="store_true", help="tool check only: no reference, no J (for short runs)")
    r.add_argument("--out", required=True)
    c = sub.add_parser("check", help="re-check the recorded filter calls and choices on the network file")
    c.add_argument("--out", required=True)
    c.add_argument("runs", nargs="+")
    k = sub.add_parser("compare", help="are two runs the same run, bit for bit?")
    k.add_argument("first")
    k.add_argument("second")
    args = parser.parse_args()
    return {"run": live_run, "check": check, "compare": compare}[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
