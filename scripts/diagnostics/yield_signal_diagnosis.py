#!/usr/bin/env python3
"""What does the ordinary-car "yield request" routing signal actually do? (2026-09-14)

Diagnostic only: it changes no production code and writes only into the folder given by --out.

The signal, as implemented:
- Sender (`p3_runtime._broadcast_vehicle_messages`): an ordinary car sets `yield_request` (and
  priority 1) whenever `EmergencyManager.get_nearest_emergency` puts it within 200 m of an ACTIVE
  INCIDENT, by road distance to the incident's closed centre edge. It is not tied to any responder
  or to the car actually yielding. The message names the sender's own next route edge.
- Receiver (`HybridRouter.protocol_weight_for_edge`): a candidate edge named by such a message
  scores 0.3 instead of the neutral 0.5, for ordinary cars (the minimum is taken) and for
  responders (the maximum is taken, so 0.3 survives only if nothing else names the edge).

    python scripts/diagnostics/yield_signal_diagnosis.py history --out DIR RUN_DIR [RUN_DIR ...]
    python scripts/diagnostics/yield_signal_diagnosis.py run (--default-weights | --weights-from T) [--variant V] --out DIR
    python scripts/diagnostics/yield_signal_diagnosis.py compare --runs-dir DIR [--identity-dir DIR] --out DIR

`run` is one real tuning trial (`tune_e3_weights.evaluate_weights`, 3,600 s, tuning seed). At every
decision where a flagged message names a candidate it records the candidates (cars, open-road distance
to the incident, whether a route to the destination exists from them while the closure is on), each
flagged sender (where it is, its next edge, whether that edge heads toward the incident, whether it
is actually yielding), the choice, whether the reroute applied, and what each rule in RULES would have
chosen on the same traffic state. Vehicles whose choice a rule would change are tracked edge by edge.
With `--variant` other than `production`, the run itself uses that rule, emulated here (the broadcast
is corrected just after it is made, before any decision reads it); the production code is untouched.
`compare` puts the runs of one folder side by side per weight set, classifies what each signal-decided
choice did, and checks the observed production run against an uninstrumented one for identity.

Added 2026-09-15:
- `deadends`: an offline count of E3's own dead-end choices during closures.
- `run --variant clean`: the user's clean design, emulated.
  - (a) `yield_request` is only raised while the car is actually yielding, and no receiver's routing
    reads it.
  - (b) Responder broadcasts are unchanged.
  - (c) While a closure is on, E3's candidates are only the edges from which SUMO's router, with the
    car's own vehicle type, reaches its destination. If none does, the list is left as today.
  - At every decision the filter or a production-rule flag touches, it records, on the same state,
    the clean winner, the clean winner without the filter, and today's winner. It also checks that
    its recomputation reproduces E3's actual choice.
- `compare-clean`: puts those runs beside the production runs, response times included.

`history` re-reads the `decisions.json` of existing `e3_component_reachability.py` runs. For every
contested decision it re-scores the recorded candidates with the production scorer twice: as recorded,
and with the yield-request contribution removed (0.3 -> 0.5, exact, because 0.3 can only come
from a yield-request message). Where the winner changes, the signal decided the choice, and the
script records the candidates, values, senders and the choice's relation to the open incident:
- open-road distance from the chosen and from the avoided edge to the closed edge (closure ignored:
  "toward the incident" or "away");
- whether either edge feeds the closed edge directly (its "closed approach");
- the closure-aware free-flow travel time from each to the car's destination (a detour or not);
- the congestion the signal claims ("this edge is busy"): cars on each edge at the decision.
"""

from __future__ import annotations

import argparse
import heapq
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", ""), "tools"))

import sumolib  # noqa: E402
import yaml  # noqa: E402

from p3_hybrid_router import HybridContext, HybridRouter, NEUTRAL_WEIGHT  # noqa: E402

# The router's YIELDING_NEIGHBOUR_WEIGHT before the clean design removed it (2026-09-15). The recorded
# decisions of the earlier runs carry this value, so `history` and `deadends` still need it.
YIELDING_NEIGHBOUR_WEIGHT = 0.3

NET_FILE = REPO_ROOT / "network" / "porto.net.xml"
ROUTE_FILES = [REPO_ROOT / "demand/seeds/porto.rou.20260721.xml", REPO_ROOT / "demand/porto.emergency.rou.xml",
               REPO_ROOT / "demand/porto.dispatch.rou.xml"]
SCENARIO = REPO_ROOT / "scenarios" / "emergency_scenarios.yaml"
TOL_M = 1.0


def incidents() -> dict:
    events = yaml.safe_load(SCENARIO.read_text(encoding="utf-8"))["events"]
    return {str(e["event_id"]): {"start": float(e["start_time"]), "end": float(e["end_time"]),
                                 "closed": list(e["affected_edges"]), "center": e["center_edge"]} for e in events}


def active_incident(events: dict, t: float):
    for event_id, e in events.items():
        if e["start"] <= t < e["end"]:
            return event_id, e
    return None, None


def destinations(wanted: set) -> dict:
    found = {}
    for path in ROUTE_FILES:
        for _, element in ET.iterparse(path, events=("end",)):
            if element.tag == "vehicle":
                vid = element.get("id")
                if vid in wanted:
                    route = element.find("route")
                    if route is not None:
                        found[vid] = route.get("edges").split()[-1]
                element.clear()
    return found


class Graph:
    """Passenger-car road graph from the network file, for offline distances."""

    def __init__(self) -> None:
        self.net = sumolib.net.readNet(str(NET_FILE))
        self.cache = {}

    def edge(self, edge_id):
        try:
            return self.net.getEdge(edge_id)
        except KeyError:
            return None

    def successors(self, edge_id: str) -> list:
        e = self.edge(edge_id)
        if e is None:
            return []
        return sorted(s.getID() for s in e.getOutgoing() if s.allows("passenger"))

    def search(self, start: str, goal: str, closed=(), time_based=False) -> float:
        """Cost from the END of `start` to the END of `goal` (or to the START of `goal` when
        measuring distance to a closed edge). Dijkstra over passenger-allowed edges, `closed`
        excluded. Distance in metres, or free-flow time in seconds."""
        key = (start, goal, tuple(sorted(closed)), time_based)
        if key in self.cache:
            return self.cache[key]
        if start == goal:
            return 0.0
        blocked = set(closed)
        best = {start: 0.0}
        heap = [(0.0, start)]
        result = float("inf")
        while heap:
            cost, current = heapq.heappop(heap)
            if cost > best.get(current, float("inf")):
                continue
            for nxt in self.successors(current):
                if nxt == goal and not time_based:
                    result = min(result, cost)            # reached the start of the goal edge
                    continue
                if nxt in blocked:
                    continue
                e = self.edge(nxt)
                step = e.getLength() / max(e.getSpeed(), 0.1) if time_based else e.getLength()
                new = cost + step
                if nxt == goal:
                    result = min(result, new)
                    continue
                if new < best.get(nxt, float("inf")):
                    best[nxt] = new
                    heapq.heappush(heap, (new, nxt))
            if heap and heap[0][0] >= result:
                break
        self.cache[key] = result
        return result


def rescore(weights: dict, record: dict, protocol: list):
    router = HybridRouter(weights=dict(weights))
    candidates = [(edge, HybridContext(pheromone=record["pheromone"][i], heuristic=record["heuristic"][i],
                                       protocol_weight=protocol[i], bco_exploration=record["bco"][i],
                                       velocity=record["velocity"][i]))
                  for i, edge in enumerate(record["edges"])]
    return router.select_best_edge(candidates)[0]


def cars_on(congestion_value: float) -> int:
    return round(1.0 / congestion_value - 1.0) if congestion_value > 0 else -1


def history(args) -> int:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    events = incidents()
    graph = Graph()
    runs, cases = [], []
    for run_dir in (Path(p) for p in args.runs):
        meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
        decisions = json.loads((run_dir / "decisions.json").read_text(encoding="utf-8"))
        weights = meta["weights"]
        contested = [r for r in decisions if len(r["edges"]) > 1]
        reproduced = sum(1 for r in contested if rescore(weights, r, r["protocol"]) == r["chosen"])
        named = [r for r in contested if any(abs(p - YIELDING_NEIGHBOUR_WEIGHT) < 1e-9 for p in r["protocol"])]
        decided = []
        for r in named:
            no_yield = [NEUTRAL_WEIGHT if abs(p - YIELDING_NEIGHBOUR_WEIGHT) < 1e-9 else p for p in r["protocol"]]
            without = rescore(weights, r, no_yield)
            if without != r["chosen"]:
                decided.append((r, without))
        summary = {"run": run_dir.name, "path": str(run_dir), "weights": weights, "contested": len(contested),
                   "scorer_reproduces": reproduced, "yield_named_a_candidate": len(named),
                   "yield_named_responder_decisions": sum(1 for r in named if r.get("is_emergency_vehicle")),
                   "yield_decided": len(decided)}
        runs.append(summary)
        for r, without in decided:
            cases.append({"run": run_dir.name, "record": r, "without_signal": without})

    dest = destinations({c["record"]["vehicle"] for c in cases})
    for case in cases:
        r, without = case["record"], case["without_signal"]
        event_id, event = active_incident(events, r["t"])
        # A responder is exempt from closures, so for it no edge is closed.
        closed = event["closed"] if event and not r.get("is_emergency_vehicle") else []
        centre = event["center"] if event else None
        i_c, i_w = r["edges"].index(r["chosen"]), r["edges"].index(without)
        provenance = r.get("protocol_provenance") or {}        # not recorded before 2026-09-14
        own = provenance.get("own_message") or {}
        senders = provenance.get("sources") or [["not recorded"] for _ in r["edges"]]
        destination = dest.get(r["vehicle"])

        def relation(edge):
            return {
                "edge": edge,
                "protocol": r["protocol"][r["edges"].index(edge)],
                "yield_senders": [s for s in senders[r["edges"].index(edge)] if s != r["vehicle"]],
                "cars_on_edge": cars_on(r["congestion"][r["edges"].index(edge)]),
                "open_distance_to_closed_edge_m": graph.search(edge, centre) if centre else None,
                "feeds_closed_edge": bool(event) and any(s in event["closed"] for s in graph.successors(edge)),
                "free_flow_s_to_destination_closure_aware": graph.search(edge, destination, closed, True)
                if destination else None,
            }

        chosen, avoided = relation(r["chosen"]), relation(without)
        dc, da = chosen["open_distance_to_closed_edge_m"], avoided["open_distance_to_closed_edge_m"]
        direction = ("toward the incident" if dc + TOL_M < da else "away from the incident" if da + TOL_M < dc
                     else "neither") if dc is not None and da is not None else "no incident"
        tc = chosen["free_flow_s_to_destination_closure_aware"]
        ta = avoided["free_flow_s_to_destination_closure_aware"]
        route_effect = (None if tc is None or ta is None else
                        "shorter route" if tc + 1 < ta else "longer route (detour)" if ta + 1 < tc else "same")
        case.update({
            "t": r["t"], "vehicle": r["vehicle"], "responder": bool(r.get("is_emergency_vehicle")),
            "incident": event_id, "destination": destination, "candidates": r["edges"],
            "protocol": r["protocol"], "own_message_named": own.get("next_edge") if own.get("yield_request") else None,
            "chosen": chosen, "avoided_because_of_signal": avoided, "direction": direction,
            "route_to_destination": route_effect,
            "signal_claim_busier": avoided["cars_on_edge"] > chosen["cars_on_edge"],
        })
        del case["record"]

    (out / "history.json").write_text(json.dumps({"runs": runs, "decided_cases": cases}, indent=1), encoding="utf-8")
    print(f"{'run':44} {'contested':>9} {'scorer ok':>9} {'yield named':>11} {'(responder)':>11} {'decided':>7}")
    for s in runs:
        print(f"{s['run']:44} {s['contested']:>9} {s['scorer_reproduces']:>9} {s['yield_named_a_candidate']:>11} "
              f"{s['yield_named_responder_decisions']:>11} {s['yield_decided']:>7}")
    print("\ndecided cases:")
    for c in cases:
        ch, av = c["chosen"], c["avoided_because_of_signal"]
        print(f"  {c['run'][:28]:28} t={c['t']:6.0f} {c['vehicle']:22} {'RESPONDER ' if c['responder'] else ''}"
              f"inc {c['incident']}: chose {ch['edge']} ({ch['open_distance_to_closed_edge_m']:.0f} m, "
              f"feeds closure {ch['feeds_closed_edge']}, cars {ch['cars_on_edge']}) over {av['edge']} "
              f"({av['open_distance_to_closed_edge_m']:.0f} m, feeds closure {av['feeds_closed_edge']}, cars "
              f"{av['cars_on_edge']}, senders {av['yield_senders']}) -> {c['direction']}; {c['route_to_destination']}")
    print(f"written: {out / 'history.json'}")
    return 0


# ================================================================== dead ends (offline, 2026-09-15)

def deadends(args) -> int:
    """How often does E3 itself pick a road that is a dead end while a closure is on?

    For every contested decision of an ordinary car during an incident window, from the recorded
    decisions of component-reachability runs: a candidate is a dead end when no passenger path leads
    from it to the car's destination without the closed edge (offline, on the network file). Counted:
    decisions offering a dead end next to a reachable road, decisions where E3 chose the dead end, and
    of those how many the yield signal decided (the choice differs without the 0.3 contribution).
    Also: what E3 would choose if dead ends were not candidates ("route relevance kept internal").
    """
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    events = incidents()
    graph = Graph()
    summary, cases = [], []
    for run_dir in (Path(p) for p in args.runs):
        meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
        decisions = json.loads((run_dir / "decisions.json").read_text(encoding="utf-8"))
        weights = meta["weights"]
        in_window = [(r, *active_incident(events, r["t"])) for r in decisions
                     if len(r["edges"]) > 1 and not r.get("is_emergency_vehicle")]
        in_window = [(r, eid, e) for r, eid, e in in_window if e is not None]
        dest = destinations({r["vehicle"] for r, _, _ in in_window})
        offered = chose_dead = by_yield = 0
        for r, event_id, event in in_window:
            destination = dest.get(r["vehicle"])
            if destination is None:
                continue
            dead = [graph.search(edge, destination, event["closed"], True) == float("inf") for edge in r["edges"]]
            if not any(dead) or all(dead):
                continue
            offered += 1
            chosen_dead = dead[r["edges"].index(r["chosen"])]
            if not chosen_dead:
                continue
            chose_dead += 1
            no_yield = [NEUTRAL_WEIGHT if abs(p - YIELDING_NEIGHBOUR_WEIGHT) < 1e-9 else p for p in r["protocol"]]
            decided_by_yield = rescore(weights, r, no_yield) != r["chosen"]
            by_yield += decided_by_yield
            reachable = {k: [v for v, d in zip(r[k], dead) if not d] for k in
                         ("edges", "pheromone", "heuristic", "protocol", "bco", "velocity", "congestion", "progress")}
            # The heuristic is scaled across the candidate set, so it is re-blended over the smaller set
            # with E3's own function; every other component is min-max scaled by the scorer itself.
            from algorithms.e3_core import E3Core
            reachable["heuristic"] = E3Core.blend_heuristic(reachable["congestion"], reachable["progress"])
            internal = rescore(weights, reachable, reachable["protocol"])
            cases.append({"run": run_dir.name, "t": r["t"], "vehicle": r["vehicle"], "incident": event_id,
                          "candidates": r["edges"], "dead_end": dead, "chosen": r["chosen"],
                          "decided_by_yield_signal": decided_by_yield,
                          "choice_without_dead_ends": internal})
        summary.append({"run": run_dir.name, "ordinary_contested_in_incident_windows": len(in_window),
                        "offered_a_dead_end_next_to_a_reachable_road": offered, "e3_chose_the_dead_end": chose_dead,
                        "of_which_the_yield_signal_decided": by_yield})
    (out / "deadends.json").write_text(json.dumps({"runs": summary, "cases": cases}, indent=1), encoding="utf-8")
    for s in summary:
        print(s)
    for c in cases:
        print(f"  {c['run'][:30]:30} t={c['t']:6.0f} {c['vehicle']:14} inc {c['incident']} chose dead end {c['chosen']} "
              f"(yield-decided {c['decided_by_yield_signal']}); without dead ends: {c['choice_without_dead_ends']}")
    return 0


# ================================================================== live runs
#
# Candidate rules, applied to the messages a deciding vehicle hears. The sender-side rules say when an
# ordinary car's broadcast carries the flag; a car whose flag is cleared broadcasts priority 0, as the
# sender code would (a cleared flag with the priority bump left in would score 0.6, an artefact).
#   production        the code as it is
#   no_yield          no ordinary car's flag reaches routing
#   toward_only       the flag only when the car's next edge is closer to the incident than its current
#                     edge (it is heading into the incident zone)
#   yielding_only     the flag only while the car is actually yielding (a yield action in the 3 s before
#                     the broadcast), which is the design document's definition of the field
#   responder_neutral a responder ignores ordinary cars' flags (they score neutral for it)
RULES = ("production", "no_yield", "toward_only", "yielding_only", "responder_neutral",
         "toward_only+responder_neutral", "yielding_only+responder_neutral")


def live_run(args) -> int:
    import dataclasses
    import traci
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    import tune_e3_weights as tuner
    import p3_runtime
    from algorithms import e3_core
    from algorithms.route_utils import vehicle_type_for_routing
    from emergency_manager import get_emergency_manager
    import p3_hybrid_router
    import inspect
    if "speed" not in inspect.signature(p3_runtime.P3RuntimeCoordinator._yield_to_emergency).parameters:
        raise SystemExit("`run` observes the yield action as it was before the unified rule (2026-09-15): brake to "
                         "40%, then lane 0. Production now runs the unified rule (THESIS_CODE_AUDIT_FIXES.md, "
                         "2026-09-15), so it "
                         "would not mean what its evidence says. The analysis modes still read existing runs.")

    out = Path(args.out)
    trial_dir = out / "trial"
    if trial_dir.exists() and any(trial_dir.iterdir()):
        raise SystemExit(f"{trial_dir} already holds files; evidence is never overwritten")
    out.mkdir(parents=True, exist_ok=True)
    variant = args.variant
    # The clean design (user decision 2026-09-15): (a) the flag only while actually yielding, with no
    # routing effect for anyone; (b) responder broadcasts unchanged; (c) while a closure is on, E3
    # keeps only candidates from which the car's own vehicle type can still reach its destination.
    clean = variant == "clean"
    sender_rule = "toward_only" if "toward_only" in variant else "yielding_only" if ("yielding_only" in variant or clean) \
        else "no_yield" if variant == "no_yield" else "production"
    receiver_neutral = "responder_neutral" in variant
    manager = get_emergency_manager()
    # Kept before any patching: the same-state alternatives are always scored with the production rule.
    production_weight = p3_hybrid_router.HybridRouter.protocol_weight_for_edge
    holder = {"coordinator": None, "yields": {}, "errors": [], "contested": 0, "flag_named": 0,
              "orig": {}, "flags_production": 0, "flags_clean": 0, "filter": None, "filter_removed": 0,
              "filter_fallbacks": 0, "reach_queries": 0, "check_mismatches": 0}
    records, tracks = [], {}

    # -------- helpers
    def event_distance(edge, vehicle_type):
        best = float("inf")
        for event_id in list(manager.active_events):
            event = manager.events[event_id]
            d = 0.0 if edge in event.blocked_edges else manager._route_distance(
                edge, event.center_edge, manager._distance_query_type(vehicle_type))
            best = min(best, d)
        return best

    def toward(msg):
        if not msg.next_edge or msg.distance_to_emergency is None:
            return False
        return event_distance(msg.next_edge, msg.vehicle_type) < float(msg.distance_to_emergency)

    def yielding(msg):
        return any(0.0 < msg.timestamp - y <= p3_runtime.YIELD_DURATION_S for y in holder["yields"].get(msg.vehicle_id, ()))

    def with_flag(msg, flag):
        if bool(msg.yield_request) == flag:
            return msg
        return dataclasses.replace(msg, yield_request=flag, priority_level=1 if flag else 0)

    def apply_sender(msgs, rule):
        if rule == "production":
            return list(msgs)
        result = []
        for m in msgs:
            if m.emergency_flag:
                result.append(m)
            elif rule == "no_yield":
                result.append(with_flag(m, False))
            elif rule == "toward_only":
                result.append(with_flag(m, bool(m.yield_request) and toward(m)))
            elif rule == "yielding_only":
                result.append(with_flag(m, yielding(m)))
        return result

    def apply_rule(msgs, rule, is_responder):
        base = rule.replace("+responder_neutral", "").replace("responder_neutral", "production")
        msgs = apply_sender(msgs, base)
        if "responder_neutral" in rule and is_responder:
            msgs = [m if m.emergency_flag else with_flag(m, False) for m in msgs]
        return msgs

    # -------- emulate the run's own rule (never in the production variant)
    real_init = p3_runtime.P3RuntimeCoordinator.__init__

    def init(self, *a, **k):
        real_init(self, *a, **k)
        holder["coordinator"] = self

    p3_runtime.P3RuntimeCoordinator.__init__ = init

    real_yield = p3_runtime.P3RuntimeCoordinator._yield_to_emergency

    def record_yield(self, vehicle_id, edge_id, speed, sim_time):
        before = self.last_yield.get(vehicle_id)
        real_yield(self, vehicle_id, edge_id, speed, sim_time)
        if self.last_yield.get(vehicle_id) != before:
            holder["yields"].setdefault(vehicle_id, []).append(sim_time)

    p3_runtime.P3RuntimeCoordinator._yield_to_emergency = record_yield

    if sender_rule in ("toward_only", "yielding_only", "no_yield"):
        real_broadcast = p3_runtime.P3RuntimeCoordinator._broadcast_vehicle_messages

        def broadcast(self, sim_time):
            real_broadcast(self, sim_time)
            try:
                for entry in self.protocol_state.neighbors.values():
                    m = entry.message
                    if m.timestamp == sim_time and not m.emergency_flag:
                        new = apply_sender([m], sender_rule)[0]
                        if clean:            # keep the production version for the same-state comparison
                            holder["orig"][m.vehicle_id] = m
                            holder["flags_production"] += bool(m.yield_request)
                            holder["flags_clean"] += bool(new.yield_request)
                        entry.message = new
            except Exception as exc:          # recorded, and fails the run at the end
                holder["errors"].append(f"broadcast emulation: {exc!r}")

        p3_runtime.P3RuntimeCoordinator._broadcast_vehicle_messages = broadcast

    if receiver_neutral or clean:
        real_weight = p3_hybrid_router.HybridRouter.protocol_weight_for_edge

        def weight(self, edge_id, messages, is_emergency_vehicle=False, corridor_edge=None):
            messages = list(messages)
            # clean (a): a yield flag says nothing about any edge, for any receiver. A flagged message
            # then scores like a plain one (0.5), which equals leaving it out.
            if clean or is_emergency_vehicle:
                messages = [m if m.emergency_flag else with_flag(m, False) for m in messages]
            return real_weight(self, edge_id, messages, is_emergency_vehicle=is_emergency_vehicle,
                               corridor_edge=corridor_edge)

        p3_hybrid_router.HybridRouter.protocol_weight_for_edge = weight

    # -------- clean (c): the exact destination-reachability filter, only while a closure is on
    from algorithms import aco_pso, route_utils
    reach_cache = {}

    def reaches(edge, destination, vtype):
        """SUMO's own router, with the car's own vehicle type and today's lane permissions: the check
        `build_destination_valid_route` makes after choosing, made before choosing instead."""
        if edge == destination:
            return True
        key = (edge, destination, vtype, route_utils.blocked_edges())
        if key not in reach_cache:
            holder["reach_queries"] += 1
            try:
                found = traci.simulation.findRoute(edge, destination, vtype, -1.0, 0)
                reach_cache[key] = bool(found.edges) and found.edges[0] == edge and found.edges[-1] == destination
            except traci.TraCIException:
                reach_cache[key] = False
        return reach_cache[key]

    if clean:
        real_outgoing = e3_core.get_outgoing_edges

        def outgoing(edge_id, count_stats=True, veh_id=None, exclude_blocked=None):
            edges = real_outgoing(edge_id, count_stats=count_stats, veh_id=veh_id, exclude_blocked=exclude_blocked)
            holder["filter"] = None
            if not (route_utils.blocked_edges() and veh_id and len(edges) >= 2):
                return edges
            try:
                destination = traci.vehicle.getRoute(veh_id)[-1]
                kept = [e for e in edges if reaches(e, destination, vehicle_type_for_routing(veh_id))]
            except Exception as exc:
                holder["errors"].append(f"filter {veh_id}: {exc!r}")
                return edges
            if not kept:                       # nothing reaches the destination: today's behaviour
                holder["filter_fallbacks"] += 1
                holder["filter"] = {"vehicle": veh_id, "full": list(edges), "kept": list(edges), "fallback": True}
                return edges
            holder["filter"] = {"vehicle": veh_id, "full": list(edges), "kept": kept, "fallback": False}
            if len(kept) < len(edges):
                holder["filter_removed"] += 1
            return kept

        e3_core.get_outgoing_edges = outgoing

    def pure_contexts(self, edges, destination, protocol_values):
        """The contexts `score_candidates` would build, without its one side effect (noting edges for
        the next PSO advance), so a counterfactual can be scored without touching the run."""
        heuristics = e3_core.E3Core.blend_heuristic(
            [e3_core.E3Core.congestion_suitability(e) for e in edges],
            [self.progress_suitability(e, destination) for e in edges])
        return [(e, p3_hybrid_router.HybridContext(
            pheromone=aco_pso.pheromone.get(e, 1.0), heuristic=h, protocol_weight=p,
            bco_exploration=1.0 - self.router.scout_memory.get(e, 0.0), velocity=aco_pso.velocity.get(e, 0.0)))
            for e, h, p in zip(edges, heuristics, protocol_values)]

    def evaluate_clean(self, vehicle_id, outgoing_edges, destination, messages, is_resp, corridor):
        """Before E3 chooses (the choice updates BCO memory): the same-state winners of the clean
        design, of the clean design without the filter, and of today's rules."""
        info = holder["filter"] if holder["filter"] and holder["filter"]["vehicle"] == vehicle_id else None
        full = info["full"] if info else list(outgoing_edges)
        if len(full) < 2:
            return None
        holder["contested"] += 1
        msgs = list(messages)
        prod_msgs = []
        for m in msgs:
            o = holder["orig"].get(m.vehicle_id)
            prod_msgs.append(o if o is not None and o.timestamp == m.timestamp else m)
        clean_msgs = [m if m.emergency_flag else with_flag(m, False) for m in msgs]
        filtered = bool(info) and len(info["kept"]) < len(full)
        flag_named = any(m.yield_request and not m.emergency_flag and m.next_edge in full for m in prod_msgs)
        if not (filtered or flag_named):
            return None

        def best(edges, message_list):
            values = [production_weight(self.router, e, message_list, is_emergency_vehicle=is_resp,
                                        corridor_edge=corridor) for e in edges]
            return self.router.select_best_edge(pure_contexts(self, edges, destination, values))[0], values

        kept = info["kept"] if info else full
        expect_clean, _ = best(kept, clean_msgs)
        clean_nofilter, p_clean = best(full, clean_msgs)
        production, p_prod = best(full, prod_msgs)
        vtype = vehicle_type_for_routing(vehicle_id)
        blocked = set(route_utils.blocked_edges())
        try:
            remaining = traci.vehicle.getRoute(vehicle_id)[traci.vehicle.getRouteIndex(vehicle_id):]
            crosses = any(e in blocked for e in remaining)
        except traci.TraCIException:
            crosses = None
        return {"t": traci.simulation.getTime(), "vehicle": vehicle_id, "responder": is_resp,
                "on": traci.vehicle.getRoadID(vehicle_id), "destination": destination,
                "route_crosses_closure": crosses, "active_incidents": sorted(str(x) for x in manager.active_events),
                "candidates": [{"edge": e, "reachable": reaches(e, destination, vtype), "kept": e in kept,
                                "cars": traci.edge.getLastStepVehicleNumber(e),
                                "protocol_clean": pc, "protocol_production": pp}
                               for e, pc, pp in zip(full, p_clean, p_prod)],
                "filtered": filtered, "fallback": bool(info and info["fallback"]), "production_flag_named": flag_named,
                "expected_clean": expect_clean, "winners": {"clean_without_filter": clean_nofilter,
                                                            "production": production}}

    # -------- observe every decision (all variants)
    real_score = e3_core.E3Core.score_candidates
    real_select = e3_core.E3Core.select_edge
    real_decide = e3_core.E3Core.decide
    real_decay = e3_core.E3Core.decay
    buffer = {}

    def score(self, vehicle_id, outgoing_edges, destination_edge, messages, is_emergency_vehicle=False,
              corridor_edge=None):
        messages = list(messages)
        scored = real_score(self, vehicle_id, outgoing_edges, destination_edge, messages,
                            is_emergency_vehicle=is_emergency_vehicle, corridor_edge=corridor_edge)
        buffer.update(scored=scored, messages=messages, is_emergency=bool(is_emergency_vehicle),
                      corridor=corridor_edge, destination=destination_edge)
        return scored

    def evaluate(self, vehicle_id, chosen):
        scored, msgs = buffer["scored"], buffer["messages"]
        edges = [e for e, _ in scored]
        if len(edges) < 2:
            return None
        holder["contested"] += 1
        flagged = {m.vehicle_id for m in apply_sender(msgs, "production") if m.yield_request and m.next_edge in edges}
        flagged |= {m.vehicle_id for m in apply_sender(msgs, "yielding_only") if m.yield_request and m.next_edge in edges}
        if not flagged:
            return None
        holder["flag_named"] += 1
        t = traci.simulation.getTime()
        is_resp = buffer["is_emergency"]
        winners, protocols = {}, {}
        for rule in RULES:
            ruled = apply_rule(msgs, rule, is_resp)
            values = [production_weight(self.router, e, ruled, is_emergency_vehicle=is_resp,
                                        corridor_edge=buffer["corridor"]) for e in edges]
            contexts = [(e, dataclasses.replace(c, protocol_weight=v)) for (e, c), v in zip(scored, values)]
            winners[rule] = self.router.select_best_edge(contexts)[0]
            protocols[rule] = values
        vtype = vehicle_type_for_routing(vehicle_id)
        candidates = []
        for e, c in scored:
            try:
                found = traci.simulation.findRoute(e, buffer["destination"], vtype, -1.0, 0)
                route_s = float(found.travelTime) if found.edges and found.edges[-1] == buffer["destination"] else None
            except traci.TraCIException:
                route_s = None
            candidates.append({
                "edge": e, "pheromone": c.pheromone, "heuristic": c.heuristic, "bco": c.bco_exploration,
                "velocity": c.velocity, "cars": traci.edge.getLastStepVehicleNumber(e),
                "open_distance_to_incident_m": event_distance(e, traci.vehicle.getTypeID(vehicle_id)),
                "route_to_destination_s": route_s, "dead_end": route_s is None,
                "flag_senders": sorted(m.vehicle_id for m in msgs if m.yield_request and m.next_edge == e
                                       and not m.emergency_flag)})
        blocked = set(manager.blocked_edges)

        def crosses_closure(v):
            try:
                return any(e in blocked for e in traci.vehicle.getRoute(v)[traci.vehicle.getRouteIndex(v):])
            except traci.TraCIException:
                return None

        senders = []
        for m in msgs:
            if m.vehicle_id in flagged:
                senders.append({"sender": m.vehicle_id, "type": m.vehicle_type, "on": m.position_edge,
                                "next_edge": m.next_edge, "age_s": t - m.timestamp, "speed": m.speed,
                                "flag_in_run": bool(m.yield_request),
                                "distance_now_m": m.distance_to_emergency,
                                "distance_next_m": event_distance(m.next_edge, m.vehicle_type) if m.next_edge else None,
                                "heading_toward": toward(m), "yielding": yielding(m),
                                "sender_route_crosses_closure": crosses_closure(m.vehicle_id)})
        record = {"t": t, "vehicle": vehicle_id, "responder": is_resp, "on": traci.vehicle.getRoadID(vehicle_id),
                  "destination": buffer["destination"], "route_crosses_closure": crosses_closure(vehicle_id),
                  "chosen": chosen, "winners": winners, "protocol": protocols, "candidates": candidates,
                  "senders": senders, "active_incidents": sorted(str(x) for x in manager.active_events)}
        records.append(record)
        if winners["no_yield"] != chosen or any(w != chosen for w in winners.values()):
            tracks.setdefault(vehicle_id, [])
        return record

    def select(self, vehicle_id, outgoing, destination, messages, is_emergency_vehicle=False, corridor_edge=None):
        buffer.clear()
        messages = list(messages)
        pre = None
        if clean:
            try:
                pre = evaluate_clean(self, vehicle_id, outgoing, destination, messages,
                                     bool(is_emergency_vehicle), corridor_edge)
            except Exception as exc:
                holder["errors"].append(f"evaluate_clean {vehicle_id}: {exc!r}")
        chosen, n = real_select(self, vehicle_id, outgoing, destination, messages,
                                is_emergency_vehicle=is_emergency_vehicle, corridor_edge=corridor_edge)
        if clean:
            if pre is not None:
                pre["chosen"] = chosen
                if pre["expected_clean"] != chosen:
                    holder["check_mismatches"] += 1
                records.append(pre)
                if pre["winners"]["production"] != chosen or pre["winners"]["clean_without_filter"] != chosen:
                    tracks.setdefault(vehicle_id, [])
                buffer["record"] = pre
            return chosen, n
        try:
            buffer["record"] = evaluate(self, vehicle_id, chosen) if buffer.get("scored") else None
        except Exception as exc:
            holder["errors"].append(f"evaluate {vehicle_id}: {exc!r}")
        return chosen, n

    def decide(self, vehicle_id, sim_time, messages=None, is_emergency_vehicle=False, corridor_edge=None):
        report = real_decide(self, vehicle_id, sim_time, messages=messages,
                             is_emergency_vehicle=is_emergency_vehicle, corridor_edge=corridor_edge)
        record = buffer.pop("record", None)
        if record is not None and record["vehicle"] == vehicle_id:
            record["applied"] = report.get("applied")
            record["reason"] = report.get("reason")
        return report

    def decay(self, seconds=1.0):
        real_decay(self, seconds)
        try:
            t = traci.simulation.getTime()
            # Chaos control (--nudge-time): one small physical perturbation that touches no routing
            # decision. The lowest-id ordinary car moving faster than 5 m/s is slowed to half its
            # speed over 2 s, once, at the first step at or after the given time.
            if args.nudge_time is not None and "nudge" not in holder and t >= args.nudge_time:
                for v in sorted(traci.vehicle.getIDList()):
                    speed = traci.vehicle.getSpeed(v)
                    if v.startswith("normal_") and speed > 5.0:
                        traci.vehicle.slowDown(v, speed * 0.5, 2.0)
                        holder["nudge"] = {"vehicle": v, "time": t, "speed": speed, "road": traci.vehicle.getRoadID(v)}
                        break
            present = set(traci.vehicle.getIDList()) if tracks else set()
            for v, path in tracks.items():
                road = traci.vehicle.getRoadID(v) if v in present else "(gone)"
                if not path or path[-1][1] != road:
                    path.append((t, road))
        except Exception as exc:
            holder["errors"].append(f"track: {exc!r}")

    e3_core.E3Core.score_candidates = score
    e3_core.E3Core.select_edge = select
    e3_core.E3Core.decide = decide
    e3_core.E3Core.decay = decay

    # -------- the real trial function
    if args.default_weights:
        weights = {name: float(p3_hybrid_router.DEFAULT_WEIGHTS[name]) for name in tuner.WEIGHT_NAMES}
    else:
        weights = json.loads(Path(args.weights_from).read_text(encoding="utf-8"))["weights_requested"]
    reference = tuner.load_reference(Path(args.reference), 20260721, args.sim_time)
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), args.sim_time, 20260721,
                               trial_dir=trial_dir, reference=reference)
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    summary = {"variant": variant, "weights": weights, "rejected": rejected, "instrumentation_errors": holder["errors"],
               "contested_decisions": holder["contested"], "decisions_with_a_flag_naming_a_candidate": holder["flag_named"],
               "yield_actions": sum(len(v) for v in holder["yields"].values()), "nudge": holder.get("nudge"),
               "clean": {"flags_broadcast_production_rule": holder["flags_production"],
                         "flags_broadcast_clean_rule": holder["flags_clean"],
                         "decisions_where_the_filter_removed_a_candidate": holder["filter_removed"],
                         "filter_fallbacks_nothing_reachable": holder["filter_fallbacks"],
                         "reachability_queries": holder["reach_queries"],
                         "recorded_decisions": len(records) if clean else None,
                         "recomputation_mismatches": holder["check_mismatches"]} if clean else None}
    (out / "records.json").write_text(json.dumps(records, indent=1, default=str), encoding="utf-8")
    (out / "tracks.json").write_text(json.dumps(tracks, indent=1), encoding="utf-8")
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(json.dumps(summary))
    return 0 if rejected is None and not holder["errors"] else 1


# ================================================================== comparison of the live runs

def consequence(record: dict, alternative: str) -> str:
    """What the signal's choice did, against the choice the car would have made without it."""
    by_edge = {c["edge"]: c for c in record["candidates"]}
    chosen, other = by_edge[record["chosen"]], by_edge[alternative]
    trapped = bool(record.get("route_crosses_closure"))
    if chosen["dead_end"] and not other["dead_end"]:
        return "HURT: kept a trapped car on its route through the closure" if trapped else \
            "no effect: chose the dead end, so kept its already valid route"
    if other["dead_end"] and not chosen["dead_end"]:
        return "HELPED: freed a trapped car (avoided the dead end)" if trapped else \
            "switched to another valid route instead of keeping its own"
    if chosen["route_to_destination_s"] is None or other["route_to_destination_s"] is None:
        return "no route either way"
    delta = chosen["route_to_destination_s"] - other["route_to_destination_s"]
    return ("route swap, chosen faster" if delta < -1 else "route swap, chosen slower" if delta > 1
            else "route swap, same time")


def trip_outcomes(trial_dir: Path, vehicles: set) -> dict:
    found = {}
    for element in ET.parse(trial_dir / "tripinfo.xml").getroot().iter("tripinfo"):
        if element.get("id") in vehicles:
            found[element.get("id")] = {"duration": float(element.get("duration")),
                                        "time_loss": float(element.get("timeLoss")),
                                        "arrived": float(element.get("arrival")) >= 0 and not element.get("vaporized"),
                                        "vaporized": element.get("vaporized")}
    return found


def compare(args) -> int:
    root = Path(args.runs_dir)
    out = {"weights": {}}
    for weights in ("def", "t0", "t1"):
        runs = {p.name[len(weights) + 1:]: p for p in sorted(root.glob(f"{weights}_*")) if (p / "summary.json").is_file()}
        if "production" not in runs:
            continue
        production = json.loads((runs["production"] / "records.json").read_text(encoding="utf-8"))
        section = {"runs": {}, "same_state": {}, "signal_decided": [], "identity": None}
        for name, path in runs.items():
            summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
            trial = json.loads((path / "trial" / "trial.json").read_text(encoding="utf-8"))
            m = trial["metrics"]
            tracebacks = sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback")
                             for p in (path / "run.err", path / "trial" / "errors.log") if p.is_file())
            section["runs"][name] = {
                "J": trial["objective"]["J"], "finished": m["finished"], "scheduled": m["vehicles_scheduled"],
                "teleports": m["teleports"], "never_inserted": m["never_inserted"],
                "travel_s": round(m["travel_s"], 2), "time_loss_s": round(m["time_loss_s"], 2),
                "responses_s": {e: r["value_s"] for e, r in m["responses"].items()},
                "reroutes": trial["coordinator"]["reroute_executed"], "yield_actions": summary["yield_actions"],
                "flag_named_decisions": summary["decisions_with_a_flag_naming_a_candidate"],
                "contested": summary["contested_decisions"], "rejected": summary["rejected"],
                "instrumentation_errors": len(summary["instrumentation_errors"]), "tracebacks": tracebacks,
                "exceptions": len(trial["exceptions_logged"])}
        # Same traffic state: what each rule would have chosen at every flagged decision of the production run.
        for rule in RULES[1:]:
            changed = [r for r in production if r["winners"][rule] != r["chosen"]]
            classes = {}
            for r in changed:
                label = consequence(r, r["winners"][rule])
                classes[label] = classes.get(label, 0) + 1
            section["same_state"][rule] = {"changed_vs_production": len(changed),
                                           "responder_decisions_changed": sum(r["responder"] for r in changed),
                                           "production_choice_relative_to_rule": classes}
        for r in production:
            if r["winners"]["no_yield"] != r["chosen"]:
                section["signal_decided"].append({
                    "t": r["t"], "vehicle": r["vehicle"], "responder": r["responder"], "on": r["on"],
                    "incidents": r["active_incidents"], "route_crosses_closure": r["route_crosses_closure"],
                    "chose": r["chosen"], "without_signal": r["winners"]["no_yield"], "applied": r.get("applied"),
                    "consequence": consequence(r, r["winners"]["no_yield"]),
                    "kept_by": [rule for rule in RULES[1:] if r["winners"][rule] == r["chosen"]],
                    "senders": [(s["sender"], s["on"], s["next_edge"], s["heading_toward"], s["yielding"])
                                for s in r["senders"] if s["flag_in_run"]],
                    "candidates": [(c["edge"], c["dead_end"], c["cars"], round(c["open_distance_to_incident_m"], 0)
                                    if c["open_distance_to_incident_m"] != float("inf") else None,
                                    c["flag_senders"]) for c in r["candidates"]]})
        affected = {d["vehicle"] for d in section["signal_decided"]}
        section["affected_vehicle_trips"] = {name: trip_outcomes(path / "trial", affected) for name, path in runs.items()}
        if args.identity_dir:
            reference = Path(args.identity_dir) / f"{weights}_production" / "trial"
            if reference.is_dir():
                mine = runs["production"] / "trial"
                a = [dict(e.attrib) for e in ET.parse(reference / "tripinfo.xml").getroot().iter("tripinfo")]
                b = [dict(e.attrib) for e in ET.parse(mine / "tripinfo.xml").getroot().iter("tripinfo")]
                ja = json.loads((reference / "trial.json").read_text(encoding="utf-8"))["objective"]["J"]
                section["identity"] = {"against": str(reference), "J_equal": ja == section["runs"]["production"]["J"],
                                       "tripinfo_records_differing": sum(1 for x, y in zip(a, b) if x != y) + abs(len(a) - len(b))}
        out["weights"][weights] = section
    Path(args.out).mkdir(parents=True, exist_ok=True)
    (Path(args.out) / "comparison.json").write_text(json.dumps(out, indent=1, default=str), encoding="utf-8")
    for weights, section in out["weights"].items():
        print(f"\n=== weights {weights} === identity with the uninstrumented production run: {section['identity']}")
        for name, s in section["runs"].items():
            print(f"  {name:36} J {s['J']:.4f} finished {s['finished']}/{s['scheduled']} ({100 * s['finished'] / s['scheduled']:.2f}%) "
                  f"teleports {s['teleports']} responses {s['responses_s']} reroutes {s['reroutes']} flagged decisions "
                  f"{s['flag_named_decisions']} errors {s['instrumentation_errors']} tracebacks {s['tracebacks']}")
        print("  same traffic state (production run): decisions each rule would change")
        for rule, s in section["same_state"].items():
            print(f"    {rule:34} {s['changed_vs_production']:3} (responders {s['responder_decisions_changed']}) {s['production_choice_relative_to_rule']}")
        print("  decisions the signal decided in production:")
        for d in section["signal_decided"]:
            print(f"    t={d['t']:6.0f} {d['vehicle']:18} inc {d['incidents']} trapped {d['route_crosses_closure']} chose {d['chose']} "
                  f"(without {d['without_signal']}) applied {d['applied']}: {d['consequence']}; kept by {d['kept_by']}")
    print(f"\nwritten: {Path(args.out) / 'comparison.json'}")
    return 0


def compare_clean(args) -> int:
    """Each clean-design run beside the production run of the same weights (2026-09-15)."""
    clean_dir, prod_dir = Path(args.clean_dir), Path(args.production_dir)
    report = {}

    def steps(trial):
        return [{k: v for k, v in s.attrib.items() if k != "duration"}
                for s in ET.parse(trial / "summary.xml").getroot().iter("step")]

    for weights in ("def", "t0", "t1"):
        c_run, p_run = clean_dir / f"{weights}_clean", prod_dir / f"{weights}_production"
        if not (c_run / "summary.json").is_file() or not (p_run / "summary.json").is_file():
            continue
        section = {"runs": {}}
        for name, run in (("production", p_run), ("clean", c_run)):
            trial = json.loads((run / "trial" / "trial.json").read_text(encoding="utf-8"))
            summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
            m = trial["metrics"]
            tracebacks = sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback")
                             for p in (run / "run.err", run / "trial" / "errors.log") if p.is_file())
            section["runs"][name] = {
                "J": trial["objective"]["J"], "finished": m["finished"], "scheduled": m["vehicles_scheduled"],
                "completion_pct": round(100 * m["finished"] / m["vehicles_scheduled"], 2),
                "teleports": m["teleports"], "never_inserted": m["never_inserted"],
                "travel_s_finished_trips": round(m["travel_s"], 1), "time_loss_s": round(m["time_loss_s"], 1),
                "responses_s": {e: r["value_s"] for e, r in m["responses"].items()},
                "reroutes": trial["coordinator"]["reroute_executed"], "yield_actions": summary["yield_actions"],
                "clean_counters": summary.get("clean"), "errors": len(summary["instrumentation_errors"]),
                "exceptions": len(trial["exceptions_logged"]), "tracebacks": tracebacks}
        sp, sc = steps(p_run / "trial"), steps(c_run / "trial")
        section["first_differing_step"] = next((x["time"] for x, y in zip(sp, sc) if x != y), None)
        recs = json.loads((c_run / "records.json").read_text(encoding="utf-8"))
        # The counterfactual winners recorded inside a clean run are NOT used for claims: once the filter
        # has removed a dead end, E3's BCO memory no longer adds +0.05 to it (only candidates are
        # updated), so the clean run's internal state differs from production's and a recomputed
        # "today's rules" choice drifts toward the dead end. What each run actually did is compared
        # instead: the clean record against the production run's own record of the same car and time.
        prod_recs = {(r["vehicle"], r["t"]): r for r in
                     json.loads((p_run / "records.json").read_text(encoding="utf-8"))}
        side_by_side = []
        for r in recs:
            by_edge = {c["edge"]: c for c in r["candidates"]}
            p = prod_recs.get((r["vehicle"], r["t"]))
            side_by_side.append({
                "t": r["t"], "vehicle": r["vehicle"], "trapped": r["route_crosses_closure"],
                "incidents": r["active_incidents"], "filtered": r["filtered"],
                "clean": (r["chosen"], by_edge[r["chosen"]]["reachable"], r.get("applied")),
                "production": (p["chosen"], not next(c["dead_end"] for c in p["candidates"] if c["edge"] == p["chosen"]),
                               p.get("applied")) if p else "not recorded (no flag named a candidate there)",
                "before_first_divergence": section["first_differing_step"] is None
                or r["t"] < float(section["first_differing_step"])})
        section["decisions"] = side_by_side
        section["same_state"] = {"recorded_decisions": len(recs),
                                 "filter_removed_a_candidate": sum(r["filtered"] for r in recs),
                                 "clean_chose_a_dead_end": sum(not s["clean"][1] for s in side_by_side),
                                 "responder_decisions_recorded": sum(r["responder"] for r in recs)}
        report[weights] = section
    Path(args.out).mkdir(parents=True, exist_ok=True)
    (Path(args.out) / "comparison_clean.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    for weights, s in report.items():
        print(f"\n=== {weights} === first differing step: {s['first_differing_step']}")
        for name, r in s["runs"].items():
            print(f"  {name:10} J {r['J']:.4f}  finished {r['finished']}/{r['scheduled']} ({r['completion_pct']}%)  teleports {r['teleports']}  "
                  f"responses 404/401/405 {r['responses_s']}  travel {r['travel_s_finished_trips']} s (finished trips)  "
                  f"time loss {r['time_loss_s']} s  reroutes {r['reroutes']}  yields {r['yield_actions']}  errors {r['errors']} tracebacks {r['tracebacks']}")
        print(f"  clean counters: {s['runs']['clean']['clean_counters']}")
        print(f"  decisions: {s['same_state']}")
        for d in s["decisions"]:
            print(f"    t={d['t']:6.0f} {d['vehicle']:14} inc {d['incidents']} trapped {d['trapped']} filtered {d['filtered']}: "
                  f"clean (chose, reachable, applied) {d['clean']} | production {d['production']}"
                  + ("" if d["before_first_divergence"] else "  [after the runs diverged]"))
    print(f"\nwritten: {Path(args.out) / 'comparison_clean.json'}")
    return 0


def records(path: Path, tag: str, ignore=()) -> list:
    """Every `tag` element's attributes (minus `ignore`) and children, in file order."""
    return [({k: v for k, v in e.attrib.items() if k not in ignore}, [(c.tag, dict(c.attrib)) for c in e])
            for e in ET.parse(path).getroot().iter(tag)]


def sumo_lines(path: Path) -> list:
    return [line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.startswith(("Warning", "Error"))]


def reproduce_check(args) -> int:
    """Is a production run of the clean design the same run as the emulation that preceded it?

    Compared exactly:
    - J with its terms, every metric, and the coordinator and outcome counters;
    - every tripinfo record, every summary step (SUMO's wall-clock `duration` excluded) and every SUMO
      warning;
    - the route counters the two share.

    The production run's three new filter counters are listed, and checked against the emulation's own
    count of filtered decisions and fallbacks.
    """
    a, b = Path(args.emulated), Path(args.production)
    failures = []

    def same(label, x, y):
        ok = x == y
        if not ok:
            failures.append(label)
        print(f"  {'same     ' if ok else 'DIFFERENT'} {label}" + ("" if ok else f": {str(x)[:100]} | {str(y)[:100]}"))

    ta, tb = (json.loads((r / "trial" / "trial.json").read_text(encoding="utf-8")) for r in (a, b))
    same("J", ta["objective"]["J"], tb["objective"]["J"])
    same("objective terms", ta["objective"], tb["objective"])
    same("metrics", ta["metrics"], tb["metrics"])
    same("coordinator counters", ta["coordinator"], tb["coordinator"])
    same("outcome counters", ta["learning"]["outcomes"], tb["learning"]["outcomes"])
    same("pheromone table entries", ta["learning"]["pheromone_table_entries"], tb["learning"]["pheromone_table_entries"])
    same("exceptions logged", ta["exceptions_logged"], tb["exceptions_logged"])
    shared = sorted(set(ta["route_stats"]) & set(tb["route_stats"]))
    same("route counters both runs have", {k: ta["route_stats"][k] for k in shared}, {k: tb["route_stats"][k] for k in shared})
    new = {k: tb["route_stats"][k] for k in sorted(set(tb["route_stats"]) - set(ta["route_stats"]))}
    print(f"  new in production: {new}")
    ra, rb = (records(r / "trial" / "tripinfo.xml", "tripinfo") for r in (a, b))
    same(f"tripinfo records ({len(ra)} / {len(rb)})", ra, rb)
    sa, sb = (records(r / "trial" / "summary.xml", "step", ignore=("duration",)) for r in (a, b))
    same(f"summary steps ({len(sa)} / {len(sb)})", sa, sb)
    wa, wb = (sumo_lines(r / "trial" / "errors.log") for r in (a, b))
    same(f"SUMO warning lines ({len(wa)} / {len(wb)})", wa, wb)
    emulated = json.loads((a / "summary.json").read_text(encoding="utf-8")).get("clean") or {}
    if emulated and new:
        same("filtered decisions: emulation vs production counter",
             emulated.get("decisions_where_the_filter_removed_a_candidate"), new.get("closure_filtered_decisions"))
        same("fallbacks: emulation vs production counter",
             emulated.get("filter_fallbacks_nothing_reachable"), new.get("closure_filter_fallbacks"))
    tracebacks = sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback")
                     for p in (b / "run.err", b / "trial" / "errors.log") if p.is_file())
    same("tracebacks in the production run", 0, tracebacks)
    print(f"{'IDENTICAL' if not failures else 'NOT IDENTICAL: ' + ', '.join(failures)}")
    return 0 if not failures else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    h = sub.add_parser("history", help="re-analyse existing component-reachability runs")
    h.add_argument("--out", required=True)
    h.add_argument("runs", nargs="+")
    r = sub.add_parser("run", help="one instrumented 3,600 s trial; --variant emulates a candidate rule")
    r.add_argument("--out", required=True)
    r.add_argument("--variant", default="production", choices=RULES + ("clean",))
    source = r.add_mutually_exclusive_group(required=True)
    source.add_argument("--weights-from")
    source.add_argument("--default-weights", action="store_true")
    r.add_argument("--sim-time", type=int, default=3600)
    r.add_argument("--nudge-time", type=float, default=None,
                   help="chaos control: brake one uninvolved car to half speed for 2 s at this time")
    r.add_argument("--reference", default=str(REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"))
    c = sub.add_parser("compare", help="compare the live runs of a folder, per weight set")
    c.add_argument("--runs-dir", required=True)
    c.add_argument("--identity-dir", default=None, help="folder with <weights>_production runs of the same code")
    c.add_argument("--out", required=True)
    d = sub.add_parser("deadends", help="how often E3 itself picks a dead end during a closure (offline)")
    d.add_argument("--out", required=True)
    d.add_argument("runs", nargs="+")
    cc = sub.add_parser("compare-clean", help="clean-design runs beside the production runs of the same weights")
    cc.add_argument("--clean-dir", required=True)
    cc.add_argument("--production-dir", required=True)
    cc.add_argument("--out", required=True)
    rc = sub.add_parser("reproduce-check", help="is a production clean-design run the emulated run, bit for bit?")
    rc.add_argument("emulated")
    rc.add_argument("production")
    args = parser.parse_args()
    return {"history": history, "run": live_run, "compare": compare, "deadends": deadends,
            "compare-clean": compare_clean, "reproduce-check": reproduce_check}[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
