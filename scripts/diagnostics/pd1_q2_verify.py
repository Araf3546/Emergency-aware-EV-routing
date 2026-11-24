#!/usr/bin/env python3
"""PD-1 question 2 verification (2026-09-13): ambient emergency vehicles keep no road permission
unique to emergency vehicles; dispatched responders keep the closure exemption.

    python scripts/diagnostics/pd1_q2_verify.py all --out results/pd1_q2_verification_2026-09-13

Short runs only. Two parts, both with the rule on and, for contrast, off
(`EmergencyManager.restrict_ambient_road_access`):

1. `physics` - the real network, otherwise empty. One vehicle per run on responder 404's own
   route, whose last edge is closed from t = 1 to the end; teleporting is disabled so a blocked
   vehicle stays blocked. Run for the responder (fire_truck), an ambient vehicle
   (emergency_fire_truck) and a car (normal_passenger). Also: the ambient type's parameters
   before and after the switch, and its incident distances under both rules.
2. `run` - the real scenario at the tuning seed, 1,250 s (incident 404 closes 600-900 s, incident
   401 closes 900-1,230 s), for the baseline, P3 layer ON and P3 layer OFF, each with the rule on
   and off: six runs, launched in parallel by `all`. Every step logs every vehicle that ENTERS a
   closed edge while it is closed. Nothing can differ before the first closure at t = 600, so the
   two runs of each arm must agree exactly up to then; that is the check that the rule changes
   nothing else.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402

SEED = 20260721
DURATION = 1250
FIRST_CLOSURE = 600
COMMITTED_S = 3        # an entry this soon after a closure began was already crossing the junction
ARMS = ("baseline", "p3on", "p3off")
TYPE_GETTERS = ("getLength", "getMaxSpeed", "getAccel", "getDecel", "getEmergencyDecel",
                "getApparentDecel", "getImperfection", "getTau", "getMinGap", "getSpeedFactor",
                "getSpeedDeviation", "getEmissionClass", "getWidth", "getHeight", "getShapeClass",
                "getMaxSpeedLat", "getMinGapLat", "getLateralAlignment", "getActionStepLength",
                "getPersonCapacity", "getColor", "getVehicleClass")


def kind(vehicle_id: str) -> str:
    if vehicle_id.startswith("emergency_dispatch_"):
        return "responder"
    if vehicle_id.startswith("emergency_"):
        return "ambient"
    return "car"


# ------------------------------------------------------------------ 1. physics


def _types_xml():
    """vType elements for the responder, ambient and car types, as the demand files define them."""
    wanted = {"fire_truck": "demand/porto.dispatch.rou.xml",
              "emergency_fire_truck": "demand/porto.emergency.rou.xml",
              "normal_passenger": f"demand/seeds/porto.rou.{SEED}.xml"}
    out = []
    for type_id, path in wanted.items():
        for vtype in ET.parse(path).getroot().iter("vType"):
            if vtype.get("id") == type_id:
                out.append(ET.tostring(vtype, encoding="unicode"))
    return "".join(out)


def _route_404():
    for vehicle in ET.parse("demand/porto.dispatch.rou.xml").getroot().iter("vehicle"):
        if vehicle.get("id") == "emergency_dispatch_404":
            return vehicle.find("route").get("edges").split()
    raise SystemExit("responder 404 not found")


def physics_one(out: Path, policy: bool, vehicle_id: str, vehicle_type: str) -> dict:
    from algorithms.route_utils import vehicle_is_closure_exempt
    from emergency_manager import EmergencyManager, get_emergency_manager

    EmergencyManager.restrict_ambient_road_access = policy
    edges = _route_404()
    centre = edges[-1]
    tag = f"{'on' if policy else 'off'}_{kind(vehicle_id)}"
    route_file = out / f"mini_{tag}.rou.xml"
    route_file.write_text(
        f'<routes>{_types_xml()}<vehicle id="{vehicle_id}" type="{vehicle_type}" depart="5">'
        f'<route edges="{" ".join(edges)}"/></vehicle></routes>', encoding="utf-8")
    manager = get_emergency_manager()
    manager.set_planned_events([{
        "event_id": "mini", "start_time": 1, "end_time": 100000, "incident_type": "test",
        "center_edge": centre, "affected_edges": [centre],
        "dispatch": {"vehicle_id": "emergency_dispatch_404", "vehicle_type": "fire_truck"}}])
    manager.generate_emergencies()
    traci.start(["sumo", "-n", "network/porto.net.xml", "-r", str(route_file), "--no-step-log", "true",
                 "--time-to-teleport", "-1", "--ignore-route-errors", "true", "--end", "600",
                 "--error-log", str(out / f"mini_{tag}_errors.log")])
    before = {g: getattr(traci.vehicletype, g)("emergency_fire_truck") for g in TYPE_GETTERS}
    on_centre, last_edge, exempt, vclass = [], None, None, None
    distances = {}
    try:
        while traci.simulation.getTime() < 600:
            traci.simulationStep()
            t = traci.simulation.getTime()
            manager.update(t)
            ids = traci.vehicle.getIDList()
            manager.record_responder_arrivals(t, vehicle_ids=ids, arrived_ids=traci.simulation.getArrivedIDList())
            if t == 2:
                after = {g: getattr(traci.vehicletype, g)("emergency_fire_truck") for g in TYPE_GETTERS}
                query = manager._distance_query_type("emergency_fire_truck")
                distances = {e: manager._route_distance(e, centre, query) for e in edges[:-1:10]}
            if vehicle_id in ids:
                last_edge = traci.vehicle.getRoadID(vehicle_id)
                exempt = vehicle_is_closure_exempt(vehicle_id)
                vclass = traci.vehicle.getVehicleClass(vehicle_id)
                if last_edge == centre and centre in manager.blocked_edges:
                    on_centre.append(t)
    finally:
        traci.close()
    event = manager.events["mini"]
    return {
        "vehicle": vehicle_id, "type": vehicle_type, "rule": "on" if policy else "off",
        "class while driving": vclass, "router treats it as closure-exempt": exempt,
        "entered the closed centre edge": bool(on_centre),
        "first time on the closed centre edge": on_centre[0] if on_centre else None,
        "last edge seen": last_edge,
        "response recorded (responder only)": event.first_arrival_time,
        "type parameters changed by the switch": sorted(g for g in TYPE_GETTERS if before[g] != after[g]),
        "ambient-type incident distances from route edges": distances,
    }


def physics(out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    cases = (("emergency_dispatch_404", "fire_truck"), ("emergency_99999", "emergency_fire_truck"),
             ("normal_99999", "normal_passenger"))
    rows = [physics_one(out, policy, vid, vt) for policy in (True, False) for vid, vt in cases]
    by = {(r["rule"], kind(r["vehicle"])): r for r in rows}
    checks = {
        "rule on: responder enters its closed incident road": by["on", "responder"]["entered the closed centre edge"],
        "rule on: responder's arrival is recorded while the road is closed":
            by["on", "responder"]["response recorded (responder only)"] is not None,
        "rule on: ambient vehicle never enters the closed road": not by["on", "ambient"]["entered the closed centre edge"],
        "rule on: car never enters the closed road": not by["on", "car"]["entered the closed centre edge"],
        "rule on: ambient class is authority, responder emergency, car passenger":
            (by["on", "ambient"]["class while driving"], by["on", "responder"]["class while driving"],
             by["on", "car"]["class while driving"]) == ("authority", "emergency", "passenger"),
        "rule on: router exemption responder yes, ambient no, car no":
            (by["on", "responder"]["router treats it as closure-exempt"],
             by["on", "ambient"]["router treats it as closure-exempt"],
             by["on", "car"]["router treats it as closure-exempt"]) == (True, False, False),
        "rule off (contrast): ambient vehicle does enter the closed road":
            by["off", "ambient"]["entered the closed centre edge"],
        "rule off: car still never enters": not by["off", "car"]["entered the closed centre edge"],
        "the switch changes only the vehicle class of the ambient type":
            by["on", "ambient"]["type parameters changed by the switch"] == ["getVehicleClass"],
        "rule off: no type parameter changes": by["off", "ambient"]["type parameters changed by the switch"] == [],
        "ambient incident distances identical under both rules":
            by["on", "ambient"]["ambient-type incident distances from route edges"]
            == by["off", "ambient"]["ambient-type incident distances from route edges"]
            and all(v != float("inf") for v in by["on", "ambient"]["ambient-type incident distances from route edges"].values()),
    }
    return {"checks": checks, "runs": rows}


# ------------------------------------------------------------------ 2. real scenario


def run(arm: str, policy: bool, duration: int, out: Path) -> None:
    from algorithms.route_utils import get_route_stats
    from emergency_manager import EmergencyManager, get_emergency_manager
    from evaluation_seeds import route_choice_seed, sumo_seed_args

    EmergencyManager.restrict_ambient_road_access = policy
    out.mkdir(parents=True, exist_ok=True)
    seed_args = sumo_seed_args(SEED, Path("simulation.sumocfg"))
    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator, exceptions = None, []
    if arm != "baseline":
        from p3_runtime import P3RuntimeCoordinator
        coordinator = P3RuntimeCoordinator(emergency_layer=(arm == "p3on"), route_choice_seed=route_choice_seed(SEED))
        original = coordinator._log_exception

        def counted(context, exc):
            exceptions.append(f"{context}: {type(exc).__name__}: {exc}")
            original(context, exc)
        coordinator._log_exception = counted
    traci.start(["sumo", "-c", "simulation.sumocfg", "--no-step-log", "true", "--end", str(duration),
                 "--tripinfo-output", str(out / "tripinfo.xml"), "--tripinfo-output.write-unfinished", "true",
                 "--summary-output", str(out / "summary.xml"), "--error-log", str(out / "errors.log"),
                 *seed_args])
    departed_class, entries, present_at_close = {}, [], []
    closed_since, previous = {}, {}
    responders = {e.dispatch_vehicle_id for e in manager.events.values()}
    flag_mismatch_steps, counters_at_599 = 0, None
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            t = traci.simulation.getTime()
            manager.update(t)
            if coordinator is not None:
                coordinator.update_step(t)
            else:
                manager.record_responder_arrivals(t, vehicle_ids=traci.vehicle.getIDList(),
                                                  arrived_ids=traci.simulation.getArrivedIDList())
            for vehicle_id in traci.simulation.getDepartedIDList():
                departed_class[vehicle_id] = traci.vehicle.getVehicleClass(vehicle_id)
            ended_teleport = set(traci.simulation.getEndingTeleportIDList())
            current = {}
            for edge in sorted(manager.blocked_edges):
                closed_since.setdefault(edge, t)
                now = set(traci.edge.getLastStepVehicleIDs(edge))
                current[edge] = now
                if edge not in previous:
                    present_at_close += [{"t": t, "edge": edge, "vehicle": v, "kind": kind(v)} for v in sorted(now)]
                    continue
                for v in sorted(now - previous[edge]):
                    entries.append({"t": t, "edge": edge, "vehicle": v, "kind": kind(v),
                                    "class": departed_class.get(v), "s_after_closure": t - closed_since[edge],
                                    "by_teleport": v in ended_teleport})
            for edge in set(closed_since) - set(current):
                del closed_since[edge]
            previous = current
            # PD-1: in the network, the emergency class is held only by active responders (or a
            # responder already on its own centre edge after arriving).
            in_network = set(traci.vehicle.getIDList())
            emergency_now = {v for v in in_network if departed_class.get(v) == "emergency"}
            if not emergency_now <= responders:
                flag_mismatch_steps += 1
            if t == FIRST_CLOSURE - 1 and coordinator is not None:
                counters_at_599 = {k: getattr(coordinator.metrics, k) for k in (
                    "reroute_decisions", "reroute_executed", "candidate_edges_scored",
                    "yield_actions_performed", "emergency_reroutes")}
    finally:
        traci.close()
    record = {
        "arm": arm, "rule": "on" if policy else "off", "seed": SEED, "duration": duration,
        "ambient_road_classes": manager.ambient_road_classes,
        "departure classes by kind": {k: sorted({c for v, c in departed_class.items() if kind(v) == k})
                                      for k in ("responder", "ambient", "car")},
        "departed by kind": {k: sum(1 for v in departed_class if kind(v) == k) for k in ("responder", "ambient", "car")},
        "steps where a non-responder held class emergency": flag_mismatch_steps,
        "entries onto a closed edge": entries,
        "on the edge when it closed": present_at_close,
        "responses": manager.get_response_summary(),
        "exceptions_logged": exceptions,
        "counters_at_599": counters_at_599,
        "counters_at_end": None if coordinator is None else {k: getattr(coordinator.metrics, k) for k in (
            "reroute_decisions", "reroute_executed", "candidate_edges_scored",
            "yield_actions_performed", "emergency_reroutes")},
        "route_stats": {k: v for k, v in get_route_stats().items() if isinstance(v, (int, float))},
        "incident_errors": manager.incident_errors,
    }
    (out / "run.json").write_text(json.dumps(record, indent=1, default=str), encoding="utf-8")


def _summary_steps(path: Path, before: float):
    """Every summary step before `before`, without `duration`: that attribute is SUMO's wall-clock
    computing time for the step in milliseconds, so it differs between any two runs. (The first
    verdict of 2026-09-13 compared it and failed on it alone: 515-540 of 600 steps differed in
    `duration` and in nothing else.)"""
    return [{k: v for k, v in s.attrib.items() if k != "duration"}
            for s in ET.parse(path).getroot().iter("step") if float(s.get("time")) < before]


def _trips(path: Path):
    return {t.get("id"): dict(t.attrib) for t in ET.parse(path).getroot().iter("tripinfo")}


def compare(out: Path) -> dict:
    runs = {(a, r): json.loads((out / f"{a}_{r}" / "run.json").read_text(encoding="utf-8"))
            for a in ARMS for r in ("on", "off")}
    checks, detail = {}, {}
    for arm in ARMS:
        on, off = runs[arm, "on"], runs[arm, "off"]
        d_on, d_off = out / f"{arm}_on", out / f"{arm}_off"
        same_summary = _summary_steps(d_on / "summary.xml", FIRST_CLOSURE) == _summary_steps(d_off / "summary.xml", FIRST_CLOSURE)
        t_on, t_off = _trips(d_on / "tripinfo.xml"), _trips(d_off / "tripinfo.xml")
        early = lambda trips: {k: v for k, v in trips.items() if 0 <= float(v["arrival"]) < FIRST_CLOSURE}
        same_early = early(t_on) == early(t_off)
        checks[f"{arm}: identical to the rule-off run before the first closure (every summary step, every trip finished by t=600)"] = same_summary and same_early
        if arm != "baseline":
            checks[f"{arm}: coordinator counters at t=599 identical"] = on["counters_at_599"] == off["counters_at_599"]
        checks[f"{arm}: all four ambient types switched to authority"] = sorted(on["ambient_road_classes"].values()) == ["authority"] * 4
        checks[f"{arm}: departure classes ambient=authority, responder=emergency, car=passenger"] = (
            on["departure classes by kind"] == {"responder": ["emergency"], "ambient": ["authority"], "car": ["passenger"]})
        checks[f"{arm}: rule off keeps ambient emergency (contrast run is the old behaviour)"] = (
            off["departure classes by kind"]["ambient"] == ["emergency"])
        checks[f"{arm}: no non-responder held class emergency in any step"] = on["steps where a non-responder held class emergency"] == 0
        driven = lambda r, k: [e for e in r["entries onto a closed edge"]
                               if e["kind"] == k and not e["by_teleport"] and e["s_after_closure"] > COMMITTED_S]
        checks[f"{arm}: no ambient vehicle drove onto a closed road"] = driven(on, "ambient") == []
        checks[f"{arm}: no car drove onto a closed road (rule on and off)"] = driven(on, "car") == [] and driven(off, "car") == []
        checks[f"{arm}: no exception logged"] = not on["exceptions_logged"] and not off["exceptions_logged"]
        detail[arm] = {
            "ambient entries onto a closed road, rule off": driven(off, "ambient"),
            "ambient entries onto a closed road, rule on": driven(on, "ambient"),
            "responder entries onto a closed road, rule on": driven(on, "responder"),
            "entries within 3 s of a closure starting or by teleport, rule on": [
                e for e in on["entries onto a closed edge"] if e["by_teleport"] or e["s_after_closure"] <= COMMITTED_S],
            "responses rule on": {e: (s["response_time_s"], s["response_valid"]) for e, s in on["responses"].items()},
            "responses rule off": {e: (s["response_time_s"], s["response_valid"]) for e, s in off["responses"].items()},
            "trip records differing after t=600 (on vs off)": sum(1 for k in set(t_on) | set(t_off) if t_on.get(k) != t_off.get(k)),
            "car records differing (on vs off)": sum(1 for k in set(t_on) | set(t_off) if kind(k) == "car" and t_on.get(k) != t_off.get(k)),
            "counters at end rule on / off": (on["counters_at_end"], off["counters_at_end"]),
            "route stats rule on (selected)": {k: on["route_stats"].get(k) for k in (
                "reroutes_applied", "routes_crossing_closed_edge", "setroute_silent_rejections", "setroute_errors")},
            "route stats rule off (selected)": {k: off["route_stats"].get(k) for k in (
                "reroutes_applied", "routes_crossing_closed_edge", "setroute_silent_rejections", "setroute_errors")},
        }
    checks["p3off: the control arm still performs no yielding (rule on and off)"] = (
        runs["p3off", "on"]["counters_at_end"]["yield_actions_performed"] == 0
        and runs["p3off", "off"]["counters_at_end"]["yield_actions_performed"] == 0)
    checks["p3on: the treatment arm still yields (rule on)"] = runs["p3on", "on"]["counters_at_end"]["yield_actions_performed"] > 0
    return {"checks": checks, "detail": detail}


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("--arm", choices=ARMS, required=True)
    p.add_argument("--rule", choices=("on", "off"), required=True)
    p.add_argument("--duration", type=int, default=DURATION)
    p.add_argument("--out", required=True)
    p = sub.add_parser("all")
    p.add_argument("--out", required=True)
    p = sub.add_parser("verdict", help="re-judge saved scenario runs; re-runs only the physics part (seconds)")
    p.add_argument("--out", required=True)
    args = parser.parse_args()
    if args.cmd == "run":
        run(args.arm, args.rule == "on", args.duration, Path(args.out))
        return 0

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    procs = []
    if args.cmd == "all":
        for arm in ARMS:
            for rule in ("on", "off"):
                log = open(out / f"{arm}_{rule}.log", "w", encoding="utf-8")
                procs.append((subprocess.Popen([sys.executable, __file__, "run", "--arm", arm, "--rule", rule,
                                                "--out", str(out / f"{arm}_{rule}")], stdout=log, stderr=subprocess.STDOUT), log))
    phys = physics(out / "physics")
    for proc, log in procs:
        proc.wait()
        log.close()
    tracebacks = [p.name for p in out.glob("*.log") if "Traceback" in p.read_text(encoding="utf-8", errors="replace")]
    scenario = compare(out)
    checks = {**{f"physics: {k}": v for k, v in phys["checks"].items()}, **scenario["checks"],
              "no traceback in any run log": not tracebacks}
    ok = all(checks.values())
    verdict = {"pass": ok, "checks": checks, "tracebacks": tracebacks,
               "physics": phys["runs"], "scenario": scenario["detail"]}
    (out / "verdict.json").write_text(json.dumps(verdict, indent=1, default=str), encoding="utf-8")
    for key, value in checks.items():
        print(f"{'PASS' if value else 'FAIL'}  {key}")
    print("pd1 q2:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
