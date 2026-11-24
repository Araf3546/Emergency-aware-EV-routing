#!/usr/bin/env python3
"""The unified yield rule: "make room if you can, never brake in the responder's lane" (2026-09-15).

Diagnostic only. It changes no production code and writes only into the folder given by --out. Follows
`single_lane_yield_check.py` (results/single_lane_yield_2026-09-15/), whose findings motivated the rule:
- braking on a single-lane road holds up a responder that cannot pass;
- multi-lane yields made room within 3 s in only 18% of the cases with the responder directly behind,
  and otherwise braked in the responder's own lane.

**The rule (the user's decision to test).** Who yields, when and to whom is unchanged
(`_apply_yield_behaviour`), and so is the bookkeeping (cooldown, yield flag, counters). What a
yielding car does:
- it never brakes;
- on a road with two or more lanes its class may use, if it is in the responder's lane path ahead of
  the responder, and a usable lane lies to its right, it is asked to move there for the same 3 s. The
  target is the rightmost usable lane, which is lane 0, the lane production asks for, whenever the car
  may use lane 0;
- otherwise it keeps driving. That covers a single-lane road, a car not in the responder's lane, and a
  car already in the rightmost lane it may use.
Production, for comparison: every yielding car brakes to max(2 m/s, 40%) over 3 s. On a road with more
than one lane, a car not in lane 0 is asked to move to lane 0, whether or not it is in the responder's
lane.

**"In the responder's lane path"** means the car is one of these:
- the responder's immediate leader;
- ahead of the responder in its lane;
- in a lane the responder continues into (SUMO's best-lane continuation for the responder's current
  lane).

**Variants.**
  production      the code as it is (observation only)
  make_room       the rule, for every responder's yields
  make_room --only-responder 401|405|404
                  the rule for that responder's yields only. Such a run is the production run until that
                  responder's first yield: a same-state comparison for that incident.
A shorter run is the production run up to its end (checked: 1,250 s against the first 1,250 s of 3,600 s,
identical).

**Measured**, as in `single_lane_yield_check.py`, plus:
- whether the car was in the responder's lane path at the yield and in the next 5 s;
- whether it was braked;
- both lane ids;
- the responder's lane each step, and whether the car ahead of it is braking for a yield.

    python scripts/diagnostics/make_room_yield_check.py run --variant production --default-weights --out DIR/def_production
    python scripts/diagnostics/make_room_yield_check.py run --variant make_room --only-responder 401 --sim-time 1800 --unscored --default-weights --out DIR/def_make_room_401
    python scripts/diagnostics/make_room_yield_check.py analyse --out DIR/analysis DIR/def_production DIR/def_make_room ...
    python scripts/diagnostics/make_room_yield_check.py table DIR/analysis/analysis.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from single_lane_yield_check import (  # noqa: E402  (imports yield_signal_diagnosis, which chdirs to REPO_ROOT)
    FOLLOW_M, REFERENCE, SEED, TRACE_S, at, episodes, following, mean, median, per_responder)
from yield_signal_diagnosis import records  # noqa: E402

PATH_TRACE_S = 5        # seconds after a yield in which the car's place in the responder's lane path is re-checked


def live_run(args) -> int:
    import traci
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    import tune_e3_weights as tuner
    import p3_runtime
    import p3_hybrid_router
    from algorithms import route_utils
    import inspect
    if "speed" not in inspect.signature(p3_runtime.P3RuntimeCoordinator._yield_to_emergency).parameters:
        raise SystemExit("`run` emulates the unified rule on top of the yield action as it was before the unified rule (2026-09-15) "
                         "(brake to 40%, then lane 0). Production now runs the unified rule itself, so it would not "
                         "mean what its evidence says; `unified_yield_reproduce.py` observes it. `analyse` and "
                         "`table` still read existing runs.")

    out = Path(args.out)
    trial_dir = out / "trial"
    if trial_dir.exists() and any(trial_dir.iterdir()):
        raise SystemExit(f"{trial_dir} already holds files; evidence is never overwritten")
    out.mkdir(parents=True, exist_ok=True)
    variant = args.variant
    only = f"emergency_dispatch_{args.only_responder}" if args.only_responder else None
    C = p3_runtime.P3RuntimeCoordinator
    duration, radius = p3_runtime.YIELD_DURATION_S, p3_runtime.YIELD_RADIUS_M
    holder = {"coordinator": None, "errors": [], "open": [], "lanes": {}, "paths": {}, "path_t": None,
              "braked_until": {}}
    events, steps = [], []
    counts = {"yields": 0, "single_lane": 0, "multi_lane": 0, "rule_applied": 0, "ambiguous_responder": 0,
              "no_responder": 0, "actions": {}}

    # -------- helpers: reads only
    def lanes_for(edge_id, vclass):
        """Indices of the lanes of `edge_id` that `vclass` may use, and the lane count. A closed edge is not cached."""
        closed = edge_id in route_utils.blocked_edges()
        key = (edge_id, vclass)
        if not closed and key in holder["lanes"]:
            return holder["lanes"][key]
        total = traci.edge.getLaneNumber(edge_id)
        usable = []
        for i in range(total):
            allowed = traci.lane.getAllowed(f"{edge_id}_{i}")
            disallowed = traci.lane.getDisallowed(f"{edge_id}_{i}")
            if (not allowed or vclass in allowed) and vclass not in disallowed:
                usable.append(i)
        if not closed:
            holder["lanes"][key] = (usable, total)
        return usable, total

    def leader(v, dist):
        try:
            lead = traci.vehicle.getLeader(v, dist)
        except traci.TraCIException:
            return None, None
        if not lead or not lead[0]:
            return None, None
        return lead[0], float(lead[1])

    def lane_path(r, t):
        """The responder's lane now, then the lanes it continues into. Cached for the current step."""
        if holder["path_t"] != t:
            holder["paths"], holder["path_t"] = {}, t
        if r not in holder["paths"]:
            lane = traci.vehicle.getLaneID(r)
            path = [lane]
            try:
                for entry in traci.vehicle.getBestLanes(r):
                    if entry[0] == lane:
                        path += list(entry[5])
                        break
            except traci.TraCIException:
                pass
            holder["paths"][r] = path
        return holder["paths"][r]

    def in_path(car, r, t):
        """How `car` is in the responder's lane path ahead of it, or None."""
        lid, _ = leader(r, radius)
        if lid == car:
            return "leader"
        car_lane, resp_lane = traci.vehicle.getLaneID(car), traci.vehicle.getLaneID(r)
        if car_lane == resp_lane:
            return "same_lane" if traci.vehicle.getLanePosition(car) > traci.vehicle.getLanePosition(r) else None
        if car_lane in lane_path(r, t)[1:]:
            return "continuation"
        return None

    def responder_for(self, edge_id, car, sim_time, present):
        """The responder whose broadcast made `car` yield, by `_apply_yield_behaviour`'s own test (its road or its
        announced next road is `edge_id`). Read from the message store without pruning it."""
        found = []
        for vid, entry in self.protocol_state.neighbors.items():
            m = entry.message
            if vid == car or vid not in present or not m.is_emergency() \
                    or m.is_stale(sim_time, self.protocol_state.freshness_window):
                continue
            if edge_id in (traci.vehicle.getRoadID(vid), m.next_edge):
                a, b = traci.vehicle.getPosition(car), traci.vehicle.getPosition(vid)
                found.append((math.hypot(a[0] - b[0], a[1] - b[1]), vid))
        found.sort()
        return (found[0][1] if found else None), len(found)

    def bookkeeping(self, vehicle_id, sim_time):
        """Exactly what production's `_yield_to_emergency` records after acting."""
        self.last_yield[vehicle_id] = sim_time
        self.yield_actions += 1
        try:
            self.metrics.record_yield_action(vehicle_id)
        except Exception:
            pass

    real_init = C.__init__

    def init(self, *a, **k):
        real_init(self, *a, **k)
        holder["coordinator"] = self

    C.__init__ = init

    # -------- the yield response (the only behaviour a variant changes)
    real_yield = C._yield_to_emergency

    def yield_action(self, vehicle_id, edge_id, speed, sim_time):
        ev = {"t": sim_time, "car": vehicle_id, "edge": edge_id, "car_speed": speed}
        try:
            present = set(traci.vehicle.getIDList())
            ev["class"] = traci.vehicle.getVehicleClass(vehicle_id)
            usable, total = lanes_for(edge_id, ev["class"])
            ev.update(usable_lanes=len(usable), total_lanes=total, usable_indices=usable,
                      lane=traci.vehicle.getLaneID(vehicle_id), lane_index=traci.vehicle.getLaneIndex(vehicle_id),
                      car_allowed=traci.vehicle.getAllowedSpeed(vehicle_id),
                      car_pos=traci.vehicle.getLanePosition(vehicle_id))
            r, matches = responder_for(self, edge_id, vehicle_id, sim_time, present)
            ev["responder"], ev["responder_matches"] = r, matches
            if r is not None:
                lid, gap = leader(r, 2 * FOLLOW_M)
                ev.update(resp_speed=traci.vehicle.getSpeed(r), resp_allowed=traci.vehicle.getAllowedSpeed(r),
                          resp_edge=traci.vehicle.getRoadID(r), resp_lane=traci.vehicle.getLaneID(r),
                          resp_lane_index=traci.vehicle.getLaneIndex(r), resp_pos=traci.vehicle.getLanePosition(r),
                          resp_leader=lid, resp_gap=gap, blocking=in_path(vehicle_id, r, sim_time))
        except Exception as exc:
            holder["errors"].append(f"yield record {vehicle_id}: {exc!r}")
            ev.setdefault("usable_lanes", 2)
        single = ev["usable_lanes"] < 2
        counts["yields"] += 1
        counts["single_lane" if single else "multi_lane"] += 1
        counts["ambiguous_responder"] += ev.get("responder_matches", 0) > 1
        counts["no_responder"] += ev.get("responder") is None

        rule = variant == "make_room" and (only is None or ev.get("responder") == only)
        if not rule:
            before = self.last_yield.get(vehicle_id)
            real_yield(self, vehicle_id, edge_id, speed, sim_time)
            ev["applied"] = self.last_yield.get(vehicle_id) != before
            ev["action"] = "brake" + ("+move_right" if ev.get("total_lanes", 1) > 1 and ev.get("lane_index", 0) > 0 else "")
            ev["braked"] = ev["applied"]
            if ev["applied"]:
                holder["braked_until"][vehicle_id] = sim_time + duration
        else:
            counts["rule_applied"] += 1
            if single:
                action = "keep_driving_single_lane"
            elif not ev.get("blocking"):
                action = "keep_driving_not_in_lane"
            else:
                below = [i for i in ev.get("usable_indices", []) if i < ev.get("lane_index", 0)]
                if not below:
                    action = "keep_driving_cannot_move"
                else:
                    try:
                        traci.vehicle.changeLane(vehicle_id, min(below), duration)
                    except traci.TraCIException as exc:   # production returns without bookkeeping when its action fails
                        holder["errors"].append(f"changeLane {vehicle_id}: {exc!r}")
                        ev.update(action="move_failed", applied=False, braked=False)
                        events.append(ev)
                        return
                    action = "move_right"
                    ev["target_lane"] = min(below)
            ev.update(action=action, applied=True, braked=False)
            bookkeeping(self, vehicle_id, sim_time)
        counts["actions"][ev["action"]] = counts["actions"].get(ev["action"], 0) + 1
        ev["trace"] = []
        events.append(ev)
        holder["open"].append(ev)

    C._yield_to_emergency = yield_action

    # -------- every step, after the coordinator: observe
    real_update = C.update_step

    def update_step(self, sim_time):
        real_update(self, sim_time)
        try:
            present = set(traci.vehicle.getIDList())
            for r in sorted(self._active_responders):
                if r not in present:
                    continue
                edge = traci.vehicle.getRoadID(r)
                row = {"t": sim_time, "r": r, "speed": traci.vehicle.getSpeed(r),
                       "allowed": traci.vehicle.getAllowedSpeed(r), "edge": edge,
                       "lane_index": traci.vehicle.getLaneIndex(r), "odo": traci.vehicle.getDistance(r)}
                if not edge.startswith(":"):
                    row["usable_lanes"] = len(lanes_for(edge, "passenger")[0])
                lid, gap = leader(r, FOLLOW_M)
                if lid is not None:
                    row.update(leader=lid, gap=gap, leader_speed=traci.vehicle.getSpeed(lid),
                               leader_is_responder=lid in self._active_responders,
                               leader_yielding=0.0 < sim_time - self.last_yield.get(lid, -1e9) <= duration,
                               leader_braked=holder["braked_until"].get(lid, -1e9) >= sim_time)
                steps.append(row)
            still = []
            for ev in holder["open"]:
                k = sim_time - ev["t"]
                if k > TRACE_S:
                    continue
                c, r = ev["car"], ev.get("responder")
                row = {"k": k}
                if c in present:
                    row.update(cs=traci.vehicle.getSpeed(c), ce=traci.vehicle.getRoadID(c),
                               cl=traci.vehicle.getLaneIndex(c), cln=traci.vehicle.getLaneID(c),
                               cp=traci.vehicle.getLanePosition(c))
                if r is not None and r in present:
                    row.update(rs=traci.vehicle.getSpeed(r), ra=traci.vehicle.getAllowedSpeed(r),
                               re=traci.vehicle.getRoadID(r), rl=traci.vehicle.getLaneIndex(r),
                               rp=traci.vehicle.getLanePosition(r))
                    row["r_leader"] = leader(r, 2 * FOLLOW_M)[0]
                    if c in present and k <= PATH_TRACE_S:
                        row["in_path"] = in_path(c, r, sim_time)
                ev["trace"].append(row)
                still.append(ev)
            holder["open"] = still
        except Exception as exc:
            holder["errors"].append(f"observe t={sim_time}: {exc!r}")

    C.update_step = update_step

    # -------- the real trial
    if args.default_weights:
        weights = {name: float(p3_hybrid_router.DEFAULT_WEIGHTS[name]) for name in tuner.WEIGHT_NAMES}
    else:
        weights = json.loads(Path(args.weights_from).read_text(encoding="utf-8"))["weights_requested"]
    reference = None if args.unscored else tuner.load_reference(Path(args.reference), SEED, args.sim_time)
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), args.sim_time, SEED,
                               trial_dir=trial_dir, reference=reference)
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    summary = {"variant": variant, "only_responder": only, "sim_time": args.sim_time, "weights": weights,
               "rejected": rejected, "instrumentation_errors": holder["errors"], "counts": counts,
               "yield_actions_coordinator": holder["coordinator"].yield_actions if holder["coordinator"] else None}
    (out / "events.json").write_text(json.dumps(events), encoding="utf-8")
    (out / "responder_steps.json").write_text(json.dumps(steps), encoding="utf-8")
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(json.dumps(summary))
    return 0 if rejected is None and not holder["errors"] else 1


# ================================================================== analysis

def left_path(e, by=3):
    """The car was in the responder's lane path at the yield and was out of it at some second 1..`by`."""
    return bool(e.get("blocking")) and any(r.get("k", 0) >= 1 and r["k"] <= by and "in_path" in r and not r["in_path"]
                                           for r in e.get("trace", []))


def entered_path(e, by=3):
    """The car was not in the responder's lane path at the yield and was in it at some second 1..`by`."""
    return e.get("responder") is not None and not e.get("blocking") and any(
        r.get("k", 0) >= 1 and r["k"] <= by and r.get("in_path") for r in e.get("trace", []))


def lane_type_metrics(evs) -> dict:
    blocking = [e for e in evs if e.get("blocking")]
    not_blocking = [e for e in evs if e.get("responder") is not None and not e.get("blocking")]
    made_room = [e for e in blocking if left_path(e)]
    by_lane_change = [e for e in made_room if any(r.get("k", 0) >= 1 and r["k"] <= 3 and r.get("ce") == e["edge"]
                                                  and r.get("cl") is not None and r["cl"] != e.get("lane_index")
                                                  for r in e.get("trace", []))]
    braked = [e for e in evs if e.get("braked")]

    def resp(group, key="rs"):
        return [mean([at(e, k, key) for e in group]) for k in (0, 1, 2, 3)]

    return {
        "yields": len(evs), "in_responder_lane_path": len(blocking), "not_in_path": len(not_blocking),
        "in_path_left_within_1_2_3_s": [sum(left_path(e, b) for e in blocking) for b in (1, 2, 3)],
        "left_by_changing_lane_on_the_same_road": len(by_lane_change),
        "not_in_path_entered_it_within_3_s": sum(entered_path(e) for e in not_blocking),
        "braked": len(braked),
        "braked_while_in_path_and_not_leaving_it_within_3_s": sum(1 for e in blocking if e.get("braked") and not left_path(e)),
        "braked_while_not_in_path": sum(1 for e in not_blocking if e.get("braked")),
        "passenger_speed_0_1_2_3_s": [mean([at(e, k, "cs") for e in evs]) for k in (0, 1, 2, 3)],
        "responder_speed_0_1_2_3_s_car_in_path": resp(blocking),
        "responder_over_allowed_3_s_car_in_path": mean([at(e, 3, "rs") / at(e, 3, "ra") for e in blocking
                                                        if at(e, 3, "rs") is not None and at(e, 3, "ra")]),
        "responder_over_allowed_3_s_car_in_path_and_stayed": mean(
            [at(e, 3, "rs") / at(e, 3, "ra") for e in blocking if not left_path(e)
             and at(e, 3, "rs") is not None and at(e, 3, "ra")]),
        "responder_over_allowed_3_s_car_left_path": mean(
            [at(e, 3, "rs") / at(e, 3, "ra") for e in made_room if at(e, 3, "rs") is not None and at(e, 3, "ra")]),
    }


def move_right_outcome(evs) -> dict:
    """Move-right requests (the rule's, or production's brake-and-move), and how many were carried out within 3 s:
    a lower lane index on the same road. Split by whether the car was in the responder's lane path."""
    asked = [e for e in evs if e.get("action") in ("move_right", "brake+move_right")]

    def moved(e):
        return any(1 <= r["k"] <= 3 and r.get("ce") == e["edge"] and r.get("cl") is not None
                   and r["cl"] < e["lane_index"] for r in e.get("trace", []))

    in_path = [e for e in asked if e.get("blocking")]
    return {"asked": len(asked), "moved_within_3s": sum(moved(e) for e in asked),
            "asked_in_path": len(in_path), "moved_within_3s_in_path": sum(moved(e) for e in in_path)}


def behind_braked(steps) -> dict:
    """Responder seconds behind a car braking for a yield (within 50 m), by lane type, and its speed then."""
    out = {}
    for lanes in ("single", "multi"):
        rows = [s for s in steps if s.get("usable_lanes") is not None and (s["usable_lanes"] < 2) == (lanes == "single")
                and s.get("leader") and not s["leader_is_responder"] and s["gap"] <= FOLLOW_M and s.get("leader_braked")]
        out[lanes] = {"seconds": len(rows), "speed_over_allowed": mean([s["speed"] / s["allowed"] for s in rows if s["allowed"]])}
    return out


def per_incident_braked(steps) -> dict:
    out = {}
    for r in sorted({s["r"] for s in steps}):
        rows = [s for s in steps if s["r"] == r and s.get("leader") and not s["leader_is_responder"]
                and s["gap"] <= FOLLOW_M and s.get("leader_braked")]
        out[r] = {"seconds_behind_a_braking_car": len(rows),
                  "speed_over_allowed_there": mean([s["speed"] / s["allowed"] for s in rows if s["allowed"]])}
    return out


def analyse(args) -> int:
    report, runs = {}, {}
    for run in (Path(p) for p in args.runs):
        trial = json.loads((run / "trial" / "trial.json").read_text(encoding="utf-8"))
        summ = json.loads((run / "summary.json").read_text(encoding="utf-8"))
        evs = json.loads((run / "events.json").read_text(encoding="utf-8"))
        steps = json.loads((run / "responder_steps.json").read_text(encoding="utf-8"))
        runs[run.name] = (run, trial, summ, evs, steps)
        m = trial["metrics"]
        single = [e for e in evs if e.get("usable_lanes", 2) < 2]
        multi = [e for e in evs if e.get("usable_lanes", 2) >= 2]
        tracebacks = sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback")
                         for p in (run / "run.err", run / "trial" / "errors.log") if p.is_file())
        report[run.name] = {
            "variant": summ["variant"], "only_responder": summ["only_responder"], "sim_time": summ["sim_time"],
            "responses_s": {e: r["value_s"] for e, r in m["responses"].items()},
            "response_status": {e: r["status"] for e, r in m["responses"].items()},
            "J": (trial["objective"] or {}).get("J"), "finished": m["finished"], "scheduled": m["vehicles_scheduled"],
            "completion_pct": round(100 * m["finished"] / m["vehicles_scheduled"], 2), "teleports": m["teleports"],
            "travel_s_finished_trips": round(m["travel_s"], 1), "time_loss_s": round(m["time_loss_s"], 1),
            "counts": summ["counts"], "single_lane": lane_type_metrics(single), "multi_lane": lane_type_metrics(multi),
            "episodes_all": episodes(evs), "move_right": move_right_outcome(evs),
            "responder_following": following(steps),
            "responder_behind_a_braking_car": behind_braked(steps),
            "per_responder": per_responder(evs, steps), "per_responder_braked": per_incident_braked(steps),
            "checks": {"exceptions": len(trial["exceptions_logged"]), "tracebacks": tracebacks,
                       "instrumentation_errors": len(summ["instrumentation_errors"]), "rejected": summ["rejected"],
                       "finished_equals_summary": trial["finished_equals_summary_arrived"],
                       "speed_factor_differences": trial["pairing"]["speed_factor_differences"],
                       "yield_counts_agree": summ["counts"]["yields"] == len(evs)
                       and summ["yield_actions_coordinator"] == trial["coordinator"]["yield_actions"]},
        }

    # Same state. A make_room run acts differently from production only at a yield the rule covers, so up to its
    # first such yield it is the production run: the same traffic, car and responder. From there the responder's
    # progress, and its incident's response, are compared.
    def summary_steps(run):
        return records(run / "trial" / "summary.xml", "step", ignore=("duration",))

    same_state = {}
    for name, (run, trial, summ, evs, steps) in runs.items():
        if summ["variant"] != "make_room":
            continue
        w = name.split("_make_room")[0]
        base_name = f"{w}_production"
        if base_name not in runs:
            continue
        brun, btrial, bsumm, bevs, bsteps = runs[base_name]
        first = next((e for e in evs if e.get("action", "").startswith(("keep_driving", "move_right", "move_failed"))), None)
        if first is None:
            continue
        r, t0 = first["responder"], first["t"]
        twin = next((e for e in bevs if e["t"] == t0 and e["car"] == first["car"]), None)
        inc = r.rsplit("_", 1)[-1] if r else None
        odo_v = {s["t"]: s["odo"] for s in steps if s["r"] == r}
        odo_p = {s["t"]: s["odo"] for s in bsteps if s["r"] == r}
        a, b = summary_steps(brun), summary_steps(run)
        same_state[name] = {
            "weights": w, "incident": inc, "t": t0, "car": first["car"], "action": first["action"],
            "production_action": twin["action"] if twin else None, "same_yield_in_production": twin is not None,
            "odometer_equal_at_t": odo_v.get(t0) == odo_p.get(t0) and t0 in odo_v,
            "first_differing_step": next((x[0].get("time") for x, y in zip(a, b) if x != y), None),
            "metres_after_10_20_30_60_s": {
                "production": [round(odo_p[t0 + d] - odo_p[t0], 1) if t0 in odo_p and t0 + d in odo_p else None
                               for d in (10, 20, 30, 60)],
                "make_room": [round(odo_v[t0 + d] - odo_v[t0], 1) if t0 in odo_v and t0 + d in odo_v else None
                              for d in (10, 20, 30, 60)]},
            "response_s": {"production": btrial["metrics"]["responses"].get(inc, {}).get("value_s"),
                           "make_room": trial["metrics"]["responses"].get(inc, {}).get("value_s")},
        }

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "analysis.json").write_text(json.dumps({"runs": report, "same_state": same_state}, indent=1), encoding="utf-8")
    print(f"written: {out / 'analysis.json'}")
    for name, r in report.items():
        print(name, r["checks"], r["counts"]["actions"])
    return 0


def table(args) -> int:
    data = json.loads(Path(args.analysis).read_text(encoding="utf-8"))
    runs = data["runs"]

    def f(x, d=1):
        return "-" if x is None else f"{x:.{d}f}"

    full = [n for w in ("def", "t0", "t1") for n in (f"{w}_production", f"{w}_make_room") if n in runs]

    print("\n**Outcomes, whole runs** (single seed; noise-sensitive)\n")
    print("| run | response 404 / 401 / 405 | J | finished of 4,003 | teleports | travel, finished trips | time loss |")
    print("|---|---|---|---|---|---|---|")
    for n in full:
        r = runs[n]
        resp = " / ".join(f(r["responses_s"].get(e), 0) for e in ("404", "401", "405"))
        print(f"| {n} | {resp} s | {f(r['J'], 4)} | {r['finished']:,} ({r['completion_pct']}%) | {r['teleports']} | "
              f"{r['travel_s_finished_trips']} s | {r['time_loss_s']} s |")

    print("\n**What the yielding car did** (actions)\n")
    print("| run | yields (single / multi) | actions |")
    print("|---|---|---|")
    for n in full:
        c = runs[n]["counts"]
        print(f"| {n} | {c['yields']} ({c['single_lane']} / {c['multi_lane']}) | "
              + ", ".join(f"{k} {v}" for k, v in sorted(c["actions"].items())) + " |")

    for lanes in ("multi_lane", "single_lane"):
        print(f"\n**{lanes.replace('_', '-')} yields: making room, braking, speeds** (m/s; the responder speeds are for "
              f"yields with the car in its lane path)\n")
        print("| run | yields; car in the responder's lane path | of those, left it within 1 / 2 / 3 s (by a lane change) "
              "| not in the path but entered it within 3 s | braked: total; in path and not leaving; not in path | "
              "passenger 0 / 1 / 2 / 3 s | responder 0 / 1 / 2 / 3 s | responder / allowed at 3 s: all; car stayed; car left |")
        print("|---|---|---|---|---|---|---|---|")
        for n in full:
            s = runs[n][lanes]
            lw = " / ".join(str(x) for x in s["in_path_left_within_1_2_3_s"])
            p = " / ".join(f(x) for x in s["passenger_speed_0_1_2_3_s"])
            q = " / ".join(f(x) for x in s["responder_speed_0_1_2_3_s_car_in_path"])
            print(f"| {n} | {s['yields']}; {s['in_responder_lane_path']} | {lw} ({s['left_by_changing_lane_on_the_same_road']}) | "
                  f"{s['not_in_path_entered_it_within_3_s']} of {s['not_in_path']} | {s['braked']}; "
                  f"{s['braked_while_in_path_and_not_leaving_it_within_3_s']}; {s['braked_while_not_in_path']} | {p} | {q} | "
                  f"{f(s['responder_over_allowed_3_s_car_in_path'], 2)}; "
                  f"{f(s['responder_over_allowed_3_s_car_in_path_and_stayed'], 2)}; "
                  f"{f(s['responder_over_allowed_3_s_car_left_path'], 2)} |")

    print("\n**Move-right requests carried out within 3 s** (a lower lane on the same road)\n")
    print("| run | requests | carried out | requests for a car in the responder's path | carried out |")
    print("|---|---|---|---|---|")
    for n in full:
        mv = runs[n]["move_right"]
        print(f"| {n} | {mv['asked']} | {mv['moved_within_3s']} | {mv['asked_in_path']} | {mv['moved_within_3s_in_path']} |")

    print("\n**The responder, every step: speed over its allowed speed by what is ahead within 50 m** (seconds)\n")
    print("| run | single: behind a car asked to yield | single: behind a braking yielder | single: free | "
          "multi: behind a car asked to yield | multi: behind a braking yielder | multi: free |")
    print("|---|---|---|---|---|---|---|")
    for n in full:
        fo, bb = runs[n]["responder_following"], runs[n]["responder_behind_a_braking_car"]
        cells = []
        for l in ("single", "multi"):
            cells += [f"{f(fo[l]['behind_a_yielding_car']['speed_over_allowed'], 2)} ({fo[l]['behind_a_yielding_car']['steps']})",
                      f"{f(bb[l]['speed_over_allowed'], 2)} ({bb[l]['seconds']})",
                      f"{f(fo[l]['free']['speed_over_allowed'], 2)} ({fo[l]['free']['steps']})"]
        print(f"| {n} | " + " | ".join(cells) + " |")

    print("\n**Per incident: the responder's seconds behind a car asked to yield** (single-lane / multi-lane, at speed "
          "over allowed), **behind a braking yielder, and active seconds**\n")
    print("| run | 404 | 401 | 405 |")
    print("|---|---|---|---|")
    for n in full:
        pr, pb = runs[n]["per_responder"], runs[n]["per_responder_braked"]
        cells = []
        for inc in ("404", "401", "405"):
            x, y = pr.get(f"emergency_dispatch_{inc}"), pb.get(f"emergency_dispatch_{inc}")
            cells.append("-" if x is None else
                         f"{x['seconds_behind_a_yielding_car_single']} s at {f(x['speed_over_allowed_there_single'], 2)} / "
                         f"{x['seconds_behind_a_yielding_car_multi']} s at {f(x['speed_over_allowed_there_multi'], 2)}; "
                         f"braking {y['seconds_behind_a_braking_car']} s; active {x['active_seconds']} s")
        print(f"| {n} | " + " | ".join(cells) + " |")

    print("\n**Same state: each incident's first yield under the rule** (the run is production up to it)\n")
    print("| run | incident | yield (t, car, action; production's action) | first differing step | metres after "
          "10 / 20 / 30 / 60 s: production | make_room | response: production -> make_room |")
    print("|---|---|---|---|---|---|---|")
    for n, s in data["same_state"].items():
        mp = " / ".join(f(x, 0) for x in s["metres_after_10_20_30_60_s"]["production"])
        mr = " / ".join(f(x, 0) for x in s["metres_after_10_20_30_60_s"]["make_room"])
        ok = "" if s["same_yield_in_production"] and s["odometer_equal_at_t"] else " (NOT the same state)"
        print(f"| {n} | {s['incident']} | {s['t']:.0f} s, {s['car']}, {s['action']}; {s['production_action']}{ok} | "
              f"{s['first_differing_step']} | {mp} | {mr} | {f(s['response_s']['production'], 0)} -> "
              f"{f(s['response_s']['make_room'], 0)} s |")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="one instrumented tuner trial with the given yield response")
    r.add_argument("--variant", choices=("production", "make_room"), required=True)
    r.add_argument("--only-responder", choices=("404", "401", "405"), default=None,
                   help="make_room: apply the rule to this responder's yields only")
    source = r.add_mutually_exclusive_group(required=True)
    source.add_argument("--weights-from", help="a trial.json whose weights_requested are used")
    source.add_argument("--default-weights", action="store_true")
    r.add_argument("--sim-time", type=int, default=3600)
    r.add_argument("--unscored", action="store_true", help="no reference and no J (runs shorter than 3,600 s)")
    r.add_argument("--reference", default=str(REFERENCE), help="the reference that scores J (must match --sim-time)")
    r.add_argument("--out", required=True)
    a = sub.add_parser("analyse", help="the comparison, into analysis.json")
    a.add_argument("--out", required=True)
    a.add_argument("runs", nargs="+")
    t = sub.add_parser("table", help="markdown tables from an analysis.json")
    t.add_argument("analysis")
    args = parser.parse_args()
    return {"run": live_run, "analyse": analyse, "table": table}[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
