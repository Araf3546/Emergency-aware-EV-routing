#!/usr/bin/env python3
"""Yielding on single-lane roads (2026-09-15): does it hold the responder up, and would another action help?

Diagnostic only. It changes no production code and writes only into the folder given by --out.

**The question.** At one seed the V2V layer made responses slower on the two routes with the most
single-lane road (404: 42% of its length, 401: 34%), and faster on the route with the least (405: 9%).
- A yielding car slows to max(2 m/s, 40% of its speed) over 3 s, and may be asked again every 5 s.
- On a road with another lane, it also moves right, so the responder can pass.
- On a single-lane road it cannot. This network has no opposite-lane overtaking and no sublane model,
  so the responder stays behind it.

**Variants.** Who yields, when and to whom is unchanged (`_apply_yield_behaviour`: within 250 m, ahead of
the responder on its road or on its next road, moving, not in the last 5 s). Only what the car does on a
single-lane road changes. On a road with two or more lanes it may use, every variant does exactly what
production does.
  production  the code as it is.
  keep_speed  on a single-lane road the car does nothing: it keeps driving normally.
  boost       the user's hypothesis. On a single-lane road the car does not slow. For the same 3 s its
              desired speed is raised to the responder's: its speed factor becomes the responder's, and
              is never lowered. SUMO still holds it to its own maximum speed (13.89 m/s for a passenger
              car), acceleration and safe gap, and to junction and signal rules. Its own factor is
              restored afterwards.
The yield bookkeeping (cooldown, yield flag, counters) is the same in all three, so the trigger is unchanged.
"Single-lane" means fewer than two lanes of the car's road that its own vehicle class may use, read live.

**Measured, per run** (the tuner's own trial: tuning seed, frozen demand, all three incidents):
- every yield: where, the lanes, the car and the responder it yielded to, then 30 s of both vehicles'
  speeds, roads and positions (`events.json`);
- every step of every active responder: its speed, its allowed speed, its road, its leader within 50 m
  and whether that leader is yielding, and its odometer (`responder_steps.json`);
- J and the trip metrics (`trial/trial.json`).
`analyse` turns these into the comparison tables.

    python scripts/diagnostics/single_lane_yield_check.py run --variant production --default-weights --out DIR/def_production
    python scripts/diagnostics/single_lane_yield_check.py run --variant boost --weights-from T0_TRIAL_JSON --out DIR/t0_boost
    python scripts/diagnostics/single_lane_yield_check.py analyse --out DIR DIR/def_production DIR/def_keep_speed ...
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from yield_signal_diagnosis import records  # noqa: E402  (also chdirs to REPO_ROOT)

REFERENCE = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"
SEED = 20260721
VARIANTS = ("production", "keep_speed", "boost")
TRACE_S = 30            # seconds of both vehicles followed after each yield
FOLLOW_M = 50.0         # a responder "follows" a leader closer than this
EPISODE_GAP_S = 8.0     # a car's yields to one responder within cooldown + duration form one episode


def live_run(args) -> int:
    import traci
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    import tune_e3_weights as tuner
    import p3_runtime
    import p3_hybrid_router
    from algorithms import route_utils
    import inspect
    if "speed" not in inspect.signature(p3_runtime.P3RuntimeCoordinator._yield_to_emergency).parameters:
        raise SystemExit("`run` emulates on top of the yield action as it was before the unified rule (2026-09-15): "
                         "brake to 40%, then lane 0. Production now runs the unified rule (THESIS_CODE_AUDIT_FIXES.md, "
                         "2026-09-15), so "
                         "it would not mean what its evidence says. `analyse` and `table` still read existing runs.")

    out = Path(args.out)
    trial_dir = out / "trial"
    if trial_dir.exists() and any(trial_dir.iterdir()):
        raise SystemExit(f"{trial_dir} already holds files; evidence is never overwritten")
    out.mkdir(parents=True, exist_ok=True)
    variant = args.variant
    C = p3_runtime.P3RuntimeCoordinator
    duration = p3_runtime.YIELD_DURATION_S
    holder = {"coordinator": None, "errors": [], "boosts": {}, "open": [], "lanes": {}}
    events, steps = [], []
    counts = {"yields": 0, "single_lane": 0, "multi_lane": 0, "ambiguous_responder": 0, "no_responder": 0,
              "boosts_raising_the_factor": 0, "boost_restores": 0}

    # -------- helpers: reads only
    def usable_lanes(edge_id, vclass):
        """Lanes of `edge_id` that `vclass` may use, and all lanes. Live: a closed edge is never cached."""
        closed = edge_id in route_utils.blocked_edges()
        key = (edge_id, vclass)
        if not closed and key in holder["lanes"]:
            return holder["lanes"][key]
        total = traci.edge.getLaneNumber(edge_id)
        usable = 0
        for i in range(total):
            lane = f"{edge_id}_{i}"
            allowed, disallowed = traci.lane.getAllowed(lane), traci.lane.getDisallowed(lane)
            usable += (not allowed or vclass in allowed) and vclass not in disallowed
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

    def responder_for(self, edge_id, car, sim_time, present):
        """The responder whose broadcast made `car` yield: the same test `_apply_yield_behaviour` makes (its road
        or its announced next road is `edge_id`), read from the message store without pruning it."""
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

    # -------- the coordinator the tuner builds
    real_init = C.__init__

    def init(self, *a, **k):
        real_init(self, *a, **k)
        holder["coordinator"] = self

    C.__init__ = init

    # -------- the single-lane action (the only behaviour a variant changes)
    real_yield = C._yield_to_emergency

    def yield_action(self, vehicle_id, edge_id, speed, sim_time):
        ev = {"t": sim_time, "car": vehicle_id, "edge": edge_id, "car_speed": speed}
        try:
            present = set(traci.vehicle.getIDList())
            ev["class"] = traci.vehicle.getVehicleClass(vehicle_id)
            ev["usable_lanes"], ev["total_lanes"] = usable_lanes(edge_id, ev["class"])
            ev["lane_index"] = traci.vehicle.getLaneIndex(vehicle_id)
            ev["car_allowed"] = traci.vehicle.getAllowedSpeed(vehicle_id)
            ev["car_factor"] = traci.vehicle.getSpeedFactor(vehicle_id)
            ev["car_pos"] = traci.vehicle.getLanePosition(vehicle_id)
            r, matches = responder_for(self, edge_id, vehicle_id, sim_time, present)
            ev["responder"], ev["responder_matches"] = r, matches
            if r is not None:
                a, b = traci.vehicle.getPosition(vehicle_id), traci.vehicle.getPosition(r)
                lid, gap = leader(r, 2 * FOLLOW_M)
                ev.update(resp_speed=traci.vehicle.getSpeed(r), resp_allowed=traci.vehicle.getAllowedSpeed(r),
                          resp_edge=traci.vehicle.getRoadID(r), resp_pos=traci.vehicle.getLanePosition(r),
                          resp_factor=traci.vehicle.getSpeedFactor(r), distance=math.hypot(a[0] - b[0], a[1] - b[1]),
                          resp_leader=lid, resp_gap=gap)
        except Exception as exc:
            holder["errors"].append(f"yield record {vehicle_id}: {exc!r}")
            ev.setdefault("usable_lanes", 2)
        single = ev["usable_lanes"] < 2
        counts["yields"] += 1
        counts["single_lane" if single else "multi_lane"] += 1
        counts["ambiguous_responder"] += ev.get("responder_matches", 0) > 1
        counts["no_responder"] += ev.get("responder") is None

        if variant == "production" or not single:
            before = self.last_yield.get(vehicle_id)
            real_yield(self, vehicle_id, edge_id, speed, sim_time)
            ev["action"] = "slow" + ("+move_right" if ev.get("total_lanes", 1) > 1 and ev.get("lane_index", 0) > 0 else "")
            ev["applied"] = self.last_yield.get(vehicle_id) != before
        elif variant == "keep_speed":
            ev["action"], ev["applied"] = "keep_speed", True
            bookkeeping(self, vehicle_id, sim_time)
        else:  # boost
            try:
                current = traci.vehicle.getSpeedFactor(vehicle_id)
                original = holder["boosts"][vehicle_id][1] if vehicle_id in holder["boosts"] else current
                target = max(original, ev.get("resp_factor", original))
                if target > current:
                    traci.vehicle.setSpeedFactor(vehicle_id, target)
                    counts["boosts_raising_the_factor"] += 1
                ev["car_allowed_boosted"] = traci.vehicle.getAllowedSpeed(vehicle_id)
            except traci.TraCIException as exc:   # production returns without bookkeeping when its action fails
                holder["errors"].append(f"boost {vehicle_id}: {exc!r}")
                ev["action"], ev["applied"] = "boost_failed", False
                events.append(ev)
                return
            holder["boosts"][vehicle_id] = (sim_time + duration, original)
            ev.update(action="boost", applied=True, boost_from=original, boost_to=target)
            bookkeeping(self, vehicle_id, sim_time)
        ev["trace"] = []
        events.append(ev)
        holder["open"].append(ev)

    C._yield_to_emergency = yield_action

    # -------- every step, after the coordinator: restore boosts, then observe
    real_update = C.update_step

    def update_step(self, sim_time):
        real_update(self, sim_time)
        try:
            present = set(traci.vehicle.getIDList())
            for v, (until, original) in list(holder["boosts"].items()):
                if sim_time >= until:
                    if v in present:
                        traci.vehicle.setSpeedFactor(v, original)
                        counts["boost_restores"] += 1
                    del holder["boosts"][v]
            for r in sorted(self._active_responders):
                if r not in present:
                    continue
                edge = traci.vehicle.getRoadID(r)
                row = {"t": sim_time, "r": r, "speed": traci.vehicle.getSpeed(r),
                       "allowed": traci.vehicle.getAllowedSpeed(r), "edge": edge,
                       "odo": traci.vehicle.getDistance(r)}
                if not edge.startswith(":"):
                    row["usable_lanes"] = usable_lanes(edge, "passenger")[0]
                lid, gap = leader(r, FOLLOW_M)
                if lid is not None:
                    row.update(leader=lid, gap=gap, leader_speed=traci.vehicle.getSpeed(lid),
                               leader_is_responder=lid in self._active_responders,
                               leader_yielding=0.0 < sim_time - self.last_yield.get(lid, -1e9) <= duration)
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
                               cl=traci.vehicle.getLaneIndex(c), cp=traci.vehicle.getLanePosition(c))
                if r is not None and r in present:
                    row.update(rs=traci.vehicle.getSpeed(r), ra=traci.vehicle.getAllowedSpeed(r),
                               re=traci.vehicle.getRoadID(r), rp=traci.vehicle.getLanePosition(r))
                    row["r_leader"] = leader(r, 2 * FOLLOW_M)[0]
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
    summary = {"variant": variant, "weights": weights, "rejected": rejected, "instrumentation_errors": holder["errors"],
               "counts": counts, "boosts_still_active_at_end": len(holder["boosts"]),
               "yield_actions_coordinator": holder["coordinator"].yield_actions if holder["coordinator"] else None}
    (out / "events.json").write_text(json.dumps(events), encoding="utf-8")
    (out / "responder_steps.json").write_text(json.dumps(steps), encoding="utf-8")
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(json.dumps(summary))
    return 0 if rejected is None and not holder["errors"] else 1


# ================================================================== analysis

def mean(xs):
    xs = [x for x in xs if x is not None]
    return round(sum(xs) / len(xs), 3) if xs else None


def median(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 3) if xs else None


def at(ev, k, key):
    for row in ev.get("trace", []):
        if row["k"] == k:
            return row.get(key)
    return None


def event_metrics(evs) -> dict:
    """Speeds right after the yields, and whether the responder then stayed behind the car on one road."""
    behind = [e for e in evs if e.get("resp_leader") == e["car"]]     # the responder directly behind the car
    shared_events, shared_s, shared_ratio = 0, 0, []
    for e in evs:
        rows = [r for r in e.get("trace", []) if r.get("ce") and r.get("ce") == r.get("re")
                and r.get("rp") is not None and r.get("cp") is not None and r["rp"] < r["cp"]]
        if rows:
            shared_events += 1
            shared_s += len(rows)
            shared_ratio += [r["rs"] / r["ra"] for r in rows if r.get("ra")]
    recover = []
    for e in evs:
        v0 = e.get("car_speed") or 0.0
        if v0 <= 0:
            continue
        k = next((r["k"] for r in e.get("trace", []) if r["k"] >= 1 and r.get("cs") is not None
                  and r["cs"] >= 0.95 * v0), None)
        recover.append(k if k is not None else TRACE_S + 1)
    return {
        "yields": len(evs),
        "passenger_speed_at_yield": mean([e.get("car_speed") for e in evs]),
        "passenger_speed_after_1_2_3_s": [mean([at(e, k, "cs") for e in evs]) for k in (1, 2, 3)],
        "passenger_speed_ratio_after_3_s": mean([at(e, 3, "cs") / e["car_speed"] for e in evs
                                                 if e.get("car_speed") and at(e, 3, "cs") is not None]),
        "passenger_back_to_95pct_s_median": median(recover),
        "responder_directly_behind_at_yield": len(behind),
        "responder_speed_at_yield": mean([e.get("resp_speed") for e in behind]),
        "responder_speed_after_1_2_3_s": [mean([at(e, k, "rs") for e in behind]) for k in (1, 2, 3)],
        "responder_speed_over_allowed_after_3_s": mean([at(e, 3, "rs") / at(e, 3, "ra") for e in behind
                                                       if at(e, 3, "rs") is not None and at(e, 3, "ra")]),
        "events_with_responder_behind_car_on_same_road_within_30s": shared_events,
        "seconds_responder_behind_car_on_same_road": shared_s,
        "responder_speed_over_allowed_while_behind": mean(shared_ratio),
    }


def episodes(evs) -> dict:
    by = {}
    for e in sorted(evs, key=lambda e: e["t"]):
        by.setdefault((e["car"], e.get("responder")), []).append(e["t"])
    eps = []
    for times in by.values():
        start = prev = times[0]
        n = 1
        for t in times[1:]:
            if t - prev <= EPISODE_GAP_S:
                n, prev = n + 1, t
            else:
                eps.append((n, prev - start + 3.0))
                start = prev = t
                n = 1
        eps.append((n, prev - start + 3.0))
    return {"episodes": len(eps), "yields_per_episode_mean": mean([n for n, _ in eps]),
            "yields_per_episode_max": max((n for n, _ in eps), default=0),
            "episode_s_mean": mean([d for _, d in eps]), "episode_s_max": max((d for _, d in eps), default=0)}


def following(steps) -> dict:
    """Responder speed over its allowed speed, by what is in front of it, on single- and multi-lane roads."""
    out = {}
    for lanes in ("single", "multi"):
        rows = [s for s in steps if s.get("usable_lanes") is not None
                and (s["usable_lanes"] < 2) == (lanes == "single") and s.get("allowed")]
        groups = {
            "behind_a_yielding_car": [s for s in rows if s.get("leader") and not s["leader_is_responder"]
                                      and s["gap"] <= FOLLOW_M and s["leader_yielding"]],
            "behind_a_car_not_yielding": [s for s in rows if s.get("leader") and not s["leader_is_responder"]
                                          and s["gap"] <= FOLLOW_M and not s["leader_yielding"]],
            "free": [s for s in rows if not s.get("leader")],
        }
        out[lanes] = {g: {"steps": len(v), "speed": mean([s["speed"] for s in v]),
                          "speed_over_allowed": mean([s["speed"] / s["allowed"] for s in v])}
                      for g, v in groups.items()}
    return out


def per_responder(evs, steps) -> dict:
    """For each responder (one per incident): its yields by lane type, and the seconds it spent behind a car that
    was asked to yield, by lane type, with its speed over its allowed speed in those seconds."""
    out = {}
    for r in sorted({s["r"] for s in steps}):
        mine = [e for e in evs if e.get("responder") == r]
        row = {"yields_single": sum(e.get("usable_lanes", 2) < 2 for e in mine),
               "yields_multi": sum(e.get("usable_lanes", 2) >= 2 for e in mine)}
        for lanes in ("single", "multi"):
            behind = [s for s in steps if s["r"] == r and s.get("usable_lanes") is not None
                      and (s["usable_lanes"] < 2) == (lanes == "single") and s.get("leader")
                      and not s["leader_is_responder"] and s["gap"] <= FOLLOW_M and s["leader_yielding"]]
            row[f"seconds_behind_a_yielding_car_{lanes}"] = len(behind)
            row[f"speed_over_allowed_there_{lanes}"] = mean([s["speed"] / s["allowed"] for s in behind if s["allowed"]])
        row["active_seconds"] = sum(1 for s in steps if s["r"] == r)
        out[r] = row
    return out


def multi_lane_room(evs) -> dict:
    """Multi-lane yields with the responder directly behind the car: did the car actually make room?"""
    behind = [e for e in evs if e.get("usable_lanes", 2) >= 2 and e.get("resp_leader") == e["car"]]
    rightmost = [e for e in behind if e.get("lane_index", 0) == 0]
    asked = [e for e in behind if e.get("lane_index", 0) > 0]
    moved = [e for e in asked if any(at(e, k, "cl") is not None and at(e, k, "cl") < e["lane_index"] for k in (1, 2, 3))]
    stayed = [e for e in asked if e not in moved]

    def ratio(group):
        return mean([at(e, 3, "rs") / at(e, 3, "ra") for e in group if at(e, 3, "rs") is not None and at(e, 3, "ra")])

    return {"with_responder_directly_behind": len(behind), "car_already_rightmost": len(rightmost),
            "responder_over_allowed_3s_rightmost": ratio(rightmost), "asked_to_move_right": len(asked),
            "moved_within_3s": len(moved), "responder_over_allowed_3s_moved": ratio(moved),
            "responder_over_allowed_3s_not_moved": ratio(stayed)}


def boost_detail(evs) -> dict:
    boosts = [e for e in evs if e.get("action") == "boost"]
    raised = [e for e in boosts if e.get("boost_to", 0) > e.get("boost_from", 0)]
    gains = [e["car_allowed_boosted"] - e["car_allowed"] for e in raised
             if e.get("car_allowed_boosted") is not None and e.get("car_allowed") is not None]
    return {"boosts": len(boosts), "factor_raised": len(raised), "top_speed_raised": sum(g > 1e-6 for g in gains),
            "top_speed_gain_when_raised_m_s": mean([g for g in gains if g > 1e-6])}


def moved_aside_later(evs) -> dict:
    """Cars that yielded on a single-lane road and later, to the same responder, on a road with room.
    Each car-responder pair is counted once."""
    first_single, later = {}, 0
    for e in sorted(evs, key=lambda e: e["t"]):
        key = (e["car"], e.get("responder"))
        if e.get("usable_lanes", 2) < 2:
            first_single.setdefault(key, e["t"])
        elif key in first_single:
            later += 1
            first_single.pop(key)
    return {"cars_with_a_single_lane_yield": len(first_single) + later, "of_which_later_yielded_with_room": later}


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
            "variant": summ["variant"],
            "responses_s": {e: r["value_s"] for e, r in m["responses"].items()},
            "response_status": {e: r["status"] for e, r in m["responses"].items()},
            "J": (trial["objective"] or {}).get("J"), "finished": m["finished"], "scheduled": m["vehicles_scheduled"],
            "completion_pct": round(100 * m["finished"] / m["vehicles_scheduled"], 2), "teleports": m["teleports"],
            "travel_s_finished_trips": round(m["travel_s"], 1), "time_loss_s": round(m["time_loss_s"], 1),
            "counts": summ["counts"], "episodes_single_lane": episodes(single), "episodes_multi_lane": episodes(multi),
            "single_lane": event_metrics(single), "multi_lane": event_metrics(multi),
            "moved_aside_later": moved_aside_later(evs),
            "multi_lane_room": multi_lane_room(evs),
            "boost_detail": boost_detail(evs),
            "per_responder": per_responder(evs, steps),
            "responder_following": following(steps),
            "boost_effect": {"boosts": sum(e.get("action") == "boost" for e in evs),
                             "car_allowed_before": mean([e.get("car_allowed") for e in evs if e.get("action") == "boost"]),
                             "car_allowed_boosted": mean([e.get("car_allowed_boosted") for e in evs if e.get("action") == "boost"])},
            "checks": {"exceptions": len(trial["exceptions_logged"]), "tracebacks": tracebacks,
                       "instrumentation_errors": len(summ["instrumentation_errors"]), "rejected": summ["rejected"],
                       "finished_equals_summary": trial["finished_equals_summary_arrived"],
                       "speed_factor_differences": trial["pairing"]["speed_factor_differences"],
                       "yield_counts_agree": summ["counts"]["yields"] == len(evs)
                       and summ["yield_actions_coordinator"] == trial["coordinator"]["yield_actions"]},
        }

    # Same-state comparison. The variants act differently only at a single-lane yield, so every run of one
    # weight set is the production run up to its first single-lane yield: the same traffic, the same car, the
    # same responder. What each responder covered in the next seconds is compared from that point.
    same_state = {}
    for name, (run, trial, summ, evs, steps) in runs.items():
        if summ["variant"] != "production":
            continue
        w = name.rsplit("_production", 1)[0]
        first = next((e for e in evs if e.get("usable_lanes", 2) < 2 and e.get("responder")), None)
        if first is None:
            continue
        t0, r = first["t"], first["responder"]
        section = {"t": t0, "car": first["car"], "responder": r, "edge": first["edge"], "runs": {}}
        base = None
        for other, (orun, otrial, osumm, oevs, osteps) in runs.items():
            if not other.startswith(w + "_"):
                continue
            odo = {s["t"]: s["odo"] for s in osteps if s["r"] == r}
            if t0 not in odo:
                continue
            base = odo[t0] if base is None else base
            section["runs"][osumm["variant"]] = {
                "odometer_at_t0_equal": odo[t0] == base,
                "metres_covered_after_10_20_30_60_s": [round(odo[t0 + d] - odo[t0], 1) if t0 + d in odo else None
                                                       for d in (10, 20, 30, 60)]}
        same_state[w] = section

    def summary_steps(run):
        return records(run / "trial" / "summary.xml", "step", ignore=("duration",))

    divergence = {}
    for name, (run, *_rest) in runs.items():
        variant = report[name]["variant"]
        w = name.rsplit("_" + variant, 1)[0]
        if variant == "production" or f"{w}_production" not in runs:
            continue
        a, b = summary_steps(runs[f"{w}_production"][0]), summary_steps(run)
        divergence[name] = next((x[0].get("time") for x, y in zip(a, b) if x != y), None)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "analysis.json").write_text(json.dumps({"runs": report, "same_state_first_single_lane_yield": same_state,
                                                   "first_differing_step_from_production": divergence}, indent=1),
                                       encoding="utf-8")

    for name, r in report.items():
        print(f"\n=== {name} ===  responses {r['responses_s']}  J {r['J']}  finished {r['finished']}/{r['scheduled']} "
              f"({r['completion_pct']}%)  teleports {r['teleports']}  travel {r['travel_s_finished_trips']} s  "
              f"time loss {r['time_loss_s']} s")
        print(f"  counts {r['counts']}")
        print(f"  episodes single {r['episodes_single_lane']}  multi {r['episodes_multi_lane']}")
        for lanes in ("single_lane", "multi_lane"):
            print(f"  {lanes}: {r[lanes]}")
        print(f"  responder following: {r['responder_following']}")
        print(f"  boost: {r['boost_effect']}  checks: {r['checks']}")
    print(f"\nsame state, first single-lane yield: {json.dumps(same_state, indent=1)}")
    print(f"first differing step from production: {divergence}")
    print(f"\nwritten: {out / 'analysis.json'}")
    return 0


def table(args) -> int:
    """The comparison as markdown tables, from analysis.json."""
    data = json.loads(Path(args.analysis).read_text(encoding="utf-8"))
    runs = data["runs"]
    names = [f"{w}_{v}" for w in ("def", "t0", "t1") for v in VARIANTS if f"{w}_{v}" in runs]

    def f(x, d=1):
        return "-" if x is None else f"{x:.{d}f}"

    print("\n**Outcomes**\n")
    print("| run | response 404 / 401 / 405 | J | finished of 4,003 | teleports | travel, finished trips | time loss |")
    print("|---|---|---|---|---|---|---|")
    for n in names:
        r = runs[n]
        resp = " / ".join(f(r["responses_s"].get(e), 0) for e in ("404", "401", "405"))
        print(f"| {n} | {resp} s | {f(r['J'], 4)} | {r['finished']:,} ({r['completion_pct']}%) | {r['teleports']} | "
              f"{r['travel_s_finished_trips']} s | {r['time_loss_s']} s |")

    print("\n**Yields**\n")
    print("| run | yields (single / multi) | single-lane episodes: yields each, mean / max | single-lane episode "
          "length, mean / max | single-lane cars that later yielded with room |")
    print("|---|---|---|---|---|")
    for n in names:
        r = runs[n]
        c, e = r["counts"], r["episodes_single_lane"]
        print(f"| {n} | {c['yields']} ({c['single_lane']} / {c['multi_lane']}) | {f(e['yields_per_episode_mean'], 2)} / "
              f"{e['yields_per_episode_max']} | {f(e['episode_s_mean'])} / {f(e['episode_s_max'], 0)} s | "
              f"{r['moved_aside_later']['of_which_later_yielded_with_room']} of "
              f"{r['moved_aside_later']['cars_with_a_single_lane_yield']} |")

    print("\n**Right after a single-lane yield** (speeds in m/s; the responder rows are yields with the responder "
          "directly behind the car)\n")
    print("| run | passenger at yield -> +1 / +2 / +3 s | passenger back to 95% (median) | responder behind: "
          "at yield -> +1 / +2 / +3 s | responder / its allowed speed at +3 s | yields followed by the responder "
          "behind the car on one road; seconds | responder / allowed while behind |")
    print("|---|---|---|---|---|---|---|")
    for n in names:
        s = runs[n]["single_lane"]
        p = " / ".join(f(x) for x in s["passenger_speed_after_1_2_3_s"])
        q = " / ".join(f(x) for x in s["responder_speed_after_1_2_3_s"])
        print(f"| {n} | {f(s['passenger_speed_at_yield'])} -> {p} | {f(s['passenger_back_to_95pct_s_median'])} s | "
              f"({s['responder_directly_behind_at_yield']}) {f(s['responder_speed_at_yield'])} -> {q} | "
              f"{f(s['responder_speed_over_allowed_after_3_s'], 2)} | "
              f"{s['events_with_responder_behind_car_on_same_road_within_30s']} of {s['yields']}; "
              f"{s['seconds_responder_behind_car_on_same_road']} s | {f(s['responder_speed_over_allowed_while_behind'], 2)} |")

    print("\n**Responder speed over its allowed speed, every step, by what is ahead of it within 50 m** (steps)\n")
    print("| run | single: behind a yielding car | single: behind a car not yielding | single: free | "
          "multi: behind a yielding car | multi: behind a car not yielding | multi: free |")
    print("|---|---|---|---|---|---|---|")
    for n in names:
        fo = runs[n]["responder_following"]
        cells = [f"{f(fo[l][g]['speed_over_allowed'], 2)} ({fo[l][g]['steps']})" for l in ("single", "multi")
                 for g in ("behind_a_yielding_car", "behind_a_car_not_yielding", "free")]
        print(f"| {n} | " + " | ".join(cells) + " |")

    print("\n**Multi-lane yields with the responder directly behind the car** (responder speed over its allowed "
          "speed 3 s later)\n")
    print("| run | such yields | car already rightmost (responder at +3 s) | asked to move right | moved within 3 s "
          "(responder at +3 s) | did not move (responder at +3 s) |")
    print("|---|---|---|---|---|---|")
    for n in names:
        m = runs[n]["multi_lane_room"]
        print(f"| {n} | {m['with_responder_directly_behind']} | {m['car_already_rightmost']} "
              f"({f(m['responder_over_allowed_3s_rightmost'], 2)}) | {m['asked_to_move_right']} | {m['moved_within_3s']} "
              f"({f(m['responder_over_allowed_3s_moved'], 2)}) | {m['asked_to_move_right'] - m['moved_within_3s']} "
              f"({f(m['responder_over_allowed_3s_not_moved'], 2)}) |")
    boosted = [n for n in names if runs[n]["boost_detail"]["boosts"]]
    if boosted:
        print("\n**Boosts** (did raising the speed factor raise the car's own top speed on that road?)\n")
        print("| run | boosts | factor raised | top speed raised | mean gain when raised |")
        print("|---|---|---|---|---|")
        for n in boosted:
            b = runs[n]["boost_detail"]
            print(f"| {n} | {b['boosts']} | {b['factor_raised']} | {b['top_speed_raised']} | "
                  f"{f(b['top_speed_gain_when_raised_m_s'], 2)} m/s |")

    print("\n**Per incident** (single-lane yields; seconds behind a car asked to yield on a single-lane road; "
          "the responder's speed over its allowed speed then; seconds active)\n")
    print("| run | 404 | 401 | 405 |")
    print("|---|---|---|---|")
    for n in names:
        pr = runs[n]["per_responder"]
        cells = []
        for inc in ("404", "401", "405"):
            x = pr.get(f"emergency_dispatch_{inc}")
            cells.append("-" if x is None else f"{x['yields_single']} yields; {x['seconds_behind_a_yielding_car_single']} s "
                         f"at {f(x['speed_over_allowed_there_single'], 2)}; {x['active_seconds']} s")
        print(f"| {n} | " + " | ".join(cells) + " |")

    print("\n**Same state: the first single-lane yield of each weight set** (metres the responder covered after "
          "10 / 20 / 30 / 60 s)\n")
    print("| weights | t, car, responder | production | keep_speed | boost | odometers equal at t |")
    print("|---|---|---|---|---|---|")
    for w, s in data["same_state_first_single_lane_yield"].items():
        cells = [" / ".join(f(x, 0) for x in s["runs"][v]["metres_covered_after_10_20_30_60_s"])
                 if v in s["runs"] else "-" for v in VARIANTS]
        equal = all(v["odometer_at_t0_equal"] for v in s["runs"].values())
        print(f"| {w} | {s['t']:.0f} s, {s['car']}, {s['responder']} | " + " | ".join(cells) + f" | {equal} |")
    print(f"\nfirst differing summary step from production: {data['first_differing_step_from_production']}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="one instrumented tuner trial with the given single-lane action")
    r.add_argument("--variant", choices=VARIANTS, required=True)
    source = r.add_mutually_exclusive_group(required=True)
    source.add_argument("--weights-from", help="a trial.json whose weights_requested are used")
    source.add_argument("--default-weights", action="store_true")
    r.add_argument("--sim-time", type=int, default=3600)
    r.add_argument("--unscored", action="store_true", help="tool check only: no reference, no J (for short runs)")
    r.add_argument("--reference", default=str(REFERENCE), help="the reference that scores J (must match --sim-time)")
    r.add_argument("--out", required=True)
    a = sub.add_parser("analyse", help="the comparison tables")
    a.add_argument("--out", required=True)
    a.add_argument("runs", nargs="+")
    t = sub.add_parser("table", help="markdown tables from an analysis.json")
    t.add_argument("analysis")
    args = parser.parse_args()
    return {"run": live_run, "analyse": analyse, "table": table}[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
