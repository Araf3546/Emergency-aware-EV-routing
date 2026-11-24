#!/usr/bin/env python3
"""PD-1 verification: the active-responder flag, on a real run, against a saved reference.

    # the P3 treatment arm with the PD-1 code, instrumented read-only, same parameters as the
    # saved reference run (seed 20260721, 1,800 s, layer ON, default weights)
    python scripts/diagnostics/pd1_verify.py --run --out DIR

    # compare with the reference recorded before PD-1
    python scripts/diagnostics/pd1_verify.py --compare DIR \
        --reference results/e3_component_validation_2026-09-12

The run is `e3_component_reachability.py --run` (the instrumentation that produced the reference,
so the two decision logs are directly comparable) with three more read-only recorders:

* every reroute-trigger firing, by who fired it - active responder, ambient emergency-class
  vehicle, ordinary car - and whether it came off the 30 s clock (only the every-step clause
  can do that);
* every time the OLD type clause would have fired ("emergency-class vehicle on a closed edge"),
  so the removed firings are counted, not inferred;
* at every step, whether the manager's `active_responder_ids` equals the protocol layer's own
  `_active_responders` - one definition for both.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
os.chdir(REPO_ROOT)

import traci  # noqa: E402


def run(args) -> int:
    import e3_component_reachability as reach
    import p3_runtime
    from emergency_manager import get_emergency_manager
    from p3_simulation_config import EMERGENCY_VEHICLE_TYPES

    out = Path(args.out)
    manager = get_emergency_manager()
    firings = Counter()
    old_clause = Counter()
    events = []
    flag_steps = {"steps": 0, "mismatches": 0, "steps_with_a_responder": 0}
    original_trigger = p3_runtime.should_reroute

    def trigger(vehicle_id):
        fired = original_trigger(vehicle_id)
        try:
            t = traci.simulation.getTime()
            edge = traci.vehicle.getRoadID(vehicle_id)
            emergency_class = traci.vehicle.getTypeID(vehicle_id) in EMERGENCY_VEHICLE_TYPES
        except traci.TraCIException:
            return fired
        who = ("active_responder" if manager.is_active_responder(vehicle_id)
               else "ambient_emergency" if emergency_class else "ordinary")
        on_closed = manager.is_edge_blocked(edge)
        if emergency_class and on_closed and not edge.startswith(":"):
            old_clause[who] += 1
            events.append({"t": t, "vehicle": vehicle_id, "who": who, "edge": edge,
                           "old_clause_would_fire": True, "fired_now": bool(fired)})
        if fired:
            firings[f"{who}|{'on_clock' if int(t) % 30 == 0 else 'off_clock'}"] += 1
        return fired

    p3_runtime.should_reroute = trigger
    original_step = p3_runtime.P3RuntimeCoordinator.update_step

    def update_step(self, sim_time):
        original_step(self, sim_time)
        flag_steps["steps"] += 1
        if set(self._active_responders) != set(manager.active_responder_ids):
            flag_steps["mismatches"] += 1
        if manager.active_responder_ids:
            flag_steps["steps_with_a_responder"] += 1

    p3_runtime.P3RuntimeCoordinator.update_step = update_step
    code = reach.run(args)
    (out / "pd1.json").write_text(json.dumps({
        "trigger_firings": dict(sorted(firings.items())),
        "old_type_clause_would_have_fired": dict(old_clause),
        "old_type_clause_events": events,
        "flag_vs_protocol_layer": flag_steps,
    }, indent=1), encoding="utf-8")
    print(json.dumps({"firings": dict(firings), "old_clause": dict(old_clause), "flag": flag_steps}))
    return code


def compare(directory: Path, reference: Path) -> int:
    def tripinfo(path):
        import xml.etree.ElementTree as ET
        return {e.get("id"): e.attrib for e in ET.parse(path).getroot().iter("tripinfo")}

    def key(d):
        return (d["t"], d["vehicle"], tuple(d["edges"]), d["chosen"])

    new = json.loads((directory / "decisions.json").read_text(encoding="utf-8"))
    ref = json.loads((reference / "decisions.json").read_text(encoding="utf-8"))
    removed = [d for d in ref if d["vehicle"] == "emergency_00062" and d["t"] % 30 != 0]
    ref_without = [d for d in ref if d not in removed]
    # Full records (every recorded value of every candidate), then the looser (time, vehicle,
    # candidates, chosen) key: the first shows where anything at all differs, the second where a
    # different edge was chosen or a different vehicle decided.
    full = lambda d: json.dumps(d, sort_keys=True)
    first_diff = next((i for i, (a, b) in enumerate(zip(map(full, new), map(full, ref_without))) if a != b), None)
    first_choice_diff = next((i for i, (a, b) in enumerate(zip(map(key, new), map(key, ref_without))) if a != b), None)
    change_time = min(d["t"] for d in removed) if removed else None
    before = [d for d in ref_without if change_time is not None and d["t"] < change_time]
    trips_new, trips_ref = tripinfo(directory / "tripinfo.xml"), tripinfo(reference / "tripinfo.xml")
    common = set(trips_new) & set(trips_ref)
    meta_new = json.loads((directory / "meta.json").read_text(encoding="utf-8"))
    meta_ref = json.loads((reference / "meta.json").read_text(encoding="utf-8"))
    result = {
        "reference_decisions": len(ref),
        "new_decisions": len(new),
        "reference_offclock_decisions_by_emergency_00062": [(d["t"], len(d["edges"]), d["chosen"]) for d in removed],
        "new_decisions_identical_to_reference_minus_those": [key(d) for d in new] == [key(d) for d in ref_without],
        "decisions_before_the_first_removed_one": len(before),
        "all_of_them_identical": [full(d) for d in new[:len(before)]] == [full(d) for d in before],
        "first_differing_decision_index": first_diff,
        "first_differing_decision": (new[first_diff] if first_diff is not None and first_diff < len(new) else None),
        "first_differing_choice_index": first_choice_diff,
        "trips_finished_before_the_change_identical": all(
            trips_new.get(k) == v for k, v in trips_ref.items()
            if change_time is not None and float(v["arrival"]) < change_time),
        "tripinfo_identical": trips_new == trips_ref,
        "tripinfo_records": (len(trips_new), len(trips_ref)),
        "tripinfo_records_differing": sum(1 for k in common if trips_new[k] != trips_ref[k]),
        "counters_new": meta_new["counters"],
        "counters_reference": meta_ref["counters"],
        "pairing_new": meta_new["pairing"],
    }
    (directory / "pd1_compare.json").write_text(json.dumps(result, indent=1, default=str), encoding="utf-8")
    print(json.dumps(result, indent=1, default=str))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--compare", type=Path, default=None)
    parser.add_argument("--reference", type=Path, default=Path("results/e3_component_validation_2026-09-12"))
    parser.add_argument("--seed", type=int, default=20260721)
    parser.add_argument("--duration", type=int, default=1800)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    if args.compare:
        return compare(args.compare, args.reference)
    if not args.run or not args.out:
        parser.error("--run --out DIR, or --compare DIR")
    Path(args.out).mkdir(parents=True, exist_ok=True)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
