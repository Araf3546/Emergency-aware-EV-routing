#!/usr/bin/env python3
"""Are E3's reroutes individually good or bad? Asked before making it reroute more often.

The three-arm ablation showed E3 routing alone costs the network 51 s of mean travel time
and 188 completed trips against plain SUMO, while applying only 504 route changes in an
hour. Two very different explanations fit that:

  (a) each reroute is harmful - E3 sends vehicles onto longer or worse paths, so the damage
      is proportional to how often it acts;
  (b) each reroute is fine - 390 vehicles changing path perturbs a chaotic simulation and
      the network-level difference is divergence rather than damage.

Which one is true decides whether the planned junction-triggered rerouting fix helps or
hurts. That fix multiplies the number of reroutes, so under (a) it would make the thesis
worse, and E3's scoring would have to be fixed first.

This measures the remaining distance a vehicle has left to drive, immediately before and
immediately after every reroute E3 actually applies. Destination is unchanged by
construction (audit C1), so the difference is the detour E3 chose.

The emergency layer is switched OFF so this measures the router alone.

Writes nothing into the project.

    python scripts/diagnostics/reroute_quality.py --duration 1800
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402
import p3_runtime  # noqa: E402


def edge_lengths() -> dict:
    lengths = {}
    for edge in ET.parse(REPO_ROOT / "network" / "porto.net.xml").getroot().findall("edge"):
        if edge.get("function") == "internal":
            continue
        lanes = edge.findall("lane")
        if lanes:
            lengths[edge.get("id")] = max(float(lane.get("length", 0.0)) for lane in lanes)
    return lengths


def remaining_distance(route, current_edge, lengths) -> float:
    """Distance still to drive along `route`, counting from the current edge onward."""
    try:
        index = route.index(current_edge)
    except ValueError:
        index = 0
    return sum(lengths.get(edge, 0.0) for edge in route[index:])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=1800)
    arguments = parser.parse_args()

    lengths = edge_lengths()
    out_dir = Path(tempfile.mkdtemp(prefix="reroute_quality_"))
    samples = []

    _orig_set_route = p3_runtime.set_route_checked

    def set_route_checked(vehicle_id, new_route, current_route):
        try:
            current_edge = traci.vehicle.getRoadID(vehicle_id)
        except traci.TraCIException:
            current_edge = ""
        applied = _orig_set_route(vehicle_id, new_route, current_route)
        if applied:
            before = remaining_distance(list(current_route), current_edge, lengths)
            after = remaining_distance(list(new_route), current_edge, lengths)
            samples.append({
                "vehicle": vehicle_id,
                "before_m": round(before, 1),
                "after_m": round(after, 1),
                "delta_m": round(after - before, 1),
                "same_destination": current_route[-1] == new_route[-1],
            })
        return applied

    p3_runtime.set_route_checked = set_route_checked

    from p3_runtime import P3RuntimeCoordinator
    from emergency_manager import get_emergency_manager

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator = P3RuntimeCoordinator(emergency_layer=False)

    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true",
        "--end", str(arguments.duration),
        "--tripinfo-output", str(out_dir / "tripinfo.xml"),
        "--error-log", str(out_dir / "errors.log"),
    ])
    try:
        while traci.simulation.getTime() < arguments.duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)
            coordinator.update_step(sim_time)
    finally:
        try:
            traci.close()
        except Exception:
            pass

    if not samples:
        print("No reroutes were applied - nothing to measure.")
        return 2

    deltas = [s["delta_m"] for s in samples]
    longer = [d for d in deltas if d > 1.0]
    shorter = [d for d in deltas if d < -1.0]
    same = [d for d in deltas if -1.0 <= d <= 1.0]

    print(f"\n=== {len(samples)} applied reroutes over {arguments.duration}s, emergency layer OFF ===")
    print(f"  destination preserved in all      : {all(s['same_destination'] for s in samples)}")
    print(f"  remaining distance BEFORE (median): {statistics.median(s['before_m'] for s in samples):.0f} m")
    print(f"  remaining distance AFTER  (median): {statistics.median(s['after_m'] for s in samples):.0f} m")
    print()
    print(f"  reroutes that LENGTHEN the trip   : {len(longer)} ({100*len(longer)/len(samples):.1f}%)")
    print(f"  reroutes that SHORTEN the trip    : {len(shorter)} ({100*len(shorter)/len(samples):.1f}%)")
    print(f"  no material change (+/-1 m)       : {len(same)} ({100*len(same)/len(samples):.1f}%)")
    print()
    print(f"  mean change in remaining distance : {statistics.mean(deltas):+.1f} m")
    print(f"  median change                     : {statistics.median(deltas):+.1f} m")
    if longer:
        print(f"  mean detour when it lengthens     : {statistics.mean(longer):+.1f} m")
        print(f"  worst detour                      : {max(deltas):+.1f} m")
    if shorter:
        print(f"  mean saving when it shortens      : {statistics.mean(shorter):+.1f} m")

    total_added = sum(deltas)
    print(f"\n  net distance added across all reroutes: {total_added:+.0f} m")

    print("\n=== reading ===")
    if statistics.mean(deltas) > 5.0:
        print("  E3's reroutes lengthen trips on average. The damage scales with how often it")
        print("  acts, so making it decide more often would make the network result WORSE.")
        print("  Fix the scoring (A10, PSO fitness) before junction-triggered rerouting.")
    elif statistics.mean(deltas) < -5.0:
        print("  E3's reroutes shorten trips on average, so acting more often should help.")
        print("  The network penalty is then more likely chaotic divergence than damage.")
    else:
        print("  E3's reroutes barely change trip distance either way. The network penalty is")
        print("  then not explained by detour length - it is either chaotic divergence or a")
        print("  cost of *when* vehicles change path rather than *where* they go.")
    print(f"\n  detail: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
