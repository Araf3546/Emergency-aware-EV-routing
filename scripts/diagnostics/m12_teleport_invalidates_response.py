#!/usr/bin/env python3
"""Audit M12, verification: a teleported responder must not produce a response time.

Runs the real network, the real 4,003-vehicle demand and the real incident scenario, and
reproduces on demand the event that happened by chance in the P3 arm on 2026-09-08:

    Warning: Teleporting vehicle 'emergency_dispatch_401'; waited too long (jam),
             lane='190128487#2_0', time=2260.00.
    Warning: Vehicle 'emergency_dispatch_401' ends teleporting on edge '734630276',
             time=2260.00.

`734630276` is incident 401's own centre edge, so the arrival test in contract decision
13.3 fired on a trip that was never driven and reported 1,361 s.

To reproduce it deliberately, `--jam-responder 401` holds that responder at a standstill
wherever it happens to be. SUMO's own jam-teleport logic then fires and drops it further
along its own route; held still again, it is teleported again, until a teleport lands it on
the centre edge and the arrival test fires on a trip that was never driven. The teleports
are genuinely SUMO's, produced by a genuine standstill, in a genuine run. Pair it with a
short `--time-to-teleport` so the hops take minutes rather than the whole hour.

The P3 coordinator is deliberately not used. The teleport watch lives in EmergencyManager,
which both arms drive identically, so the plain-SUMO arm exercises exactly the code under
test and the whole hour runs in about a minute instead of twenty.

Writes nothing into the project - SUMO output goes to a temporary directory.

    python scripts/diagnostics/m12_teleport_invalidates_response.py \
        --duration 2400 --jam-responder 401 --time-to-teleport 20
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

import traci  # noqa: E402

from emergency_manager import get_emergency_manager  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=3600)
    parser.add_argument("--time-to-teleport", type=int, default=None,
                        help="SUMO jam patience in seconds; omitted means SUMO's own default")
    parser.add_argument("--jam-responder", type=int, default=None, metavar="EVENT_ID",
                        help="hold this event's responder at a standstill, so SUMO's jam "
                             "teleport fires on it")
    parser.add_argument("--sumo-cfg", default=str(REPO_ROOT / "simulation.sumocfg"))
    arguments = parser.parse_args()

    out_dir = Path(tempfile.mkdtemp(prefix="m12_teleport_"))

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    command = [
        "sumo", "-c", arguments.sumo_cfg,
        "--no-step-log", "true",
        "--end", str(arguments.duration),
        "--tripinfo-output", str(out_dir / "tripinfo.xml"),
        "--summary-output", str(out_dir / "summary.xml"),
        "--error-log", str(out_dir / "sumo_errors.log"),
    ]
    if arguments.time_to_teleport is not None:
        command += ["--time-to-teleport", str(arguments.time_to_teleport)]
    traci.start(command)

    jam_vehicle = None
    if arguments.jam_responder is not None:
        event = manager.events[arguments.jam_responder]
        jam_vehicle = event.dispatch_vehicle_id
        print(f"[M12] will hold {jam_vehicle} at a standstill wherever it is, until "
              f"SUMO teleports it onto centre edge '{event.center_edge}'")

    try:
        while traci.simulation.getTime() < arguments.duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            # Hold the responder still. SUMO decides on its own when that standstill has
            # lasted long enough to be a jam worth teleporting out of; nothing here moves
            # the vehicle or touches the arrival test.
            if jam_vehicle is not None:
                try:
                    road = traci.vehicle.getRoadID(jam_vehicle)
                    traci.vehicle.setSpeed(jam_vehicle, 0.0)
                    if road and int(sim_time) % 120 == 0:
                        print(f"[M12] t={sim_time:.0f} {jam_vehicle} held on '{road}', "
                              f"waiting={traci.vehicle.getWaitingTime(jam_vehicle):.0f}s")
                except traci.TraCIException:
                    pass

            manager.update(sim_time)
            manager.record_responder_arrivals(
                sim_time,
                vehicle_ids=traci.vehicle.getIDList(),
                arrived_ids=traci.simulation.getArrivedIDList(),
            )
    finally:
        try:
            traci.close()
        except Exception:
            pass

    summary = manager.get_response_summary()
    teleports = manager.get_teleport_summary()

    print("\n=== responder teleports (audit M12) ===")
    print(json.dumps(teleports, indent=2))

    print("\n=== response summary ===")
    for event_id in sorted(summary):
        state = summary[event_id]
        print(
            f"  event {event_id}: measured={state['measured']} "
            f"valid={state['response_valid']} "
            f"response_time_s={state['response_time_s']} "
            f"unvalidated={state['unvalidated_response_time_s']} "
            f"teleported={state['responder_teleported']} "
            f"reason={state['invalid_reason']}"
        )

    invalidated = teleports["events_invalidated_by_teleport"]
    print("\n=== verdict ===")
    if not invalidated:
        print("  No responder teleported in this run - the fix was not exercised. "
              "Lower --time-to-teleport and run again.")
        print(f"  SUMO error log: {out_dir / 'sumo_errors.log'}")
        return 2

    failures = []
    for event_id in invalidated:
        state = summary[event_id]
        if state["response_time_s"] is not None:
            failures.append(f"event {event_id} still reports a response time")
        if state["measured"] and state["unvalidated_response_time_s"] is None:
            failures.append(f"event {event_id} dropped the observed number silently")

    if failures:
        for line in failures:
            print(f"  FAIL: {line}")
        return 1

    print(f"  PASS: {len(invalidated)} event(s) invalidated by a responder teleport "
          f"{invalidated}; none of them reports a response time, and each keeps the "
          f"number it would have reported.")
    print(f"  SUMO error log: {out_dir / 'sumo_errors.log'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
