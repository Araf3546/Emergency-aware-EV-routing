"""Ask SUMO directly what findRoute returns for a car routed to a closed incident edge.

Background (2026-09-11). EmergencyManager._route_distance promises "infinity if there is
no route". But when an incident closes its centre edge to every class except emergency,
SUMO's findRoute does not raise for an ordinary car - it returns an empty route with length
0.0, and _route_distance reads that as "0 metres away". Every ordinary car in the network
is then inside the 200 m threshold and broadcasts yield_request = True for the whole incident.

Expected output: before the incident, the same real distance for both vehicle types; after
it opens, SUMO's raw answer is still 0 edges and length 0.00 for the passenger type (that is
SUMO's behaviour and is left alone), while the production distance - since the fix, made with
a closure-exempt twin of the car's own type - equals the pre-closure distance and prints PASS.

    python scripts/diagnostics/closure_distance_check.py

Read-only. Plain SUMO (no P3 coordinator), same config and seed. The same query is made
just before incident 404 opens and just after, for an ordinary vehicle type and for an
emergency one, and the raw result is printed.
"""
import os
import sys
from pathlib import Path

ROOT = Path(os.environ.get("PROBE_ROOT", Path(__file__).resolve().parent.parent.parent))
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

import traci  # noqa: E402
from emergency_manager import get_emergency_manager  # noqa: E402

manager = get_emergency_manager()
manager.load_canonical_scenario_file()
manager.generate_emergencies()
centre = manager.events[404].center_edge

traci.start(["sumo", "-c", "simulation.sumocfg", "--no-step-log", "true", "--end", "620",
             "--emission-output", "NUL", "--queue-output", "NUL", "--message-log", "NUL",
             "--tripinfo-output", "NUL", "--summary-output", "NUL"])
origin = None
try:
    while traci.simulation.getTime() < 612:
        traci.simulationStep()
        t = traci.simulation.getTime()
        manager.update(t)
        if origin is None and t >= 590:
            for v in traci.vehicle.getIDList():
                e = traci.vehicle.getRoadID(v)
                if v.startswith("normal_") and not e.startswith(":"):
                    origin = e
                    break
        if t in (595.0, 610.0):
            print(f"t={t:.0f}  incident 404 active: {404 in manager.active_events}  centre={centre}")
            for vtype in ("normal_passenger", "emergency_ambulance"):
                try:
                    r = traci.simulation.findRoute(origin, centre, vtype)
                    print(f"   {vtype:<20} edges={len(r.edges):>3}  length={r.length:>9.2f}  "
                          f"travelTime={r.travelTime:>8.2f}")
                except traci.TraCIException as exc:
                    print(f"   {vtype:<20} RAISED: {exc}")
            # What the production code reports for the same car's edge since the 2026-09-11
            # fix: the query goes through a closure-exempt twin of the car's own type, so it
            # must be the pre-closure distance, never 0 and never "no route".
            if 404 in manager.active_events:
                twin = manager._distance_query_type("normal_passenger")
                d = manager._route_distance(origin, centre, twin)
                print(f"   production distance for a car (via {twin}): {d:.2f} m")
                ok = d != 0.0 and d != float("inf")
                print(f"   {'PASS' if ok else 'FAIL'}: an ordinary car is given a real road distance "
                      f"during the closure")
finally:
    traci.close()
