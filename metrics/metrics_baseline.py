"""DEAD CODE - do not use. Superseded by metrics/metrics_logger.py.

Quarantined during Step 4 of the audit repair plan. Three reasons, all from the audit:

* M8 - it records `"end_time": MAX_SIM_TIME` for vehicles that never arrive, so a vehicle
  that never completed its trip is counted as having arrived at t=7000 and is included in
  the travel-time average. That silently reports a 100% completion rate.
* M1 - it accumulates waiting time with `+=` every step from `getWaitingTime()`, which
  overcounts by a factor of 15.5 on this scenario (measured).
* H1 - it calls `traci.start()` at module import time, so merely importing this file
  launches a simulation. Nothing imports it, and nothing should.

The file is kept rather than deleted because this project is not a git repository, so a
deletion would be unrecoverable. Everything it did is now done correctly, from SUMO's own
tripinfo output, by `metrics/trip_metrics.py` and `metrics/metrics_logger.py`.

The original body follows, unexecuted: the guard below stops it running if it is invoked.
"""

import sys as _sys

if __name__ == "__main__":
    print(__doc__)
    _sys.exit(2)

_QUARANTINED = True
raise ImportError(
    "metrics/metrics_baseline.py is quarantined dead code (audit M8, H1). "
    "Use metrics/metrics_logger.py, which derives its metrics from tripinfo.xml."
)

# import sys
# import os
# sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
#
# import traci
# import csv
#
# from controller.reroute_logic import is_emergency_vehicle, get_vehicle_type
# from emergency_manager import get_emergency_manager
#
# SUMO_BINARY = "sumo"  # change to "sumo" for no GUI
# SUMO_CFG = "simulation.sumocfg"
#
# traci.start([SUMO_BINARY, "-c", SUMO_CFG])
#
# metrics = {}
# MAX_SIM_TIME = 7000  # seconds
# emergency_mgr = get_emergency_manager()
#
# # Track normal vs emergency conditions for congestion
# normal_speeds = {}
# emergency_speeds = {}
#
# while traci.simulation.getTime() < MAX_SIM_TIME:
#     traci.simulationStep()
#     sim_time = traci.simulation.getTime()
#
#     # Update emergency manager
#     emergency_mgr.update(sim_time)
#     active_emergencies = emergency_mgr.get_active_emergency_zones()
#
#     for veh in traci.vehicle.getIDList():
#         if veh not in metrics:
#             veh_type = get_vehicle_type(veh)
#             metrics[veh] = {
#                 "start_time": sim_time,
#                 "end_time": MAX_SIM_TIME,
#                 "waiting_time": 0.0,
#                 "distance": 0.0,
#                 "vehicle_type": veh_type,
#                 "is_emergency": is_emergency_vehicle(veh),
#                 "emergency_response_time": None,
#                 "max_speed_observed": 0.0
#             }
#
#         # Accumulate waiting time
#         try:
#             metrics[veh]["waiting_time"] += traci.vehicle.getWaitingTime(veh)
#             metrics[veh]["distance"] = traci.vehicle.getDistance(veh)
#             current_speed = traci.vehicle.getSpeed(veh)
#             metrics[veh]["max_speed_observed"] = max(metrics[veh]["max_speed_observed"], current_speed)
#
#             # Track speeds during normal vs emergency
#             current_edge = traci.vehicle.getRoadID(veh)
#             if current_edge and not current_edge.startswith(":"):
#                 if active_emergencies:
#                     if current_edge not in emergency_speeds:
#                         emergency_speeds[current_edge] = []
#                     emergency_speeds[current_edge].append(current_speed)
#                 else:
#                     if current_edge not in normal_speeds:
#                         normal_speeds[current_edge] = []
#                     normal_speeds[current_edge].append(current_speed)
#
#         except traci.TraCIException:
#             pass
#
#     # Detect vehicles that have arrived
#     arrived = traci.simulation.getArrivedIDList()
#     for veh in arrived:
#         if veh in metrics:
#             metrics[veh]["end_time"] = sim_time
#
#     # Emergency response tracking. Handled by the emergency manager so this runner uses
#     # the same arrival definition as every other one, and so a responder arriving after
#     # its incident window closes is still measured (contract sections 7 and 8).
#     for event_id, response_time in emergency_mgr.record_responder_arrivals(
#         sim_time, vehicle_ids=traci.vehicle.getIDList(), arrived_ids=arrived
#     ).items():
#         responder = emergency_mgr.events[event_id].first_arrival_vehicle
#         if responder in metrics and metrics[responder]["emergency_response_time"] is None:
#             metrics[responder]["emergency_response_time"] = response_time
#
# traci.close()
#
# # Calculate congestion level
# avg_congestion = 1.0
# if normal_speeds and emergency_speeds:
#     emergency_speeds_avg = sum([sum(v) / len(v) for v in emergency_speeds.values()]) / len(emergency_speeds)
#     normal_speeds_avg = sum([sum(v) / len(v) for v in normal_speeds.values()]) / len(normal_speeds)
#
#     if normal_speeds_avg > 0:
#         avg_congestion = emergency_speeds_avg / normal_speeds_avg
#         avg_congestion = max(0.0, min(1.0, avg_congestion))
#
# # Save results
# os.makedirs("results", exist_ok=True)
# output_file = "results/baseline_results.csv"
#
# with open(output_file, "w", newline="") as f:
#     writer = csv.writer(f)
#     writer.writerow([
#         "VehicleID",
#         "VehicleType",
#         "TravelTime",
#         "WaitingTime",
#         "Distance",
#         "IsEmergency",
#         "EmergencyResponseTime",
#         "CongestionLevel"
#     ])
#
#     for veh, data in metrics.items():
#         if data["end_time"] is not None:
#             travel_time = data["end_time"] - data["start_time"]
#             emergency_response_time = data["emergency_response_time"] if data["emergency_response_time"] is not None else "N/A"
#
#             writer.writerow([
#                 veh,
#                 data["vehicle_type"],
#                 round(travel_time, 2),
#                 round(data["waiting_time"], 2),
#                 round(data["distance"], 2),
#                 "Yes" if data["is_emergency"] else "No",
#                 round(emergency_response_time, 2) if isinstance(emergency_response_time, float) else emergency_response_time,
#                 round(avg_congestion, 4)
#             ])
#
# print(f"[INFO] Baseline results saved to {output_file}")
#
# # Save summary statistics
# summary_file = "results/baseline_summary.txt"
# with open(summary_file, "w") as f:
#     f.write("=== BASELINE SIMULATION SUMMARY ===\n\n")
#     f.write(f"Simulation Time: {MAX_SIM_TIME} seconds\n")
#     f.write(f"Total Vehicles: {len(metrics)}\n")
#     f.write(f"Emergency Vehicles: {sum(1 for v in metrics.values() if v['is_emergency'])}\n")
#     f.write(f"Passenger Vehicles: {sum(1 for v in metrics.values() if not v['is_emergency'])}\n\n")
#
#     f.write("=== CONGESTION ANALYSIS ===\n")
#     f.write(f"Congestion Ratio (Emergency/Normal): {avg_congestion:.4f}\n")
#     f.write(f"Speed Reduction During Emergency: {(1 - avg_congestion) * 100:.2f}%\n\n")
#
#     # Emergency vehicle statistics
#     emergency_vehicles = {v: d for v, d in metrics.items() if d["is_emergency"]}
#     if emergency_vehicles:
#         f.write("=== EMERGENCY VEHICLE STATISTICS ===\n")
#         valid_response_times = [v["emergency_response_time"] for v in emergency_vehicles.values() if v["emergency_response_time"] is not None]
#         if valid_response_times:
#             avg_response_time = sum(valid_response_times) / len(valid_response_times)
#             f.write(f"Average Emergency Response Time: {avg_response_time:.2f} seconds\n\n")
#
#     # Overall travel time statistics
#     f.write("=== TRAVEL TIME STATISTICS ===\n")
#     travel_times = [v["end_time"] - v["start_time"] for v in metrics.values() if v["end_time"] is not None]
#     if travel_times:
#         f.write(f"Average Travel Time: {sum(travel_times) / len(travel_times):.2f} seconds\n")
#         f.write(f"Min Travel Time: {min(travel_times):.2f} seconds\n")
#         f.write(f"Max Travel Time: {max(travel_times):.2f} seconds\n\n")
#
#     # Waiting time statistics
#     f.write("=== WAITING TIME STATISTICS ===\n")
#     avg_waiting = sum(v["waiting_time"] for v in metrics.values()) / len(metrics) if metrics else 0
#     f.write(f"Average Waiting Time: {avg_waiting:.2f} seconds\n")
#
# print(f"[INFO] Baseline summary saved to {summary_file}")
#
