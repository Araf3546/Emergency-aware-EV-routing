import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import traci
import csv
import argparse
import random

from controller.reroute_logic import should_reroute, is_emergency_vehicle, get_vehicle_type
from algorithms.aco import select_route as aco_select, update_pheromone
from algorithms.dijkstra import dijkstra
from algorithms.astar import select_route as astar_select
from algorithms.aco_pso import select_route as aco_pso_select, update_pheromone as aco_pso_update
from emergency_manager import get_emergency_manager

SUMO_BINARY = "sumo"   # use "sumo-gui" for visualization
SUMO_CFG = "simulation.sumocfg"
MAX_SIM_TIME = 7000


def run_metrics(algo_name):
    traci.start([SUMO_BINARY, "-c", SUMO_CFG])

    metrics = {}
    emergency_mgr = get_emergency_manager()
    
    # Track rerouting
    rerouted_vehicles = set()
    vehicles_needing_reroute = set()  # vehicles that met rerouting criteria
    vehicle_initial_route = {}
    
    # Track normal vs emergency conditions
    normal_speeds = {}  # edge -> list of speeds during normal time
    emergency_speeds = {}  # edge -> list of speeds during emergency
    
    # Emergency response tracking
    emergency_responses = {}  # event_id -> first vehicle arrival time

    while traci.simulation.getTime() < MAX_SIM_TIME:
        traci.simulationStep()
        sim_time = traci.simulation.getTime()
        
        # Update emergency manager
        emergency_mgr.update(sim_time)
        active_emergencies = emergency_mgr.get_active_emergency_zones()

        # ---------------- METRICS COLLECTION ----------------
        for veh in traci.vehicle.getIDList():
            if veh not in metrics:
                veh_type = get_vehicle_type(veh)
                metrics[veh] = {
                    "start_time": sim_time,
                    "waiting_time": 0.0,
                    "distance": 0.0,
                    "end_time": None,
                    "vehicle_type": veh_type,
                    "is_emergency": is_emergency_vehicle(veh),
                    "rerouted": False,
                    "total_reroutes": 0,
                    "emergency_response_time": None,
                    "congestion_ratio": 1.0,
                    "max_speed_observed": 0.0
                }
                vehicle_initial_route[veh] = traci.vehicle.getRoute(veh)

            try:
                current_speed = traci.vehicle.getSpeed(veh)
                metrics[veh]["max_speed_observed"] = max(metrics[veh]["max_speed_observed"], current_speed)
                
                metrics[veh]["waiting_time"] += traci.vehicle.getWaitingTime(veh)
                metrics[veh]["distance"] = traci.vehicle.getDistance(veh)
                
                # Track speeds during normal vs emergency
                current_edge = traci.vehicle.getRoadID(veh)
                if current_edge and not current_edge.startswith(":"):
                    if active_emergencies:
                        if current_edge not in emergency_speeds:
                            emergency_speeds[current_edge] = []
                        emergency_speeds[current_edge].append(current_speed)
                    else:
                        if current_edge not in normal_speeds:
                            normal_speeds[current_edge] = []
                        normal_speeds[current_edge].append(current_speed)
                
            except traci.TraCIException:
                pass

        # Track vehicle arrivals
        for veh in traci.simulation.getArrivedIDList():
            if veh in metrics and metrics[veh]["end_time"] is None:
                metrics[veh]["end_time"] = sim_time

        # Emergency response tracking (first emergency vehicle to reach active emergency)
        if active_emergencies:
            for event_id, event_data in active_emergencies.items():
                if event_id not in emergency_responses:
                    for veh in traci.vehicle.getIDList():
                        if is_emergency_vehicle(veh):
                            try:
                                veh_edge = traci.vehicle.getRoadID(veh)
                                if veh_edge in event_data['blocked_edges']:
                                    emergency_responses[event_id] = sim_time
                                    emergency_mgr.register_response_vehicle(veh, event_id)
                                    if veh in metrics:
                                        # Response time = time when reached emergency zone
                                        response_time = sim_time - event_data['start_time']
                                        if metrics[veh]["emergency_response_time"] is None:
                                            metrics[veh]["emergency_response_time"] = response_time
                                    break
                            except traci.TraCIException:
                                pass

        # ---------------- ROUTING LOGIC ----------------
        for veh in traci.vehicle.getIDList():
            try:
                # Check if vehicle should be rerouted
                needs_reroute = should_reroute(veh)
                
                if needs_reroute:
                    vehicles_needing_reroute.add(veh)
                
                if not needs_reroute:
                    continue

                current_edge = traci.vehicle.getRoadID(veh)
                route = traci.vehicle.getRoute(veh)

                if len(route) < 2:
                    continue

                destination_edge = route[-1]
                
                # Track if vehicle reroutes
                if veh not in rerouted_vehicles:
                    rerouted_vehicles.add(veh)
                    if veh in metrics:
                        metrics[veh]["rerouted"] = True
                
                if veh in metrics:
                    metrics[veh]["total_reroutes"] += 1

                # ---- ACO ----
                if algo_name == "aco":
                    new_route = aco_select(veh, current_edge)
                    if new_route:
                        traci.vehicle.setRoute(veh, new_route)
                        update_pheromone(new_route)

                # ---- DIJKSTRA ----
                elif algo_name == "dijkstra":
                    new_route = dijkstra(current_edge, destination_edge)
                    if new_route:
                        traci.vehicle.setRoute(veh, new_route)

                # ---- A* ----
                elif algo_name == "astar":
                    new_route = astar_select(veh, current_edge)
                    if new_route:
                        traci.vehicle.setRoute(veh, new_route)

                # ---- ACO-PSO ----
                elif algo_name == "aco_pso":
                    new_route = aco_pso_select(veh, current_edge)
                    if new_route:
                        traci.vehicle.setRoute(veh, new_route)
                        aco_pso_update(new_route)

            except traci.TraCIException:
                pass

    traci.close()

    # Calculate congestion level
    avg_congestion = 1.0
    if normal_speeds and emergency_speeds:
        emergency_speeds_avg = sum([sum(v) / len(v) for v in emergency_speeds.values()]) / len(emergency_speeds)
        normal_speeds_avg = sum([sum(v) / len(v) for v in normal_speeds.values()]) / len(normal_speeds)
        
        if normal_speeds_avg > 0:
            avg_congestion = emergency_speeds_avg / normal_speeds_avg
            avg_congestion = max(0.0, min(1.0, avg_congestion))  # Clamp between 0-1

    # Calculate rerouting efficiency
    rerouting_coverage = (len(rerouted_vehicles) / len(metrics) * 100) if metrics else 0
    rerouting_efficiency = (len(rerouted_vehicles) / len(vehicles_needing_reroute) * 100) if vehicles_needing_reroute else 0

    # ============ SAVE RESULTS ================
    os.makedirs("results", exist_ok=True)
    output_file = f"results/{algo_name}_results.csv"

    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "VehicleID", 
            "VehicleType",
            "TravelTime", 
            "WaitingTime", 
            "Distance",
            "IsEmergency",
            "Rerouted",
            "TotalReroutes",
            "EmergencyResponseTime",
            "CongestionLevel"
        ])

        for veh, data in metrics.items():
            if data["end_time"] is not None:
                travel_time = data["end_time"] - data["start_time"]
                emergency_response_time = data["emergency_response_time"] if data["emergency_response_time"] is not None else "N/A"
                
                writer.writerow([
                    veh,
                    data["vehicle_type"],
                    round(travel_time, 2),
                    round(data["waiting_time"], 2),
                    round(data["distance"], 2),
                    "Yes" if data["is_emergency"] else "No",
                    "Yes" if data["rerouted"] else "No",
                    data["total_reroutes"],
                    round(emergency_response_time, 2) if isinstance(emergency_response_time, float) else emergency_response_time,
                    round(avg_congestion, 4)
                ])

    # ============ SAVE SUMMARY STATISTICS ================
    summary_file = f"results/{algo_name}_summary.txt"
    with open(summary_file, "w") as f:
        f.write(f"=== SIMULATION SUMMARY: {algo_name.upper()} ===\n\n")
        f.write(f"Algorithm: {algo_name}\n")
        f.write(f"Simulation Time: {MAX_SIM_TIME} seconds\n")
        f.write(f"Total Vehicles: {len(metrics)}\n")
        f.write(f"Emergency Vehicles: {sum(1 for v in metrics.values() if v['is_emergency'])}\n")
        f.write(f"Passenger Vehicles: {sum(1 for v in metrics.values() if not v['is_emergency'])}\n\n")
        
        f.write("=== REROUTING STATISTICS ===\n")
        f.write(f"Total Vehicles: {len(metrics)}\n")
        f.write(f"Vehicles Needing Reroute: {len(vehicles_needing_reroute)}\n")
        f.write(f"Vehicles Rerouted: {len(rerouted_vehicles)}\n\n")
        f.write(f"Rerouting Coverage: {rerouting_coverage:.2f}% (rerouted / total vehicles)\n")
        f.write(f"Rerouting Efficiency: {rerouting_efficiency:.2f}% (rerouted / vehicles needing reroute)\n\n")
        f.write(f"Average Reroutes per Vehicle: {sum(v['total_reroutes'] for v in metrics.values()) / len(metrics):.2f}\n\n")
        
        f.write("=== CONGESTION ANALYSIS ===\n")
        f.write(f"Congestion Ratio (Emergency/Normal): {avg_congestion:.4f}\n")
        f.write(f"Speed Reduction During Emergency: {(1 - avg_congestion) * 100:.2f}%\n\n")
        
        # Emergency vehicle statistics
        emergency_vehicles = {v: d for v, d in metrics.items() if d["is_emergency"]}
        if emergency_vehicles:
            f.write("=== EMERGENCY VEHICLE STATISTICS ===\n")
            valid_response_times = [v["emergency_response_time"] for v in emergency_vehicles.values() if v["emergency_response_time"] is not None]
            if valid_response_times:
                avg_response_time = sum(valid_response_times) / len(valid_response_times)
                f.write(f"Average Emergency Response Time: {avg_response_time:.2f} seconds\n")
            
            emergency_rerouted = sum(1 for v in emergency_vehicles.values() if v["rerouted"])
            f.write(f"Emergency Vehicles Rerouted: {emergency_rerouted}/{len(emergency_vehicles)} ({emergency_rerouted/len(emergency_vehicles)*100:.2f}%)\n\n")
        
        # Overall travel time statistics
        f.write("=== TRAVEL TIME STATISTICS ===\n")
        travel_times = [v["end_time"] - v["start_time"] for v in metrics.values() if v["end_time"] is not None]
        if travel_times:
            f.write(f"Average Travel Time: {sum(travel_times) / len(travel_times):.2f} seconds\n")
            f.write(f"Min Travel Time: {min(travel_times):.2f} seconds\n")
            f.write(f"Max Travel Time: {max(travel_times):.2f} seconds\n\n")
        
        # Waiting time statistics
        f.write("=== WAITING TIME STATISTICS ===\n")
        avg_waiting = sum(v["waiting_time"] for v in metrics.values()) / len(metrics) if metrics else 0
        f.write(f"Average Waiting Time: {avg_waiting:.2f} seconds\n\n")

    print(f"[INFO] Results saved to {output_file}")
    print(f"[INFO] Summary saved to {summary_file}")
    print(f"[INFO] Total vehicles completed: {sum(1 for v in metrics.values() if v['end_time'] is not None)}")
    print(f"[INFO] Rerouting Coverage: {rerouting_coverage:.2f}% (of total vehicles)")
    print(f"[INFO] Rerouting Efficiency: {rerouting_efficiency:.2f}% (of vehicles needing reroute)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--algo",
        required=True,
        choices=["baseline", "aco", "dijkstra", "astar", "aco_pso"],
        help="Routing algorithm to evaluate"
    )
    args = parser.parse_args()

    run_metrics(args.algo)

