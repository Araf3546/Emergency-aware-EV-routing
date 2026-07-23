#!/usr/bin/env python3
"""
ACO (Ant Colony Optimization) Algorithm Controller

Implements class-based interface for ACO algorithm with SUMO simulation
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import traci
import csv
from datetime import datetime
from pathlib import Path
from algorithms.aco import select_route, update_pheromone
from controller.reroute_logic import should_reroute
from emergency_manager import get_emergency_manager


class ACOController:
    """ACO Algorithm Controller for SUMO Simulation"""
    
    def __init__(self, network_file: str, routes_file: str):
        """Initialize ACO controller"""
        self.network_file = network_file
        self.routes_file = routes_file
        self.sumo_cmd = ["sumo", "-c", "simulation.sumocfg", "--quit-on-end", "--no-warnings"]
        self.max_sim_time = 86400  # 24 hours
        self.results = []
        self.vehicle_stats = {}
        self.started = False
        
    def run_simulation(self) -> bool:
        """Run ACO simulation"""
        try:
            print("\n[ACO] Starting SUMO simulation with TraCI...")
            traci.start(self.sumo_cmd)
            self.started = True
            
            emergency_mgr = get_emergency_manager()
            step_count = 0
            
            while traci.simulation.getTime() < self.max_sim_time:
                traci.simulationStep()
                sim_time = traci.simulation.getTime()
                step_count += 1
                
                # Update emergency manager
                emergency_mgr.update(sim_time)
                
                # Process each vehicle
                for veh in traci.vehicle.getIDList():
                    try:
                        if not should_reroute(veh):
                            continue
                        
                        current_edge = traci.vehicle.getRoadID(veh)
                        
                        # Skip junction edges
                        if current_edge.startswith(":"):
                            continue
                        
                        # Get current route
                        route = traci.vehicle.getRoute(veh)
                        if len(route) < 2:
                            continue
                        
                        # Get new route using ACO
                        new_route = select_route(veh, current_edge)
                        if new_route:
                            traci.vehicle.setRoute(veh, new_route)
                            update_pheromone(new_route)
                    
                    except traci.TraCIException as e:
                        pass
                
                # Progress indicator
                if step_count % 10000 == 0:
                    print(f"[ACO] Simulation time: {sim_time:.1f}s")
            
            print(f"[ACO] Simulation completed at {traci.simulation.getTime()}s")
            traci.close()
            self.started = False
            return True
        
        except Exception as e:
            print(f"[ACO] Error during simulation: {e}")
            if self.started:
                try:
                    traci.close()
                except:
                    pass
            return False
    
    def save_results(self) -> bool:
        """Save simulation results"""
        try:
            # Results are extracted from tripinfo.xml by extract_results.py
            print("[ACO] Results will be extracted from tripinfo.xml")
            return True
        except Exception as e:
            print(f"[ACO] Error saving results: {e}")
            return False


def main():
    """Main entry point for standalone execution"""
    
    network_file = 'network.net.xml'
    routes_file = 'routes_realworld.rou.xml'
    
    print("\n" + "="*70)
    print("ACO (Ant Colony Optimization) Algorithm - SUMO Simulation")
    print("="*70)
    
    controller = ACOController(network_file, routes_file)
    
    # Run simulation
    success = controller.run_simulation()
    
    if success:
        # Save results
        controller.save_results()
        print("\n✓ ACO simulation completed successfully")
    else:
        print("\n✗ ACO simulation failed")
    
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
