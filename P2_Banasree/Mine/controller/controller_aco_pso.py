#!/usr/bin/env python3
"""
ACO_PSO (Hybrid) Algorithm Controller

Implements class-based interface for ACO_PSO hybrid algorithm with SUMO simulation
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import traci
from p3_runtime import P3RuntimeCoordinator
from emergency_manager import get_emergency_manager


class ACOPSOController:
    """ACO_PSO Hybrid Algorithm Controller for SUMO Simulation"""
    
    def __init__(self, network_file: str, routes_file: str):
        """Initialize ACO_PSO controller"""
        self.network_file = network_file
        self.routes_file = routes_file
        self.sumo_cmd = ["sumo", "-c", "simulation.sumocfg", "--quit-on-end", "--no-warnings"]
        self.max_sim_time = 86400  # 24 hours
        self.results = []
        self.started = False
        self.coordinator = P3RuntimeCoordinator()
        
    def run_simulation(self) -> bool:
        """Run ACO_PSO simulation"""
        try:
            print("\n[ACO_PSO] Starting SUMO simulation with TraCI...")
            traci.start(self.sumo_cmd)
            self.started = True
            
            emergency_mgr = get_emergency_manager()
            step_count = 0
            
            while traci.simulation.getTime() < self.max_sim_time:
                traci.simulationStep()
                sim_time = traci.simulation.getTime()
                step_count += 1
                
                # Update emergency manager and the protocol-aware routing coordinator
                emergency_mgr.update(sim_time)
                self.coordinator.update_step(sim_time)
                
                # Progress indicator
                if step_count % 10000 == 0:
                    print(f"[ACO_PSO] Simulation time: {sim_time:.1f}s")
            
            print(f"[ACO_PSO] Simulation completed at {traci.simulation.getTime()}s")
            traci.close()
            self.started = False
            return True
        
        except Exception as e:
            print(f"[ACO_PSO] Error during simulation: {e}")
            if self.started:
                try:
                    traci.close()
                except:
                    pass
            return False
    
    def save_results(self) -> bool:
        """Save simulation results and P3 runtime metrics."""
        try:
            print("[ACO_PSO] Saving P3 hybrid protocol metrics")
            self.coordinator.save_metrics(run_name="p3_hybrid")
            print("[ACO_PSO] Results will be extracted from tripinfo.xml")
            return True
        except Exception as e:
            print(f"[ACO_PSO] Error saving results: {e}")
            return False


def main():
    """Main entry point for standalone execution"""
    
    network_file = 'network.net.xml'
    routes_file = 'routes_realworld.rou.xml'
    
    print("\n" + "="*70)
    print("ACO_PSO (Hybrid Algorithm) - SUMO Simulation")
    print("="*70)
    
    controller = ACOPSOController(network_file, routes_file)
    
    # Run simulation
    success = controller.run_simulation()
    
    if success:
        # Save results
        controller.save_results()
        print("\n✓ ACO_PSO simulation completed successfully")
    else:
        print("\n✗ ACO_PSO simulation failed")
    
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
