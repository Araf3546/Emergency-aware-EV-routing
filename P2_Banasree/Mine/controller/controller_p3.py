#!/usr/bin/env python3
"""Dedicated P3/E3 Hybrid controller entrypoint for SUMO runs."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import traci

from emergency_manager import get_emergency_manager
from p3_runtime import P3RuntimeCoordinator
from controller.reroute_logic import should_reroute


class P3Controller:
    """Run the protocol-driven E3 Hybrid routing loop in SUMO."""

    def __init__(self, network_file: str = "network.net.xml", routes_file: str = "routes_realworld.rou.xml"):
        self.network_file = network_file
        self.routes_file = routes_file
        self.sumo_cmd = ["sumo", "-c", "simulation.sumocfg", "--quit-on-end", "--no-warnings"]
        self.max_sim_time = 14400
        self.started = False
        self.coordinator = P3RuntimeCoordinator()

    def run_simulation(self) -> bool:
        try:
            print("[P3] Starting SUMO simulation with E3 Hybrid runtime...")
            traci.start(self.sumo_cmd)
            self.started = True

            emergency_mgr = get_emergency_manager()
            step_count = 0

            while traci.simulation.getTime() < self.max_sim_time:
                traci.simulationStep()
                sim_time = traci.simulation.getTime()
                step_count += 1

                try:
                    emergency_mgr.update(sim_time)
                    self.coordinator.update_step(sim_time)
                except Exception as e:
                    print(f"[P3] Error in update_step at time {sim_time}s: {type(e).__name__}: {e}")
                    raise

                if step_count % 1000 == 0:
                    print(f"[P3] Simulation time: {sim_time:.1f}s")

            print(f"[P3] Simulation completed at {traci.simulation.getTime()}s")
            traci.close()
            self.started = False
            return True
        except Exception as exc:
            print(f"[P3] Error during simulation: {exc}")
            if self.started:
                try:
                    traci.close()
                except Exception:
                    pass
            return False

    def save_results(self) -> bool:
        try:
            self.coordinator.save_metrics(run_name="p3_hybrid")
            print("[P3] Protocol metrics saved to results/p3_hybrid_protocol_metrics.csv")
            return True
        except Exception as exc:
            print(f"[P3] Error saving results: {exc}")
            return False


def main() -> None:
    controller = P3Controller()
    success = controller.run_simulation()
    if success:
        controller.save_results()
        print("\n✓ P3 controller run completed successfully")
    else:
        print("\n✗ P3 controller run failed")


if __name__ == "__main__":
    main()
