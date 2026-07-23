import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import traci
from algorithms.aco import select_route, update_pheromone
from controller.reroute_logic import should_reroute
from emergency_manager import get_emergency_manager

SUMO_CMD = ["sumo-gui", "-c", "simulation.sumocfg"]
MAX_SIM_TIME = 7000

traci.start(SUMO_CMD)

emergency_mgr = get_emergency_manager()

while traci.simulation.getTime() < MAX_SIM_TIME:
    traci.simulationStep()
    sim_time = traci.simulation.getTime()
    
    # Update emergency manager to track active emergencies
    emergency_mgr.update(sim_time)

    for veh in traci.vehicle.getIDList():
        try:
            if should_reroute(veh):
                current_edge = traci.vehicle.getRoadID(veh)
                new_route = select_route(veh, current_edge)
                if new_route:
                    traci.vehicle.setRoute(veh, new_route)
                    update_pheromone(new_route)

        except traci.TraCIException:
            pass

traci.close()
