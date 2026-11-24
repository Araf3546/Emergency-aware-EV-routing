import random

import traci

from algorithms.route_utils import build_destination_valid_route, get_outgoing_edges

ALPHA = 1.0
BETA = 2.0
RHO = 0.1
Q = 1.0

pheromone = {}


def initialize_pheromone():
    global pheromone
    for edge in traci.edge.getIDList():
        pheromone[edge] = 1.0


def heuristic(edge_id):
    veh_count = traci.edge.getLastStepVehicleNumber(edge_id)
    return 1.0 / (1.0 + veh_count)


# Successor enumeration lives in algorithms/route_utils.py so that every algorithm sees
# the same candidate set: all lanes (C14), passenger-accessible only (C11), sorted (C13).


def select_route(veh_id, current_edge):
    if not pheromone:
        initialize_pheromone()

    outgoing_edges = get_outgoing_edges(current_edge, veh_id=veh_id)

    if not outgoing_edges:
        return list(traci.vehicle.getRoute(veh_id))

    probabilities = []
    edges = []

    for eid in outgoing_edges:
        tau = pheromone.get(eid, 1.0) ** ALPHA
        eta = heuristic(eid) ** BETA
        probabilities.append(tau * eta)
        edges.append(eid)

    total = sum(probabilities)
    probabilities = [p / total for p in probabilities]

    selected_edge = random.choices(edges, probabilities)[0]

    # Audit C1: this used to return current_route[:idx + 1] + [selected_edge], which
    # silently moved the vehicle's destination to the end of the selected edge. The route
    # must keep its original destination (contract section 5).
    current_route = list(traci.vehicle.getRoute(veh_id))
    return build_destination_valid_route(veh_id, current_route, current_edge, selected_edge)


def update_pheromone(route):
    for edge in pheromone:
        pheromone[edge] *= (1 - RHO)

    for edge in route:
        pheromone[edge] += Q
