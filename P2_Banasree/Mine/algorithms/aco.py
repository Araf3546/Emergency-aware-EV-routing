import random
import traci

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


def get_outgoing_edges(edge_id):
    """
    Get outgoing edges using lane-based connectivity (TraCI correct)
    """
    outgoing_edges = set()

    try:
        lane_id = f"{edge_id}_0"
        links = traci.lane.getLinks(lane_id)

        for link in links:
            next_lane = link[0]   # connected lane
            # Extract edge ID from lane ID (format: "edge_laneIndex")
            next_edge = next_lane.split("_")[0]
            if not next_edge.startswith(":"):
                outgoing_edges.add(next_edge)

    except traci.TraCIException:
        pass

    return list(outgoing_edges)


def select_route(veh_id, current_edge):
    if not pheromone:
        initialize_pheromone()

    outgoing_edges = get_outgoing_edges(current_edge)

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

    current_route = list(traci.vehicle.getRoute(veh_id))
    if current_edge in current_route:
        idx = current_route.index(current_edge)
        new_route = current_route[:idx + 1] + [selected_edge]
    else:
        new_route = current_route

    return new_route


def update_pheromone(route):
    for edge in pheromone:
        pheromone[edge] *= (1 - RHO)

    for edge in route:
        pheromone[edge] += Q
