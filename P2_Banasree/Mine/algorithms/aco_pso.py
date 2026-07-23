import traci
import random

# -------------------------
# ACO PARAMETERS
# -------------------------
ALPHA = 1.0
BETA = 2.0
RHO = 0.1
Q = 1.0

# -------------------------
# PSO PARAMETERS
# -------------------------
W = 0.5       # inertia weight
C1 = 1.5      # cognitive (personal best) coefficient
C2 = 1.5      # social (global best) coefficient
VMAX = 0.5    # maximum velocity

# -------------------------
# STATE
# -------------------------
pheromone = {}
velocity = {}
personal_best = {}
global_best_score = 0.0


# -------------------------
# Initialize pheromone
# -------------------------
def initialize_pheromone():
    global pheromone
    for edge in traci.edge.getIDList():
        pheromone[edge] = 1.0


# -------------------------
# Get outgoing edges
# -------------------------
def get_outgoing_edges(edge_id):
    outgoing = set()
    try:
        lane_id = f"{edge_id}_0"
        for link in traci.lane.getLinks(lane_id):
            next_lane = link[0]
            next_edge = next_lane.split("_")[0]
            if not next_edge.startswith(":"):
                outgoing.add(next_edge)
    except traci.TraCIException:
        pass
    return list(outgoing)


# -------------------------
# Heuristic (low congestion)
# -------------------------
def heuristic(edge):
    veh = traci.edge.getLastStepVehicleNumber(edge)
    return 1.0 / (1.0 + veh)


# -------------------------
# ACO + PSO decision
# -------------------------
def select_route(veh_id, current_edge):
    global global_best_score, personal_best
    
    if not pheromone:
        initialize_pheromone()

    outgoing = get_outgoing_edges(current_edge)

    if not outgoing:
        return list(traci.vehicle.getRoute(veh_id))

    probabilities = []
    edges = []

    for edge in outgoing:
        # ACO component
        tau = pheromone.get(edge, 1.0) ** ALPHA
        eta = heuristic(edge) ** BETA
        
        # Initialize velocity and personal best if not present
        if edge not in velocity:
            velocity[edge] = 0.0
        if edge not in personal_best:
            personal_best[edge] = 1.0  # Initialize to baseline pheromone

        # PSO velocity update - CORRECTED: Higher pheromone should increase velocity
        current_pheromone = pheromone.get(edge, 1.0)
        pb = personal_best.get(edge, 1.0)
        
        # Velocity increases when current pheromone is better than personal/global best
        velocity[edge] = (
            W * velocity[edge]
            + C1 * random.random() * (current_pheromone - pb)
            + C2 * random.random() * (current_pheromone - global_best_score)
        )
        
        # Clamp velocity
        velocity[edge] = max(-VMAX, min(VMAX, velocity[edge]))

        # Combined score: ACO component + PSO velocity boost
        # PSO adds exploration/exploitation balance via velocity
        score = tau * eta * (1.0 + velocity[edge])
        probabilities.append(max(score, 0.0001))
        edges.append(edge)

    # Normalize probabilities
    total = sum(probabilities)
    probabilities = [p / total for p in probabilities]

    # Select edge based on combined score
    selected_edge = random.choices(edges, probabilities)[0]
    selected_pheromone = pheromone.get(selected_edge, 1.0)

    # Update personal best for this edge if current pheromone is better
    if selected_pheromone > personal_best.get(selected_edge, 1.0):
        personal_best[selected_edge] = selected_pheromone

    # Update global best score if any edge's pheromone exceeds global best
    if selected_pheromone > global_best_score:
        global_best_score = selected_pheromone

    # Build new route
    current_route = list(traci.vehicle.getRoute(veh_id))
    if current_edge in current_route:
        idx = current_route.index(current_edge)
        new_route = current_route[:idx + 1] + [selected_edge]
    else:
        new_route = current_route

    return new_route


# -------------------------
# Pheromone update
# -------------------------
def update_pheromone(route):
    for e in pheromone:
        pheromone[e] *= (1 - RHO)
    
    for edge in route:
        pheromone[edge] += Q
