import traci
import heapq
import math


# -------------------------------------------------
# Heuristic: Euclidean distance using lane geometry
# -------------------------------------------------
def heuristic(edge1, edge2):
    try:
        lane1 = f"{edge1}_0"
        lane2 = f"{edge2}_0"

        x1, y1 = traci.lane.getShape(lane1)[-1]
        x2, y2 = traci.lane.getShape(lane2)[-1]

        return math.hypot(x2 - x1, y2 - y1)

    except traci.TraCIException:
        return 0.0


# -------------------------------------------------
# Outgoing edges (lane-link based, SUMO safe)
# -------------------------------------------------
def get_outgoing_edges(edge_id):
    outgoing = set()

    try:
        lane_id = f"{edge_id}_0"
        links = traci.lane.getLinks(lane_id)

        for link in links:
            next_lane = link[0]      # e.g. edgeX_0
            next_edge = next_lane.split("_")[0]
            outgoing.add(next_edge)

    except traci.TraCIException:
        pass

    return list(outgoing)


# -------------------------------------------------
# A* core algorithm
# -------------------------------------------------
def astar_route(start_edge, goal_edge):
    open_set = []
    heapq.heappush(open_set, (0, start_edge))

    came_from = {}
    g_score = {start_edge: 0.0}
    f_score = {start_edge: heuristic(start_edge, goal_edge)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_edge:
            return reconstruct_path(came_from, current)

        for neighbor in get_outgoing_edges(current):
            try:
                travel_time = traci.edge.getTraveltime(neighbor)
            except traci.TraCIException:
                continue

            tentative_g = g_score[current] + travel_time

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal_edge)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []


# -------------------------------------------------
# Path reconstruction
# -------------------------------------------------
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.insert(0, current)
    return path


# -------------------------------------------------
# Controller-facing wrapper
# -------------------------------------------------
def select_route(veh_id, current_edge):
    try:
        destination = traci.vehicle.getRoute(veh_id)[-1]
        path = astar_route(current_edge, destination)

        if path:
            return path

    except traci.TraCIException:
        pass

    return traci.vehicle.getRoute(veh_id)
