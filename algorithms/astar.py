"""A* baseline: full search to the vehicle's real destination, with an admissible heuristic.

Rewritten for audit findings C4 to C7.

* C4 - there was no closed set, so nodes were re-expanded repeatedly.
* C5 - the more damaging defect. `g` accumulated seconds from `getTraveltime` while `h`
  returned metres from the lane geometry, and the two were summed. In an urban network
  metres exceed seconds by about an order of magnitude, so `h` dominated and A* degenerated
  into greedy best-first search towards the goal's compass direction. It chose the wrong
  routes, not merely slowly. The heuristic is now divided by the network's maximum allowed
  speed, which converts it to seconds and makes it a lower bound on the true remaining
  travel time, so the search is admissible and consistent.
* C6 - internal junction edges are filtered (defensive: measured on this network, lane
  links never return one).
* C7 - lane geometry is cached instead of being fetched twice per node expansion.
"""

import heapq
import math

import traci

from algorithms.route_utils import (
    record_route_failure,
    blocked_edges,
    build_route_from_path,
    lane_end_point,
    passenger_edge_pool,
    remaining_route,
    vehicle_is_closure_exempt,
)
from algorithms.dijkstra import edge_travel_time, successors

INFINITY = float("inf")

# Fallback used only if the network speed cannot be measured; 27.8 m/s is the fastest
# posted speed in the Porto study area.
FALLBACK_MAX_SPEED = 27.8

_network_max_speed = None


def network_max_speed() -> float:
    """Highest allowed speed in the routable network, in m/s, measured once per run.

    Dividing the straight-line distance by this speed can never overestimate the true
    remaining travel time, which is what makes the heuristic admissible.
    """
    global _network_max_speed
    if _network_max_speed is not None:
        return _network_max_speed

    fastest = 0.0
    for edge_id in sorted(passenger_edge_pool()):
        try:
            speed = float(traci.lane.getMaxSpeed(f"{edge_id}_0"))
        except traci.TraCIException:
            continue
        if speed > fastest:
            fastest = speed
    _network_max_speed = fastest if fastest > 0 else FALLBACK_MAX_SPEED
    return _network_max_speed


def reset_network_max_speed() -> None:
    """Forget the measured speed; used by tests between networks."""
    global _network_max_speed
    _network_max_speed = None


def heuristic(edge1: str, edge2: str) -> float:
    """Admissible estimate, in seconds, of the time from the end of `edge1` to `edge2`."""
    first = lane_end_point(edge1)
    second = lane_end_point(edge2)
    if first is None or second is None:
        return 0.0
    metres = math.hypot(second[0] - first[0], second[1] - first[1])
    return metres / network_max_speed()


def astar_route(start_edge: str, goal_edge: str, closed_edges=frozenset()):
    """Least-travel-time path from `start_edge` to `goal_edge`, inclusive, or `[]`.

    `closed_edges` are edges an incident has closed to this vehicle; they are not entered.
    """
    if not start_edge or not goal_edge:
        return []
    if start_edge == goal_edge:
        return [start_edge]

    came_from = {}
    g_score = {start_edge: 0.0}
    # C4: the closed set the previous version was missing, kept separate from the incident
    # closures above so the two cannot be confused.
    expanded = set()
    # The edge id is part of the heap key so ties break by name, not by insertion accident.
    open_set = [(heuristic(start_edge, goal_edge), start_edge)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current in expanded:
            continue
        expanded.add(current)

        if current == goal_edge:
            return reconstruct_path(came_from, current)

        for neighbor in successors(current, closed_edges):
            if neighbor in expanded:
                continue
            travel_time = edge_travel_time(neighbor)
            if travel_time == INFINITY:
                continue
            tentative_g = g_score[current] + travel_time
            if tentative_g < g_score.get(neighbor, INFINITY):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heapq.heappush(
                    open_set,
                    (tentative_g + heuristic(neighbor, goal_edge), neighbor),
                )

    return []


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def select_route(veh_id, current_edge):
    """Shared interface: return the vehicle's new remaining route, starting at its edge."""
    try:
        full_route = list(traci.vehicle.getRoute(veh_id))
    except traci.TraCIException:
        return []
    if not full_route:
        return []

    destination_edge = full_route[-1]
    if current_edge == destination_edge:
        return remaining_route(veh_id)

    closed_edges = frozenset() if vehicle_is_closure_exempt(veh_id) else blocked_edges()
    path = astar_route(current_edge, destination_edge, closed_edges)
    if not path:
        record_route_failure("search_found_no_path_to_destination")
        return remaining_route(veh_id)

    route = build_route_from_path(current_edge, path, destination_edge)
    return route or remaining_route(veh_id)
