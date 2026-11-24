"""Dijkstra baseline: full search to the vehicle's real destination.

Rewritten for audit findings C2 and C3. The previous version broke the search once the
predecessor map held more than five entries, which is neither a hop limit nor a distance
limit, then truncated the reconstruction to six edges. In the normal case the destination
was never reached at all and the function returned `[]`, so the "Dijkstra baseline" was
behaviourally identical to no rerouting while still being counted as a reroute.

Contract section 5 requires Dijkstra and A* to be full-destination, complete
implementations - a weak baseline that flatters E3 is not an acceptable comparison point.
"""

import heapq

import traci

from algorithms.route_utils import (
    record_route_failure,
    blocked_edges,
    build_route_from_path,
    get_outgoing_edges,
    passenger_adjacency,
    remaining_route,
    vehicle_is_closure_exempt,
)

INFINITY = float("inf")

# Edge travel times change every simulation step, but within one step every vehicle sees
# the same network. Caching per step turns one TraCI round trip per node expansion per
# vehicle into one per edge per step.
_travel_time_cache: dict = {}
_travel_time_cache_step = None


def edge_travel_time(edge_id: str) -> float:
    """Current travel time of an edge in seconds, cached for the current step."""
    global _travel_time_cache_step

    try:
        now = traci.simulation.getTime()
    except traci.TraCIException:
        now = _travel_time_cache_step

    if now != _travel_time_cache_step:
        _travel_time_cache.clear()
        _travel_time_cache_step = now

    if edge_id not in _travel_time_cache:
        try:
            value = float(traci.edge.getTraveltime(edge_id))
        except traci.TraCIException:
            value = INFINITY
        if value < 0 or value != value:  # negative or NaN
            value = INFINITY
        _travel_time_cache[edge_id] = value
    return _travel_time_cache[edge_id]


def successors(edge_id: str, closed_edges=frozenset()):
    """Successors from the static passenger graph, falling back to live lane links.

    `closed` holds edges an incident has closed. They are excluded for ordinary traffic:
    lane closures change permissions but not connections, so without this the search routes
    straight through a closed road and SUMO rejects the replacement with only a warning.
    """
    adjacency = passenger_adjacency()
    found = adjacency.get(edge_id)
    if found is None:
        found = get_outgoing_edges(edge_id, exclude_blocked=False)
    return [edge for edge in found if edge not in closed_edges] if closed_edges else found


def dijkstra(start_edge: str, target_edge: str, closed_edges=frozenset()):
    """Shortest travel-time path from `start_edge` to `target_edge`, inclusive.

    Returns `[]` when the destination is unreachable. There is no hop limit and no
    truncation: the search settles nodes until the destination is settled.
    """
    if not start_edge or not target_edge:
        return []
    if start_edge == target_edge:
        return [start_edge]

    distance = {start_edge: 0.0}
    previous = {}
    visited = set()
    # The edge id is part of the heap key, so equal-cost expansions break ties by name
    # rather than by insertion accident - two runs settle nodes in the same order (C13).
    queue = [(0.0, start_edge)]

    while queue:
        current_distance, current_edge = heapq.heappop(queue)
        if current_edge in visited:
            continue
        visited.add(current_edge)
        if current_edge == target_edge:
            break

        for next_edge in successors(current_edge, closed_edges):
            if next_edge in visited:
                continue
            weight = edge_travel_time(next_edge)
            if weight == INFINITY:
                continue
            candidate = current_distance + weight
            if candidate < distance.get(next_edge, INFINITY):
                distance[next_edge] = candidate
                previous[next_edge] = current_edge
                heapq.heappush(queue, (candidate, next_edge))

    if target_edge not in previous:
        return []

    path = [target_edge]
    edge = target_edge
    while edge in previous:
        edge = previous[edge]
        path.append(edge)
    path.reverse()
    return path


def select_route(veh_id, current_edge):
    """Shared interface: return the vehicle's new remaining route, starting at its edge.

    The old module never defined this, which is why it was the only algorithm whose
    caller had to import the raw search function (audit C2).
    """
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
    path = dijkstra(current_edge, destination_edge, closed_edges)
    if not path:
        record_route_failure("search_found_no_path_to_destination")
        return remaining_route(veh_id)

    route = build_route_from_path(current_edge, path, destination_edge)
    return route or remaining_route(veh_id)
