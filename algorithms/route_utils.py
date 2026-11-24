"""Shared route construction, candidate filtering and route-validity accounting.

Every algorithm that picks a next edge goes through this module, so that all of them
obey the same rules:

* **Destination validity** (audit C1, contract section 5). A reroute may change *how* a
  vehicle reaches its destination, never *what* its destination is. `build_destination_valid_route`
  splices a real path from the chosen edge to the original destination; `set_route_checked`
  refuses to apply anything whose last edge differs from the original last edge.
* **Vehicle-class filtering** (audit C11). Only 5,659 of the network's 21,385 non-internal
  edges are passenger-accessible and mutually reachable. The rest are footways, steps and
  pedestrian areas, and selecting one makes the reroute silently fail.
* **Determinism** (audit C13). Candidate lists are `sorted`, never `list(set(...))`, so the
  order does not depend on Python's per-process string hash seed.
* **Complete successor sets** (audit C14). Links are unioned over every lane of an edge,
  not read from lane 0 alone.

The counters in `ROUTE_STATS` exist because contract section 5 requires route failures to
be reported rather than silently ignored.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set

import traci

# ===============================
# PATHS AND CONSTANTS
# ===============================

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_NETWORK_PATH = REPO_ROOT / "network" / "porto.net.xml"
NETWORK_VALIDATOR_PATH = REPO_ROOT / "scripts" / "03_validate_network.py"
DEFAULT_SCENARIO_CONFIG = REPO_ROOT / "config" / "scenario.yaml"

# Raising on a destination violation is the default. A silent no-op is exactly how the
# truncated-route bug (C1) survived for so long. Set P3_ROUTE_ASSERT=0 to downgrade it to
# a counted rejection.
ROUTE_ASSERT_ENABLED = os.environ.get("P3_ROUTE_ASSERT", "1") != "0"


class RouteValidityError(AssertionError):
    """Raised when a proposed route would change the vehicle's destination."""


# ===============================
# ROUTE-VALIDITY ACCOUNTING
# ===============================

_STAT_KEYS = (
    "reroute_proposals",              # select_route returned a route different from the current one
    "reroutes_applied",               # setRoute actually succeeded
    "setroute_errors",                # SUMO rejected the route (TraCIException)
    "setroute_silent_rejections",     # SUMO logged a warning and kept the old route
    "routes_crossing_closed_edge",    # refused: the route ran through a road an incident closed
    "destination_violations",         # proposed route ended somewhere else - must stay 0
    "route_build_failures",           # findRoute could not connect chosen edge to destination
    # The three topology counters are counted once per distinct edge, when that edge's
    # successors are first enumerated - lane links do not change during a run.
    "topology_successors_seen",       # successor edges found before filtering
    "topology_rejected_internal",     # dropped because they were junction-internal edges
    "topology_rejected_class",        # dropped because they are not passenger-accessible
    "no_candidate_edges",             # a decision with an empty candidate set after filtering
    # The clean design's reachability filter (2026-09-15), active only while a closure is on:
    "closure_filtered_decisions",     # decisions where at least one candidate could not reach the destination
    "closure_candidates_removed",     # candidates removed for that reason
    "closure_filter_fallbacks",       # decisions where no candidate could reach it: the list was kept
)

# Filled in once when the passenger graph is built. This is the C11 evidence: how much of
# the network is not usable by a passenger car, and therefore how many successor links the
# unfiltered algorithms used to offer as candidates.
GRAPH_STATS: Dict[str, int] = {}

ROUTE_STATS: Dict[str, int] = {key: 0 for key in _STAT_KEYS}

# Why route building failed, so the failures can be classified instead of just counted.
# Each key is "<cause>|closure_active" or "<cause>|no_closure", which separates failures
# caused by an incident closure from failures present in normal traffic.
ROUTE_FAILURE_REASONS: Dict[str, int] = {}


def record_route_failure(reason: str) -> None:
    """Count one route-building failure under a named cause."""
    ROUTE_STATS["route_build_failures"] += 1
    context = "closure_active" if _blocked_edges else "no_closure"
    key = f"{reason}|{context}"
    ROUTE_FAILURE_REASONS[key] = ROUTE_FAILURE_REASONS.get(key, 0) + 1


def reset_route_stats() -> None:
    """Zero every counter, and forget the per-run reachability answers. Call once per run, before the
    SUMO loop."""
    for key in _STAT_KEYS:
        ROUTE_STATS[key] = 0
    ROUTE_FAILURE_REASONS.clear()
    _reachability_cache.clear()


def get_route_stats() -> Dict[str, int]:
    """Return a copy of the counters, plus the derived rates used in reports."""
    stats = dict(ROUTE_STATS)
    stats.update(GRAPH_STATS)
    stats["route_build_failure_reasons"] = dict(sorted(ROUTE_FAILURE_REASONS.items()))
    proposals = stats["reroute_proposals"]
    stats["reroute_application_rate"] = (
        round(stats["reroutes_applied"] / proposals, 6) if proposals else 0.0
    )
    seen = stats["topology_successors_seen"]
    stats["topology_class_rejection_rate"] = (
        round(stats["topology_rejected_class"] / seen, 6) if seen else 0.0
    )
    return stats


def format_route_stats() -> str:
    """One-line-per-counter text block for run summaries."""
    stats = get_route_stats()
    return "\n".join(f"{key}: {value}" for key, value in stats.items())


# ===============================
# VEHICLE-CLASS CANDIDATE POOL (C11)
# ===============================

_passenger_pool: Optional[frozenset] = None
_passenger_accessible: Optional[frozenset] = None
_passenger_adjacency: Optional[Dict[str, List[str]]] = None


def _load_network_validator():
    """Import scripts/03_validate_network.py, whose module name cannot be imported directly."""
    specification = importlib.util.spec_from_file_location(
        "p3_network_validator", NETWORK_VALIDATOR_PATH
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f"Cannot load network validator: {NETWORK_VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def build_passenger_graph(network_path: Optional[Path] = None) -> None:
    """Build the passenger candidate pool and its adjacency once, from the network file.

    The pool is the same definition `scripts/06_map_od_to_network.py` uses to pick OD
    endpoints: passenger-accessible edges, restricted to the largest strongly connected
    component, keeping only edges with both a predecessor and a successor. Using the same
    pool means a rerouting vehicle can always be routed onward to any destination the
    demand generator was allowed to choose.

    The adjacency is taken from the network's own `<connection>` elements rather than from
    TraCI lane links. Checked against SUMO on 300 sampled pool edges: the two agree
    exactly, and the network file needs no TraCI round trips, so Dijkstra and A* can search
    a graph that is built once at startup (audit C2) instead of per node expansion.

    Costs about 0.7 s on the Porto network, so it is computed once per process rather than
    cached to disk (a stale cache would be worse than the parse).
    """
    global _passenger_pool, _passenger_accessible, _passenger_adjacency

    path = Path(network_path) if network_path else DEFAULT_NETWORK_PATH
    validator = _load_network_validator()
    edges, connections = validator.parse_network(path)

    passenger = {
        edge_id
        for edge_id, details in edges.items()
        if details["passenger_accessible"] == "true"
    }
    adjacency: Dict[str, Set[str]] = {edge_id: set() for edge_id in passenger}
    incoming: Dict[str, Set[str]] = {edge_id: set() for edge_id in passenger}
    for source, destination in connections:
        if source in passenger and destination in passenger:
            adjacency[source].add(destination)
            incoming[destination].add(source)

    components = validator.strongly_connected_components(passenger, adjacency)
    largest = max(components, key=len, default=[])
    pool = frozenset(
        edge_id for edge_id in largest if adjacency[edge_id] and incoming[edge_id]
    )

    all_links = sum(
        1
        for source, destination in connections
        if source in edges and destination in edges
    )
    pool_links = sum(len([t for t in adjacency[e] if t in pool]) for e in pool)
    GRAPH_STATS.clear()
    GRAPH_STATS.update(
        {
            "graph_non_internal_edges": len(edges),
            "graph_passenger_accessible_edges": len(passenger),
            "graph_candidate_pool_edges": len(pool),
            "graph_successor_links_total": all_links,
            "graph_successor_links_in_pool": pool_links,
            "graph_successor_links_rejected": all_links - pool_links,
        }
    )

    _passenger_accessible = frozenset(passenger)
    _passenger_pool = pool
    # C13: successors are stored already sorted, so every traversal order is fixed.
    _passenger_adjacency = {
        edge_id: sorted(target for target in adjacency[edge_id] if target in pool)
        for edge_id in pool
    }


def passenger_edge_pool(network_path: Optional[Path] = None) -> frozenset:
    """Return the cached candidate pool, building the graph on first use."""
    if _passenger_pool is None:
        build_passenger_graph(network_path)
    return _passenger_pool or frozenset()


def passenger_adjacency() -> Dict[str, List[str]]:
    """Return the cached pool adjacency, building the graph on first use."""
    if _passenger_adjacency is None:
        build_passenger_graph()
    return _passenger_adjacency or {}


def passenger_accessible_edges() -> frozenset:
    """Every passenger-accessible edge, including those outside the largest component."""
    if _passenger_accessible is None:
        build_passenger_graph()
    return _passenger_accessible or frozenset()


def reset_edge_caches() -> None:
    """Drop the graph and the per-edge caches. Used by tests between networks."""
    global _passenger_pool, _passenger_accessible, _passenger_adjacency
    GRAPH_STATS.clear()
    _passenger_pool = None
    _passenger_accessible = None
    _passenger_adjacency = None
    _successor_cache.clear()
    _lane_count_cache.clear()
    _lane_shape_cache.clear()


# ===============================
# EDGES CLOSED BY AN INCIDENT
# ===============================

_blocked_edges: frozenset = frozenset()


def set_blocked_edges(edges) -> None:
    """Tell the routing layer which edges are currently closed by an incident.

    Closures are applied with `traci.lane.setDisallowed`, which changes lane permissions
    but not the network's connections, so neither the static graph nor `traci.lane.getLinks`
    knows about them. Without this, Dijkstra and A* route ordinary traffic straight through
    a closed road and SUMO rejects the replacement with a warning and no exception -
    measured at 145 silently rejected reroutes in a 1,200-second Dijkstra run.

    Runners call this each step from `EmergencyManager.blocked_edges`.
    """
    global _blocked_edges
    _blocked_edges = frozenset(edges or ())


def blocked_edges() -> frozenset:
    return _blocked_edges


def vehicle_is_closure_exempt(veh_id: Optional[str]) -> bool:
    """A dispatched responder may drive through an incident closure; everyone else may not.

    Contract section 11, decision 13.2: a closure blocks every vehicle class except
    `emergency`, so an ambulance must still be able to route over the closed edge - that is
    how it reaches the incident at all.

    PD-1 question 2 (2026-09-13): read from the vehicle's live SUMO class, not its type id. The
    ambient emergency types now run as `authority` (EmergencyManager), which closures block, so
    only the dispatched responders keep class `emergency`. Keying on the type id, as before,
    would route ambient vehicles through roads SUMO no longer lets them enter; the live class
    keeps the router and the simulator in agreement by construction.
    """
    if not veh_id:
        return False
    from emergency_manager import EMERGENCY_EXEMPT_VCLASS

    try:
        return traci.vehicle.getVehicleClass(veh_id) == EMERGENCY_EXEMPT_VCLASS
    except traci.TraCIException:
        return False


# ===============================
# DESTINATION REACHABILITY UNDER CLOSURES (the clean design, 2026-09-15)
# ===============================
#
# While an incident has closed a road, some open roads become dead ends: the only way on from them
# is the closed road. At incident 404, `350915450#0` (2.6 m) leads only into the closed `350915450#1`.
# E3 used to be able to choose such a road, and only found out when `build_destination_valid_route`
# could not route on from it. The reroute then failed, and a car whose route ran through the closure
# stayed on it. Measured: E3 chose 404's dead end 3-7 times per 1,800 s run. The old yield flag
# rescued some of those cars by accident, and misfired on others.
#
# E3 now makes that same check before choosing, with the same router and the vehicle's own type,
# while a closure is on. Only the answer's existence is used, and it depends only on the network, the
# vehicle type and the lane permissions (the closures). So the answers are cached per run, keyed by
# the closed-edge set - the argument the incident-distance cache was validated on.
#
# This is a routing-feasibility rule, not part of the V2V emergency mechanism. It runs whether the
# emergency layer is on or off, so both arms of the ablation have it (user decision 2026-09-15).

_reachability_cache: Dict[tuple, bool] = {}


def can_reach_destination(edge_id: str, destination_edge: str, vehicle_type: str) -> bool:
    """True when SUMO's router finds a route from `edge_id` to `destination_edge` for this vehicle
    type under the current lane permissions - the check `build_destination_valid_route` makes."""
    if edge_id == destination_edge:
        return True
    key = (edge_id, destination_edge, vehicle_type, _blocked_edges)
    answer = _reachability_cache.get(key)
    if answer is None:
        try:
            found = traci.simulation.findRoute(edge_id, destination_edge, vehicle_type, -1.0, 0)
            edges = list(found.edges)
            answer = bool(edges) and edges[0] == edge_id and edges[-1] == destination_edge
        except traci.TraCIException:
            answer = False
        _reachability_cache[key] = answer
    return answer


def reachable_candidates(veh_id: str, candidates: Sequence[str], destination_edge: str) -> List[str]:
    """The candidates from which `veh_id` can still reach `destination_edge`, while a closure is on.

    - With no closure active, or fewer than two candidates, the list is returned unchanged and nothing
      is asked.
    - If no candidate can reach the destination, the original list is kept (the user's fallback),
      which is the behaviour before this filter.
    - The order is preserved.
    """
    candidates = list(candidates)
    if not _blocked_edges or len(candidates) < 2:
        return candidates
    vehicle_type = vehicle_type_for_routing(veh_id)
    kept = [edge for edge in candidates if can_reach_destination(edge, destination_edge, vehicle_type)]
    if not kept:
        ROUTE_STATS["closure_filter_fallbacks"] += 1
        return candidates
    if len(kept) < len(candidates):
        ROUTE_STATS["closure_filtered_decisions"] += 1
        ROUTE_STATS["closure_candidates_removed"] += len(candidates) - len(kept)
    return kept


# ===============================
# SUCCESSOR ENUMERATION (C6, C11, C13, C14)
# ===============================

_successor_cache: Dict[str, List[str]] = {}
_lane_count_cache: Dict[str, int] = {}


def _lane_count(edge_id: str) -> int:
    if edge_id not in _lane_count_cache:
        try:
            _lane_count_cache[edge_id] = int(traci.edge.getLaneNumber(edge_id))
        except traci.TraCIException:
            _lane_count_cache[edge_id] = 0
    return _lane_count_cache[edge_id]


def _successors_from_traci(edge_id: str) -> List[str]:
    """Fallback enumeration for an edge that is not in the pool graph."""
    raw: Set[str] = set()
    internal = 0
    # C14: union the links of every lane, not just lane 0.
    for lane_index in range(_lane_count(edge_id)):
        try:
            links = traci.lane.getLinks(f"{edge_id}_{lane_index}")
        except traci.TraCIException:
            continue
        for link in links:
            next_edge = link[0].split("_")[0]
            # C6: defensive only. Measured on this network, getLinks never returns an
            # internal approached lane, so this filter removes nothing here.
            if next_edge.startswith(":"):
                internal += 1
                continue
            raw.add(next_edge)

    pool = passenger_edge_pool()
    allowed = sorted(next_edge for next_edge in raw if next_edge in pool)
    ROUTE_STATS["topology_successors_seen"] += len(raw) + internal
    ROUTE_STATS["topology_rejected_internal"] += internal
    ROUTE_STATS["topology_rejected_class"] += len(raw) - len(allowed)
    return allowed


def get_outgoing_edges(
    edge_id: str,
    count_stats: bool = True,
    veh_id: Optional[str] = None,
    exclude_blocked: Optional[bool] = None,
) -> List[str]:
    """Successor edges of `edge_id`, class-filtered and deterministically ordered.

    Answers come from the static pool graph, which is why they are free of TraCI round
    trips and identical between runs. An edge outside the pool - a vehicle can be sitting
    on one - falls back to enumerating its lane links through TraCI.

    The dynamic part of the network, lanes closed by an incident via `setDisallowed`, is
    deliberately not reflected here: it is caught by `build_destination_valid_route`, which
    routes with the vehicle's own vehicle type and so respects live permissions.
    """
    if not edge_id or edge_id.startswith(":"):
        return []

    if exclude_blocked is None:
        exclude_blocked = not vehicle_is_closure_exempt(veh_id)

    adjacency = passenger_adjacency()
    successors = adjacency.get(edge_id)
    if successors is None:
        successors = _successor_cache.get(edge_id)
        if successors is None:
            successors = _successors_from_traci(edge_id)
            _successor_cache[edge_id] = successors

    if exclude_blocked and _blocked_edges:
        successors = [edge for edge in successors if edge not in _blocked_edges]

    if count_stats and not successors:
        ROUTE_STATS["no_candidate_edges"] += 1
    # A copy, so a caller cannot mutate the cached adjacency.
    return list(successors)


# ===============================
# LANE GEOMETRY CACHE (C7)
# ===============================

_lane_shape_cache: Dict[str, Optional[tuple]] = {}


def lane_end_point(edge_id: str) -> Optional[tuple]:
    """Cached (x, y) of the end of an edge's first lane, for heuristics."""
    if edge_id not in _lane_shape_cache:
        try:
            shape = traci.lane.getShape(f"{edge_id}_0")
            _lane_shape_cache[edge_id] = tuple(shape[-1]) if shape else None
        except traci.TraCIException:
            _lane_shape_cache[edge_id] = None
    return _lane_shape_cache[edge_id]


# ===============================
# DESTINATION-VALID ROUTE CONSTRUCTION (C1)
# ===============================

def vehicle_type_for_routing(veh_id: str) -> str:
    """The vehicle's own vType id, used so routing respects its vehicle class.

    The literal string `"passenger"` must never be used here. It is a vehicle *class*, not
    a vType id; this scenario's vTypes are `normal_passenger`, `ambulance`,
    `emergency_ambulance` and so on. Passing `"passenger"` makes `findRoute` raise
    "The vehicle type 'passenger' is not known", which is what made E3's own reroutes
    silently do nothing.
    """
    try:
        return traci.vehicle.getTypeID(veh_id)
    except traci.TraCIException:
        return ""


def remaining_route(veh_id: str) -> List[str]:
    """The part of the vehicle's route it has not driven yet, starting at its current edge.

    `traci.vehicle.getRoute` returns the whole route including edges already travelled, and
    it keeps growing after each reroute. Locating the current edge with `list.index` is
    therefore wrong as soon as a route revisits an edge - it finds the historical
    occurrence. `getRouteIndex` gives the true position.
    """
    try:
        route = list(traci.vehicle.getRoute(veh_id))
        index = traci.vehicle.getRouteIndex(veh_id)
    except traci.TraCIException:
        return []
    if not route:
        return []
    if index is None or index < 0 or index >= len(route):
        return route
    return route[index:]


def _classify_findroute_error(message: str) -> str:
    """Name the cause reported by findRoute, so failures can be classified."""
    text = message.lower()
    if "not allowed" in text and "destination" in text:
        return "destination_edge_not_usable_by_this_vehicle"
    if "not allowed" in text:
        return "edge_not_usable_by_this_vehicle"
    if "no connection" in text or "no route" in text:
        return "no_path_from_selected_edge_to_destination"
    return "findroute_other_error"


def build_destination_valid_route(
    veh_id: str,
    current_route: Sequence[str],
    current_edge: str,
    selected_edge: str,
) -> List[str]:
    """Route the vehicle over `selected_edge` while keeping its original destination.

    The result starts at the vehicle's current edge and ends at the destination of
    `current_route`. It is the *remaining* route: SUMO keeps the already-travelled edges by
    itself and re-attaches them, and passing them back in makes it log
    "Invalid route replacement ... No connection" on every single reroute (measured: 14 of
    14 reroutes warn with the travelled prefix included, 0 of 14 without it, with identical
    resulting routes).

    Returns the current remaining route unchanged when no valid continuation exists, so the
    caller can always apply the result safely.
    """
    route = list(current_route)
    if not route:
        return []
    destination_edge = route[-1]

    def unchanged() -> List[str]:
        """The vehicle's current remaining route: applying it is a no-op."""
        return remaining_route(veh_id) or route

    if current_edge not in route or current_edge == destination_edge:
        return unchanged()

    if selected_edge == destination_edge:
        continuation = [selected_edge]
    else:
        try:
            found = traci.simulation.findRoute(
                selected_edge,
                destination_edge,
                vehicle_type_for_routing(veh_id),
                -1.0,
                0,
            )
            continuation = list(found.edges)
        except traci.TraCIException as error:
            record_route_failure(_classify_findroute_error(str(error)))
            return unchanged()
        if not continuation:
            record_route_failure("findroute_returned_empty")
            return unchanged()
        if continuation[0] != selected_edge:
            record_route_failure("continuation_does_not_start_at_selected_edge")
            return unchanged()

    if continuation[-1] != destination_edge:
        record_route_failure("continuation_does_not_reach_destination")
        return unchanged()

    return [current_edge] + continuation


def build_route_from_path(
    current_edge: str,
    path: Sequence[str],
    destination_edge: str,
) -> List[str]:
    """Validate a self-computed path (Dijkstra, A*) into an applicable remaining route.

    Returns an empty list if the path does not start at the current edge or does not end at
    the destination, which the caller then treats as "no reroute".
    """
    edges = list(path)
    if not edges or edges[0] != current_edge or edges[-1] != destination_edge:
        if edges:
            record_route_failure("computed_path_has_wrong_endpoints")
        return []
    return edges


# ===============================
# CHECKED ROUTE APPLICATION (C1, contract section 5)
# ===============================

def set_route_checked(
    veh_id: str,
    new_route: Sequence[str],
    current_route: Optional[Sequence[str]] = None,
) -> bool:
    """Apply a route only if it preserves the vehicle's destination, and confirm it stuck.

    `new_route` is a remaining route: it must start at the vehicle's current edge. Returns
    True when SUMO accepted a genuinely different route. Every rejection is counted,
    because contract section 5 requires route failures to be reported, not swallowed.
    """
    proposed = list(new_route or [])
    if not proposed:
        return False

    old_full = list(current_route) if current_route is not None else None
    if old_full is None:
        try:
            old_full = list(traci.vehicle.getRoute(veh_id))
        except traci.TraCIException:
            return False
    if not old_full:
        return False

    old_remaining = remaining_route(veh_id) or old_full
    if proposed == old_remaining:
        return False

    ROUTE_STATS["reroute_proposals"] += 1

    if proposed[-1] != old_full[-1]:
        ROUTE_STATS["destination_violations"] += 1
        message = (
            f"Route for {veh_id} would change destination "
            f"{old_full[-1]!r} -> {proposed[-1]!r}"
        )
        if ROUTE_ASSERT_ENABLED:
            raise RouteValidityError(message)
        return False

    # Last line of defence for closures. A route through a closed road is not applicable
    # for ordinary traffic: SUMO reports "Invalid route replacement ... No connection" and
    # the reroute is worthless. The search layer already avoids closed edges; this catches
    # the cases it cannot, such as a continuation returned by findRoute.
    if _blocked_edges and not vehicle_is_closure_exempt(veh_id):
        # The vehicle's own current edge is exempt: it is already standing there.
        if any(edge in _blocked_edges for edge in proposed[1:]):
            ROUTE_STATS["routes_crossing_closed_edge"] += 1
            return False

    try:
        traci.vehicle.setRoute(veh_id, proposed)
    except traci.TraCIException:
        ROUTE_STATS["setroute_errors"] += 1
        return False

    # SUMO can reject a replacement with only a warning and no exception - for instance
    # when the route crosses a road an incident has closed. Comparing destinations is not
    # enough to notice, because a rejected replacement leaves the old route in place and
    # that route has the same destination. Compare the remaining route itself.
    applied = remaining_route(veh_id)
    if applied != proposed:
        ROUTE_STATS["setroute_silent_rejections"] += 1
        return False

    ROUTE_STATS["reroutes_applied"] += 1
    return True


# ===============================
# DETERMINISM (C13 and contract section 5)
# ===============================

def load_route_choice_seed(config_path: Optional[Path] = None) -> int:
    """Read `seeds.route_choice` from config/scenario.yaml."""
    import yaml

    path = Path(config_path) if config_path else DEFAULT_SCENARIO_CONFIG
    with open(path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    return int(config["seeds"]["route_choice"])


def seed_route_choice(seed: Optional[int] = None) -> int:
    """Seed the `random` module used by ACO, ACO-PSO and E3 edge selection.

    Sorted candidate lists (C13) remove the hash-order dependence, but ACO and ACO-PSO
    still draw from `random.choices`, so an explicit seed is required before two runs can
    be byte-identical. Contract section 5 requires the seed to come from
    `config/scenario.yaml`.
    """
    import random

    resolved = load_route_choice_seed() if seed is None else int(seed)
    random.seed(resolved)
    return resolved
