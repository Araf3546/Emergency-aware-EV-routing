"""Tests for the shared route helpers (audit C1, C11, C13, C14).

These call the real functions in `algorithms/route_utils.py` rather than re-implementing
them (audit finding R8). The graph tests use the real Porto network; the routing tests
replace the module's `traci` handle with a stub, because they are about the decision logic,
not about SUMO.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from algorithms import route_utils
from algorithms.route_utils import (
    GRAPH_STATS,
    RouteValidityError,
    build_destination_valid_route,
    build_route_from_path,
    get_outgoing_edges,
    passenger_adjacency,
    passenger_edge_pool,
    reset_route_stats,
    set_route_checked,
)


# ---------------------------------------------------------------- graph / candidate pool


def test_candidate_pool_matches_the_validated_network_pool():
    """The pool must be the same 5,659 edges scripts/06 uses to choose OD endpoints."""
    pool = passenger_edge_pool()
    assert len(pool) == 5659
    assert GRAPH_STATS["graph_non_internal_edges"] == 21385
    assert GRAPH_STATS["graph_passenger_accessible_edges"] == 6065


def test_candidate_pool_excludes_most_of_the_network():
    """C11: the majority of successor links lead to edges no passenger car may use."""
    passenger_edge_pool()
    total = GRAPH_STATS["graph_successor_links_total"]
    rejected = GRAPH_STATS["graph_successor_links_rejected"]
    assert rejected > total / 2


def test_successors_are_sorted_and_inside_the_pool():
    """C13: candidate order is fixed, never `list(set(...))`."""
    pool = passenger_edge_pool()
    adjacency = passenger_adjacency()
    checked = 0
    for edge_id in sorted(adjacency)[:500]:
        successors = get_outgoing_edges(edge_id)
        assert successors == sorted(successors)
        assert all(successor in pool for successor in successors)
        checked += 1
    assert checked == 500


def test_internal_edges_have_no_candidates():
    assert get_outgoing_edges(":junction_0_0") == []
    assert get_outgoing_edges("") == []


# ---------------------------------------------------------------------- routing helpers


class StubTraCI:
    """Minimal stand-in for the parts of traci the route helpers touch."""

    class TraCIException(Exception):
        pass

    def __init__(self, route, route_index, vehicle_type="normal_passenger", found=None):
        self._route = list(route)
        self._route_index = route_index
        self._vehicle_type = vehicle_type
        self._found = found
        self.set_routes = []

        stub = self

        class Found:
            def __init__(self, edges):
                self.edges = edges

        class Vehicle:
            def getRoute(self, veh_id):
                return list(stub._route)

            def getRouteIndex(self, veh_id):
                return stub._route_index

            def getTypeID(self, veh_id):
                return stub._vehicle_type

            def setRoute(self, veh_id, edges):
                stub.set_routes.append(list(edges))
                stub._route = stub._route[: stub._route_index] + list(edges)

        class Simulation:
            def findRoute(self, origin, destination, vehicle_type, depart, mode):
                if stub._found is None:
                    raise StubTraCI.TraCIException("no route")
                return Found(list(stub._found))

        self.vehicle = Vehicle()
        self.simulation = Simulation()


@pytest.fixture
def stub(monkeypatch):
    def install(**kwargs):
        stub_traci = StubTraCI(**kwargs)
        monkeypatch.setattr(route_utils, "traci", stub_traci)
        reset_route_stats()
        return stub_traci

    return install


def test_built_route_keeps_the_destination_and_starts_at_the_current_edge(stub):
    """C1: a reroute changes the path, never the destination."""
    stub(route=["a", "b", "c", "d"], route_index=1, found=["x", "y", "d"])
    new_route = build_destination_valid_route("v1", ["a", "b", "c", "d"], "b", "x")
    assert new_route == ["b", "x", "y", "d"]
    assert new_route[-1] == "d"


def test_build_falls_back_to_the_current_route_when_no_path_exists(stub):
    stub(route=["a", "b", "c", "d"], route_index=1, found=None)
    new_route = build_destination_valid_route("v1", ["a", "b", "c", "d"], "b", "x")
    assert new_route == ["b", "c", "d"]
    assert route_utils.ROUTE_STATS["route_build_failures"] == 1


def test_build_rejects_a_continuation_that_ends_elsewhere(stub):
    stub(route=["a", "b", "c", "d"], route_index=1, found=["x", "y", "z"])
    new_route = build_destination_valid_route("v1", ["a", "b", "c", "d"], "b", "x")
    assert new_route == ["b", "c", "d"]
    assert route_utils.ROUTE_STATS["route_build_failures"] == 1


def test_build_route_from_path_requires_both_endpoints():
    reset_route_stats()
    assert build_route_from_path("b", ["b", "c", "d"], "d") == ["b", "c", "d"]
    assert build_route_from_path("b", ["c", "d"], "d") == []
    assert build_route_from_path("b", ["b", "c"], "d") == []
    assert route_utils.ROUTE_STATS["route_build_failures"] == 2


def test_set_route_checked_applies_a_valid_reroute(stub):
    stub_traci = stub(route=["a", "b", "c", "d"], route_index=1, found=None)
    assert set_route_checked("v1", ["b", "x", "y", "d"]) is True
    assert stub_traci.set_routes == [["b", "x", "y", "d"]]
    assert route_utils.ROUTE_STATS["reroutes_applied"] == 1
    assert route_utils.ROUTE_STATS["destination_violations"] == 0


def test_set_route_checked_refuses_a_truncated_route(stub):
    """The exact shape of the C1 bug: prefix plus one edge, ending at the wrong place."""
    stub_traci = stub(route=["a", "b", "c", "d"], route_index=1, found=None)
    with pytest.raises(RouteValidityError):
        set_route_checked("v1", ["a", "b", "x"])
    assert stub_traci.set_routes == []
    assert route_utils.ROUTE_STATS["destination_violations"] == 1
    assert route_utils.ROUTE_STATS["reroutes_applied"] == 0


def test_set_route_checked_ignores_an_unchanged_route(stub):
    stub_traci = stub(route=["a", "b", "c", "d"], route_index=1, found=None)
    assert set_route_checked("v1", ["b", "c", "d"]) is False
    assert stub_traci.set_routes == []
    assert route_utils.ROUTE_STATS["reroute_proposals"] == 0
