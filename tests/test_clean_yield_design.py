"""The clean yield design (user decision, 2026-09-15).

(a) `yield_request` means only "I am actually yielding": raised in the YIELD_DURATION_S after a real
    yield action, never by proximity to an incident, and it has no routing effect for any receiver.
(b) Responder broadcasts and the emergency-path behaviour are unchanged (the 0.05 avoid, the corridor).
(c) While a closure is on, E3 drops candidates from which SUMO's router, with the vehicle's own vehicle
    type, cannot reach the destination; if that would drop every candidate, the list is kept.

Evidence and history: results/yield_signal_diagnosis_2026-09-14/README.md, THESIS_CODE_AUDIT_FIXES.md.
"""

import pytest

import p3_runtime
from algorithms import e3, e3_core, route_utils
from p3_hybrid_router import (
    NEUTRAL_WEIGHT,
    ORDINARY_AVOID_EMERGENCY_WEIGHT,
    RESPONDER_CORRIDOR_WEIGHT,
    HybridRouter,
    ProtocolMessage,
)


# ------------------------------------------------------------------ (a) the sender

class BroadcastTraci:
    TraCIException = Exception

    def __init__(self, ids=("car",)):
        ids = list(ids)

        class Vehicle:
            @staticmethod
            def getIDList():
                return ids

            @staticmethod
            def getTypeID(v):
                return "ambulance" if v == "resp" else "normal_passenger"

            @staticmethod
            def getRoadID(v):
                return "E"

            @staticmethod
            def getSpeed(v):
                return 10.0

            @staticmethod
            def getLaneID(v):
                return "E_0"

        class Edge:
            @staticmethod
            def getLastStepVehicleNumber(e):
                return 1

            @staticmethod
            def getLastStepMeanSpeed(e):
                return 5.0

        class Lane:
            @staticmethod
            def getMaxSpeed(lane):
                return 10.0

        self.vehicle, self.edge, self.lane = Vehicle(), Edge(), Lane()


class NearIncident:
    """Every car is 150 m from an open incident: well inside the old 200 m trigger."""
    blocked_edges = ()

    def __init__(self, distance=150.0):
        self.distance = distance

    def get_nearest_emergency(self, vehicle_id):
        return None if self.distance is None else ("404", self.distance, None)


def broadcast_at(monkeypatch, t, last_yield=None, distance=150.0, ids=("car",), responders=()):
    monkeypatch.setattr(p3_runtime, "traci", BroadcastTraci(ids))
    monkeypatch.setattr(p3_runtime, "get_emergency_manager", lambda: NearIncident(distance))
    monkeypatch.setattr(p3_runtime, "remaining_route", lambda v: ["E", "F", "dest"])
    c = p3_runtime.P3RuntimeCoordinator(emergency_layer=True)
    c._active_responders = set(responders)
    if last_yield is not None:
        for v in ids:
            c.last_yield[v] = last_yield
    c._broadcast_vehicle_messages(t)
    return {v: c.protocol_state.neighbors[v].message for v in ids}


def test_proximity_to_an_incident_alone_no_longer_raises_the_flag(monkeypatch):
    m = broadcast_at(monkeypatch, 10.0)["car"]
    assert m.yield_request is False and m.priority_level == 0
    assert m.distance_to_emergency == 150.0            # still carried, as its own field


@pytest.mark.parametrize("t, expected", [(10.0, False), (11.0, True), (13.0, True), (13.5, False)])
def test_the_flag_covers_the_three_seconds_after_a_real_yield(monkeypatch, t, expected):
    # A yield at 10 is issued after that step's broadcasts, so the broadcast at 10 predates it.
    m = broadcast_at(monkeypatch, t, last_yield=10.0)["car"]
    assert m.yield_request is expected and m.priority_level == (1 if expected else 0)


def test_a_yielding_car_far_from_any_incident_still_says_so(monkeypatch):
    m = broadcast_at(monkeypatch, 11.0, last_yield=10.0, distance=None)["car"]
    assert m.yield_request is True and m.distance_to_emergency is None


def test_an_active_responder_never_raises_it(monkeypatch):
    m = broadcast_at(monkeypatch, 11.0, last_yield=10.0, ids=("resp",), responders=("resp",))["resp"]
    assert m.emergency_flag is True and m.yield_request is False and m.priority_level == 4


# ------------------------------------------------------------------ (a) no routing effect; (b) unchanged

def test_a_yield_flag_has_no_routing_effect_for_an_ordinary_car_or_a_responder():
    router = HybridRouter()
    yielding = ProtocolMessage(vehicle_id="n1", yield_request=True, priority_level=1, next_edge="x")
    assert router.protocol_weight_for_edge("x", [yielding]) == NEUTRAL_WEIGHT
    assert router.protocol_weight_for_edge("x", [yielding], is_emergency_vehicle=True) == NEUTRAL_WEIGHT


def test_the_responder_broadcast_still_carries_the_emergency_path():
    router = HybridRouter()
    yielding = ProtocolMessage(vehicle_id="n1", yield_request=True, priority_level=1, next_edge="x")
    responder = ProtocolMessage(vehicle_id="resp", emergency_flag=True, priority_level=4, next_edge="x")
    assert router.protocol_weight_for_edge("x", [yielding, responder]) == ORDINARY_AVOID_EMERGENCY_WEIGHT
    assert router.protocol_weight_for_edge("x", [yielding], is_emergency_vehicle=True,
                                           corridor_edge="x") == RESPONDER_CORRIDOR_WEIGHT


# ------------------------------------------------------------------ (c) the reachability filter

class Found:
    def __init__(self, edges):
        self.edges = edges


class RouteTraci:
    """findRoute answers from a table of (from, to) -> route edges; records every call."""

    TraCIException = type("TraCIException", (Exception,), {})

    def __init__(self, routes, raising=()):
        self.calls = []
        outer = self

        class Simulation:
            @staticmethod
            def findRoute(frm, to, vtype, depart, mode):
                outer.calls.append((frm, to, vtype))
                if frm in raising:
                    raise outer.TraCIException("no route")
                return Found(list(routes.get((frm, to), [])))

        class Vehicle:
            @staticmethod
            def getTypeID(v):
                return "ambulance" if v == "resp" else "normal_passenger"

        self.simulation, self.vehicle = Simulation(), Vehicle()


ROUTES = {("a", "dest"): ["a", "m", "dest"], ("b", "dest"): ["b", "dest"], ("dead", "dest"): []}


@pytest.fixture
def closure(monkeypatch):
    fake = RouteTraci(ROUTES, raising=("boom",))
    monkeypatch.setattr(route_utils, "traci", fake)
    route_utils.reset_route_stats()
    route_utils.set_blocked_edges({"closed"})
    yield fake
    route_utils.set_blocked_edges(())
    route_utils.reset_route_stats()


def test_no_closure_no_filter_and_no_query(monkeypatch):
    fake = RouteTraci(ROUTES)
    monkeypatch.setattr(route_utils, "traci", fake)
    route_utils.set_blocked_edges(())
    assert route_utils.reachable_candidates("car", ["a", "dead", "b"], "dest") == ["a", "dead", "b"]
    assert fake.calls == []


def test_during_a_closure_a_dead_end_is_removed_and_the_order_kept(closure):
    assert route_utils.reachable_candidates("car", ["a", "dead", "b"], "dest") == ["a", "b"]
    stats = route_utils.get_route_stats()
    assert stats["closure_filtered_decisions"] == 1 and stats["closure_candidates_removed"] == 1
    assert {vtype for _, _, vtype in closure.calls} == {"normal_passenger"}   # the car's own type


def test_the_check_uses_the_vehicles_own_type(closure):
    route_utils.reachable_candidates("resp", ["a", "b"], "dest")
    assert {vtype for _, _, vtype in closure.calls} == {"ambulance"}


def test_if_nothing_can_reach_the_destination_the_list_is_kept(closure):
    assert route_utils.reachable_candidates("car", ["dead", "boom"], "dest") == ["dead", "boom"]
    assert route_utils.get_route_stats()["closure_filter_fallbacks"] == 1


def test_a_single_candidate_is_never_filtered_or_queried(closure):
    assert route_utils.reachable_candidates("car", ["dead"], "dest") == ["dead"]
    assert closure.calls == []


def test_a_route_that_does_not_reach_the_destination_does_not_count(closure, monkeypatch):
    table = {**ROUTES, ("x", "dest"): ["x", "elsewhere"], ("y", "dest"): ["z", "dest"]}
    monkeypatch.setattr(route_utils, "traci", RouteTraci(table))
    assert route_utils.reachable_candidates("car", ["x", "y", "b"], "dest") == ["b"]


def test_the_destination_itself_needs_no_query(closure):
    assert route_utils.can_reach_destination("dest", "dest", "normal_passenger") is True
    assert closure.calls == []


def test_answers_are_cached_per_closure_set_and_per_run(closure):
    route_utils.reachable_candidates("car", ["a", "dead"], "dest")
    route_utils.reachable_candidates("car", ["a", "dead"], "dest")
    assert len(closure.calls) == 2                      # asked once each
    route_utils.set_blocked_edges({"closed", "another"})
    route_utils.reachable_candidates("car", ["a", "dead"], "dest")
    assert len(closure.calls) == 4                      # a different closure set is a different question
    route_utils.reset_route_stats()
    route_utils.reachable_candidates("car", ["a", "dead"], "dest")
    assert len(closure.calls) == 6                      # a new run forgets


def test_decide_never_offers_the_dead_end_during_a_closure(closure, monkeypatch):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    offered = []

    class Traci:
        TraCIException = Exception

        class vehicle:
            @staticmethod
            def getRoadID(v):
                return "here"

            @staticmethod
            def getRoute(v):
                return ("here", "a", "dest")

    monkeypatch.setattr(e3_core, "traci", Traci)
    monkeypatch.setattr(e3_core, "get_outgoing_edges", lambda edge, veh_id=None: ["a", "dead", "b"])
    monkeypatch.setattr(core, "select_edge", lambda v, outgoing, *a, **k: (offered.append(list(outgoing)) or ("b", 2)))
    monkeypatch.setattr(e3_core, "build_destination_valid_route", lambda v, r, cur, sel: ["here", sel, "dest"])
    monkeypatch.setattr(e3_core, "set_route_checked", lambda v, new, old: True)
    monkeypatch.setattr(core, "note_applied_route", lambda *a: None)
    core.decide("car", 30.0)
    assert offered == [["a", "b"]]


def test_the_six_algorithm_path_applies_the_same_rule(closure, monkeypatch):
    core = e3_core.E3Core(emergency_layer=True, reset_state=True)
    offered = []
    monkeypatch.setattr(e3, "get_core", lambda: core)

    class Traci:
        TraCIException = Exception

        class vehicle:
            @staticmethod
            def getRoute(v):
                return ("here", "a", "dest")

    monkeypatch.setattr(e3, "traci", Traci)
    monkeypatch.setattr(e3, "get_outgoing_edges", lambda edge, veh_id=None: ["a", "dead", "b"])
    monkeypatch.setattr(core, "select_edge", lambda v, outgoing, *a, **k: (offered.append(list(outgoing)) or ("b", 2)))
    monkeypatch.setattr(e3, "build_destination_valid_route", lambda v, r, cur, sel: ["here", sel, "dest"])

    class Manager:
        @staticmethod
        def is_active_responder(v):
            return False

    import emergency_manager
    monkeypatch.setattr(emergency_manager, "get_emergency_manager", lambda: Manager())
    e3.select_route("car", "here")
    assert offered == [["a", "b"]]
