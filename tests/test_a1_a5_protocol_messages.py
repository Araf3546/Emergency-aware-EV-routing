"""Audit A1 and A5 (2026-09-14), with the user's decision on A1 (option 2).

A1. Every vehicle used to score its own broadcast as though a neighbour had sent it.
- An ordinary car within 200 m of an incident announces a yield request naming its own next edge,
  so the minimum rule scored its own planned edge 0.3 and pushed it off its route.
- In the 1,800 s validation this happened in 4 decisions, all pushing a car back toward incident
  404's closed approach.
- A vehicle now never hears itself.
- The responder's claim on its own corridor (P8, ablation gate 3) is kept, but as an explicit
  emergency-layer rule on the edge it is about to enter, not as a self-received message.

A5. A message naming no next edge used to count for every edge. It now counts for none.

2026-09-15, the clean design: a yield flag no longer has any routing effect. The tests below that
used a yield message as "a signal that acts" now use an emergency message, or assert neutrality.
"""

import pytest

import p3_runtime
from p3_hybrid_router import (
    EMERGENCY_CORRIDOR_WEIGHT,
    NEUTRAL_WEIGHT,
    ORDINARY_AVOID_EMERGENCY_WEIGHT,
    RESPONDER_CORRIDOR_WEIGHT,
    HybridRouter,
    ProtocolMessage,
)
from p3_protocol import ProtocolStateManager, build_emergency_message, build_message


def message(sender, next_edge, t=0.0, **kw):
    return build_message(vehicle_id=sender, current_time=t, vehicle_type="normal_passenger",
                         position_edge="e0", speed=10.0, next_edge=next_edge, **kw)


# ------------------------------------------------------------------ A1: the message store


def test_a_vehicle_never_hears_its_own_broadcast():
    state = ProtocolStateManager()
    state.update(0.0, message("me", "x", yield_request=True))
    state.update(0.0, message("other", "y", yield_request=True))
    assert [m.vehicle_id for m in state.get_fresh_neighbors(0.5, exclude="me")] == ["other"]
    assert {m.vehicle_id for m in state.get_fresh_neighbors(0.5)} == {"me", "other"}   # default unchanged


def test_a_responder_still_hears_another_responders_emergency_message_but_not_its_own():
    state = ProtocolStateManager()
    for sender in ("resp_a", "resp_b"):
        state.update(0.0, build_emergency_message(vehicle_id=sender, current_time=0.0,
                                                  vehicle_type="ambulance", position_edge="e0",
                                                  speed=10.0, route_intent="x", next_edge="x"))
    assert [m.vehicle_id for m in state.get_emergency_neighbors(0.5, exclude="resp_a")] == ["resp_b"]


# ------------------------------------------------------------------ A1: the runtime passes no self-message


class FakeManager:
    blocked_edges = ()

    def get_active_emergency_zones(self):
        return []


@pytest.fixture
def coordinator(monkeypatch):
    monkeypatch.setattr(p3_runtime, "get_emergency_manager", lambda: FakeManager())
    monkeypatch.setattr(p3_runtime, "remaining_route", lambda vehicle_id: ["here", "planned", "dest"])
    c = p3_runtime.P3RuntimeCoordinator(emergency_layer=True)
    calls = []

    def decide(vehicle_id, sim_time, messages=None, is_emergency_vehicle=False, corridor_edge=None):
        calls.append({"vehicle": vehicle_id, "messages": list(messages or ()),
                      "is_emergency": is_emergency_vehicle, "corridor_edge": corridor_edge})
        return {"reason": "no_candidates", "applied": False, "candidates": 0}

    monkeypatch.setattr(c.e3, "decide", decide)
    c.calls = calls
    return c


def test_an_ordinary_cars_own_yield_request_no_longer_reaches_its_own_decision(coordinator):
    coordinator.protocol_state.update(10.0, message("car", "planned", t=10.0, yield_request=True))
    coordinator.protocol_state.update(10.0, message("neighbour", "other", t=10.0, yield_request=True))
    coordinator._apply_hybrid_decision("car", 10.0)
    call = coordinator.calls[-1]
    assert [m.vehicle_id for m in call["messages"]] == ["neighbour"]
    assert call["corridor_edge"] is None and call["is_emergency"] is False
    # Scored with what it now receives, its own planned edge is neutral. Since the clean design
    # (2026-09-15) the neighbour's yield flag is neutral too: it has no routing effect.
    router = HybridRouter()
    assert router.protocol_weight_for_edge("planned", call["messages"]) == NEUTRAL_WEIGHT
    assert router.protocol_weight_for_edge("other", call["messages"]) == NEUTRAL_WEIGHT


def test_an_active_responder_gets_the_explicit_corridor_and_not_its_own_message(coordinator):
    coordinator._active_responders = {"resp"}
    coordinator.protocol_state.update(10.0, build_emergency_message(
        vehicle_id="resp", current_time=10.0, vehicle_type="ambulance", position_edge="here",
        speed=10.0, route_intent="planned", next_edge="planned"))
    coordinator._apply_hybrid_decision("resp", 10.0)
    call = coordinator.calls[-1]
    assert call["messages"] == [] and call["is_emergency"] is True
    assert call["corridor_edge"] == "planned"          # the edge it is about to enter, from its route


def test_with_the_emergency_layer_off_there_is_no_corridor_claim(monkeypatch, coordinator):
    coordinator.emergency_layer = False
    coordinator._active_responders = {"resp"}
    coordinator._apply_hybrid_decision("resp", 10.0)
    assert coordinator.calls[-1]["corridor_edge"] is None and coordinator.calls[-1]["is_emergency"] is False


# ------------------------------------------------------------------ the explicit corridor rule


def test_the_corridor_rule_gives_exactly_what_the_self_received_broadcast_gave():
    router = HybridRouter()
    own = ProtocolMessage(vehicle_id="resp", emergency_flag=True, priority_level=4, next_edge="planned")
    old = router.protocol_weight_for_edge("planned", [own], is_emergency_vehicle=True)
    new = router.protocol_weight_for_edge("planned", [], is_emergency_vehicle=True, corridor_edge="planned")
    assert new == old == RESPONDER_CORRIDOR_WEIGHT == pytest.approx(EMERGENCY_CORRIDOR_WEIGHT + 0.1)
    assert router.protocol_weight_for_edge("alt", [], is_emergency_vehicle=True,
                                           corridor_edge="planned") == NEUTRAL_WEIGHT


def test_the_corridor_rule_is_ignored_for_ordinary_vehicles_and_with_the_layer_off():
    assert HybridRouter().protocol_weight_for_edge("planned", [], is_emergency_vehicle=False,
                                                   corridor_edge="planned") == NEUTRAL_WEIGHT
    off = HybridRouter(emergency_layer_enabled=False)
    assert off.protocol_weight_for_edge("planned", [], is_emergency_vehicle=True,
                                        corridor_edge="planned") == NEUTRAL_WEIGHT


def test_a_responder_still_takes_the_strongest_genuine_neighbour_claim():
    router = HybridRouter()
    other = ProtocolMessage(vehicle_id="resp_b", emergency_flag=True, priority_level=4, next_edge="x")
    routine = ProtocolMessage(vehicle_id="car", next_edge="x")
    assert router.protocol_weight_for_edge("x", [other, routine], is_emergency_vehicle=True) == 1.0


def test_emergency_messages_from_others_still_act_on_ordinary_cars_and_yield_flags_do_not():
    router = HybridRouter()
    yielding = ProtocolMessage(vehicle_id="n1", yield_request=True, priority_level=1, next_edge="x")
    responder = ProtocolMessage(vehicle_id="resp", emergency_flag=True, priority_level=4, next_edge="y")
    assert router.protocol_weight_for_edge("x", [yielding, responder]) == NEUTRAL_WEIGHT          # clean design
    assert router.protocol_weight_for_edge("y", [yielding, responder]) == ORDINARY_AVOID_EMERGENCY_WEIGHT
    assert router.protocol_weight_for_edge("z", [yielding, responder]) == NEUTRAL_WEIGHT


# ------------------------------------------------------------------ yielding is unchanged


class YieldTraci:
    """A responder on single-lane edge E with one moving car ahead of it on E, inside the yield radius."""

    TraCIException = type("TraCIException", (Exception,), {})

    def __init__(self):
        self.slowed, self.lane_changes = [], []
        outer = self

        class Vehicle:
            @staticmethod
            def getIDList():
                return ["resp", "car"]

            @staticmethod
            def getPosition(v):
                return {"resp": (0.0, 0.0), "car": (30.0, 0.0)}[v]

            @staticmethod
            def getRoadID(v):
                return "E"

            @staticmethod
            def getLanePosition(v):
                return {"resp": 10.0, "car": 40.0}[v]

            @staticmethod
            def getSpeed(v):
                return 12.0

            @staticmethod
            def slowDown(v, speed, duration):
                outer.slowed.append(v)

            @staticmethod
            def getLaneIndex(v):
                return 0

            @staticmethod
            def getLaneID(v):
                return "E_0"

            @staticmethod
            def getVehicleClass(v):
                return "emergency" if v == "resp" else "passenger"

            @staticmethod
            def getLeader(v, dist):
                return ("car", 22.5) if v == "resp" else None

            @staticmethod
            def getBestLanes(v):
                return (("E_0", 100.0, 0.0, 0, True, ()),)

            @staticmethod
            def changeLane(v, lane, duration):
                outer.lane_changes.append(v)

        class Edge:
            @staticmethod
            def getLastStepVehicleIDs(e):
                return ["resp", "car"] if e == "E" else []

            @staticmethod
            def getLaneNumber(e):
                return 1

        class Lane:
            @staticmethod
            def getAllowed(lane_id):
                return ()

            @staticmethod
            def getDisallowed(lane_id):
                return ()

        self.vehicle, self.edge, self.lane = Vehicle(), Edge(), Lane()


def test_yielding_still_reacts_to_the_responders_broadcast(monkeypatch):
    fake = YieldTraci()
    monkeypatch.setattr(p3_runtime, "traci", fake)
    c = p3_runtime.P3RuntimeCoordinator(emergency_layer=True)
    c._active_responders = {"resp"}
    c.protocol_state.update(5.0, build_emergency_message(
        vehicle_id="resp", current_time=5.0, vehicle_type="ambulance", position_edge="E",
        speed=12.0, route_intent="F", next_edge="F"))
    c._apply_yield_behaviour(5.0)
    # The car ahead is asked, and the responder itself never is. Since the unified rule (2026-09-15) a car on
    # a single-lane road cannot make room, so it keeps driving: it is neither braked nor moved, and does not
    # say it is yielding.
    assert c.last_asked_to_yield == {"car": 5.0}
    assert fake.slowed == [] and fake.lane_changes == [] and c.last_yield == {}


# ------------------------------------------------------------------ A5


@pytest.mark.parametrize("kind", ["emergency", "yield", "routine"])
@pytest.mark.parametrize("receiver_is_responder", [False, True])
def test_a_message_naming_no_next_edge_says_nothing_about_any_edge(kind, receiver_is_responder):
    m = ProtocolMessage(vehicle_id="s", next_edge=None, emergency_flag=(kind == "emergency"),
                        yield_request=(kind == "yield"), priority_level=4 if kind == "emergency" else 1)
    router = HybridRouter()
    for edge in ("a", "b", "anything"):
        assert router.protocol_weight_for_edge(edge, [m], is_emergency_vehicle=receiver_is_responder) \
            == NEUTRAL_WEIGHT


def test_a_named_message_acts_only_on_its_edge_and_a_nameless_one_no_longer_flattens_it():
    router = HybridRouter()
    named = ProtocolMessage(vehicle_id="resp_a", emergency_flag=True, priority_level=4, next_edge="x")
    nameless = ProtocolMessage(vehicle_id="resp_b", emergency_flag=True, priority_level=4, next_edge=None)
    # Before A5 the nameless emergency message set every edge to 0.05, y included. (This used a yield
    # message as the named signal until the clean design gave yield flags no routing effect.)
    assert router.protocol_weight_for_edge("x", [named, nameless]) == ORDINARY_AVOID_EMERGENCY_WEIGHT
    assert router.protocol_weight_for_edge("y", [named, nameless]) == NEUTRAL_WEIGHT
