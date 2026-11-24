"""The closure reachability filter in both arms of the emergency-layer ablation (user decision 2026-09-15).

The filter (`route_utils.reachable_candidates`) is a routing-feasibility rule, not part of the V2V
emergency mechanism. It must act the same with the emergency layer ON and OFF:
- valid candidates are kept, in order;
- a candidate the vehicle cannot reach its destination from is removed, even when E3 would have chosen it;
- if that would remove every candidate, the original list is kept.

The switch must still gate exactly the three emergency mechanisms it always gated, and nothing else:
  1. the protocol term in edge scoring (`HybridRouter.protocol_weight_for_edge`, neutral when off);
  2. yielding (`P3RuntimeCoordinator._apply_yield_behaviour`);
  3. the responder's corridor claim (`P3RuntimeCoordinator._apply_hybrid_decision`).

Real-run evidence: results/closure_filter_both_arms_2026-09-15/README.md.
"""

import ast
from pathlib import Path

import pytest

import p3_runtime
from algorithms import e3_core, route_utils
from p3_hybrid_router import (
    NEUTRAL_WEIGHT,
    ORDINARY_AVOID_EMERGENCY_WEIGHT,
    RESPONDER_CORRIDOR_WEIGHT,
    HybridRouter,
    ProtocolMessage,
)
from p3_protocol import build_emergency_message

REPO = Path(__file__).resolve().parent.parent
ARMS = pytest.mark.parametrize("layer", [True, False], ids=["layer_on", "layer_off"])


class FakeTraCIError(Exception):
    """Distinct from Exception, so a real error inside the code under test is not swallowed as a TraCI one."""


# ------------------------------------------------------------------ fakes

class Found:
    def __init__(self, edges):
        self.edges = edges


# While the closure is on, "dead" and "dead2" lead nowhere for a car. A dispatched responder may drive
# through the closure, so for its type every road still reaches the destination.
CAR_ROUTES = {("a", "dest"): ["a", "m", "dest"], ("b", "dest"): ["b", "dest"]}
RESPONDER_ROUTES = {**CAR_ROUTES, ("dead", "dest"): ["dead", "closed", "dest"]}


class RouteTraci:
    """route_utils' view of SUMO: findRoute per vehicle type, from a table; each vehicle's own type."""

    TraCIException = FakeTraCIError

    def __init__(self):
        self.calls = []
        outer = self

        class Simulation:
            @staticmethod
            def findRoute(frm, to, vtype, depart, mode):
                outer.calls.append((frm, to, vtype))
                table = RESPONDER_ROUTES if vtype == "emergency_ambulance" else CAR_ROUTES
                return Found(list(table.get((frm, to), [])))

        class Vehicle:
            @staticmethod
            def getTypeID(v):
                return "emergency_ambulance" if v == "resp" else "normal_passenger"

        self.simulation, self.vehicle = Simulation(), Vehicle()


class E3Traci:
    """e3_core's view of SUMO for one decision: where the vehicle is, its route, and the traffic."""

    TraCIException = FakeTraCIError

    def __init__(self, cars):
        class vehicle:
            @staticmethod
            def getRoadID(v):
                return "here"

            @staticmethod
            def getRoute(v):
                return ("here", "a", "dest")

        class edge:
            @staticmethod
            def getLastStepVehicleNumber(e):
                return cars.get(e, 0)

        class lane:
            @staticmethod
            def getShape(lane_id):
                return []          # no geometry: the distance half is the neutral 0.5 for every candidate

        self.vehicle, self.edge, self.lane = vehicle, edge, lane


class Manager:
    blocked_edges = ()

    def get_active_emergency_zones(self):
        return ["404"]


@pytest.fixture
def route_fake(monkeypatch):
    fake = RouteTraci()
    monkeypatch.setattr(route_utils, "traci", fake)
    route_utils.set_blocked_edges(())
    route_utils.reset_route_stats()
    yield fake
    route_utils.set_blocked_edges(())
    route_utils.reset_route_stats()


def coordinator(monkeypatch, layer, outgoing, cars=None):
    """A real coordinator in the given arm. Only SUMO is faked; the scorer and the filter are real.
    Records what each decision offered the scorer, what it chose, and the emergency inputs it got."""
    monkeypatch.setattr(p3_runtime, "get_emergency_manager", lambda: Manager())
    monkeypatch.setattr(p3_runtime, "remaining_route", lambda v: ["here", "a", "dest"])
    c = p3_runtime.P3RuntimeCoordinator(emergency_layer=layer)
    monkeypatch.setattr(e3_core, "traci", E3Traci(cars or {}))
    monkeypatch.setattr(e3_core, "get_outgoing_edges", lambda edge, veh_id=None: list(outgoing))
    monkeypatch.setattr(e3_core, "build_destination_valid_route", lambda v, r, cur, sel: ["here", sel, "dest"])
    monkeypatch.setattr(e3_core, "set_route_checked", lambda v, new, old: True)
    monkeypatch.setattr(c.e3, "note_applied_route", lambda *a: None)
    c.seen = []
    real_select = c.e3.select_edge

    def select(v, edges, destination, messages, is_emergency_vehicle=False, corridor_edge=None):
        chosen, n = real_select(v, edges, destination, messages, is_emergency_vehicle=is_emergency_vehicle,
                                corridor_edge=corridor_edge)
        c.seen.append({"offered": list(edges), "chosen": chosen, "as_emergency": is_emergency_vehicle,
                       "corridor": corridor_edge})
        return chosen, n

    monkeypatch.setattr(c.e3, "select_edge", select)
    return c


def counters():
    s = route_utils.get_route_stats()
    return s["closure_filtered_decisions"], s["closure_candidates_removed"], s["closure_filter_fallbacks"]


# ------------------------------------------------------------------ the filter, in both arms

@ARMS
def test_a_dead_end_is_removed_in_both_arms(route_fake, monkeypatch, layer):
    route_utils.set_blocked_edges({"closed"})
    c = coordinator(monkeypatch, layer, ["a", "dead", "b"])
    c._apply_hybrid_decision("car", 30.0)
    assert c.seen[0]["offered"] == ["a", "b"]
    assert counters() == (1, 1, 0)


@ARMS
@pytest.mark.parametrize("closure_on, expected", [(False, "dead"), (True, "b")], ids=["no_closure", "closure"])
def test_a_dead_end_that_would_win_is_not_chosen_in_either_arm(route_fake, monkeypatch, layer, closure_on, expected):
    # "dead" is the emptiest road and has the most pheromone, so E3 picks it when it is offered.
    route_utils.set_blocked_edges({"closed"} if closure_on else ())
    c = coordinator(monkeypatch, layer, ["a", "dead", "b"], cars={"a": 6, "dead": 0, "b": 3})
    e3_core.pheromone.update({"a": 1.0, "dead": 5.0, "b": 1.0})
    c._apply_hybrid_decision("car", 30.0)
    assert c.seen[0]["chosen"] == expected


@ARMS
def test_every_reachable_candidate_is_kept_in_order(route_fake, monkeypatch, layer):
    route_utils.set_blocked_edges({"closed"})
    c = coordinator(monkeypatch, layer, ["b", "a"])
    c._apply_hybrid_decision("car", 30.0)
    assert c.seen[0]["offered"] == ["b", "a"]
    assert counters() == (0, 0, 0)
    assert sorted(frm for frm, _, _ in route_fake.calls) == ["a", "b"]      # both were checked, both kept


@ARMS
def test_if_no_candidate_reaches_the_destination_the_original_list_is_kept(route_fake, monkeypatch, layer):
    route_utils.set_blocked_edges({"closed"})
    c = coordinator(monkeypatch, layer, ["dead", "dead2"])
    c._apply_hybrid_decision("car", 30.0)
    assert c.seen[0]["offered"] == ["dead", "dead2"]
    assert counters() == (0, 0, 1)


@ARMS
def test_a_responder_is_checked_with_its_own_type_in_both_arms(route_fake, monkeypatch, layer):
    # It may drive through the closure, so for it "dead" is not a dead end, whichever arm it is in.
    route_utils.set_blocked_edges({"closed"})
    c = coordinator(monkeypatch, layer, ["a", "dead", "b"])
    c._active_responders = {"resp"}
    c._apply_hybrid_decision("resp", 30.0)
    assert c.seen[0]["offered"] == ["a", "dead", "b"]
    assert {vtype for _, _, vtype in route_fake.calls} == {"emergency_ambulance"}
    assert counters() == (0, 0, 0)


def test_both_arms_filter_the_same_decisions_the_same_way(route_fake, monkeypatch):
    """One sequence of decisions, run in each arm: the same candidates offered and the same counters, and
    for the ordinary car, which hears no messages, the same choice. (The responder's choice is left out:
    its corridor claim is gate 3, which is meant to differ.)"""
    route_utils.set_blocked_edges({"closed"})
    sequence = [("car", ["a", "dead", "b"]), ("car", ["b", "a"]), ("car", ["dead", "dead2"]),
                ("resp", ["a", "dead", "b"])]
    outcome = {}
    for layer in (True, False):
        rows = []
        for vehicle, outgoing in sequence:
            c = coordinator(monkeypatch, layer, outgoing, cars={"a": 6, "dead": 0, "b": 3})
            c._active_responders = {"resp"}
            e3_core.pheromone.update({"a": 1.0, "dead": 5.0, "b": 1.0})
            c._apply_hybrid_decision(vehicle, 30.0)
            rows.append((vehicle, c.seen[0]["offered"], c.seen[0]["chosen"] if vehicle == "car" else None,
                         counters()))
        outcome[layer] = rows
    assert outcome[True] == outcome[False]
    assert [r[3] for r in outcome[True]] == [(1, 1, 0), (0, 0, 0), (0, 0, 1), (0, 0, 0)]


# ------------------------------------------------------------------ the switch gates only the three mechanisms

PRODUCTION_DIRS = (REPO, REPO / "algorithms", REPO / "controller", REPO / "metrics")
SWITCH = {"emergency_layer", "emergency_layer_enabled"}
EXPECTED_READERS = {
    "p3_runtime.py": {
        "P3RuntimeCoordinator.__init__",                 # stores the switch and hands it to E3Core
        "P3RuntimeCoordinator._apply_yield_behaviour",   # gate 2, yielding
        "P3RuntimeCoordinator._apply_hybrid_decision",   # gate 3, the responder's corridor claim
    },
    "p3_hybrid_router.py": {"HybridRouter.protocol_weight_for_edge"},   # gate 1, the protocol term
    "algorithms/e3_core.py": {"E3Core.__init__"},                        # hands it to the router
}


def switch_readers() -> dict:
    """Every production function that reads the switch, by file. Storing or passing a constant does
    not count; reading the value does."""
    found = {}
    for folder in PRODUCTION_DIRS:
        for path in sorted(folder.glob("*.py")):
            readers = set()

            def visit(node, scope):
                for child in ast.iter_child_nodes(node):
                    if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                        visit(child, scope + [child.name])
                        continue
                    name = child.id if isinstance(child, ast.Name) else \
                        child.attr if isinstance(child, ast.Attribute) else None
                    if name in SWITCH and isinstance(child.ctx, ast.Load):
                        readers.add(".".join(scope) or "<module>")
                    visit(child, scope)

            visit(ast.parse(path.read_text(encoding="utf-8")), [])
            if readers:
                found[path.relative_to(REPO).as_posix()] = readers
    return found


def test_the_switch_is_read_only_at_the_three_emergency_gates():
    """If anything else ever reads the switch, the ablation no longer isolates the emergency layer.
    This fails so that the change is reviewed. The filter's module in particular must not read it."""
    readers = switch_readers()
    assert "algorithms/route_utils.py" not in readers
    assert readers == EXPECTED_READERS


@pytest.mark.parametrize("responder", [False, True], ids=["ordinary_receiver", "responder_receiver"])
def test_gate_1_with_the_layer_off_nothing_moves_the_protocol_term(responder):
    heard = [ProtocolMessage(vehicle_id="resp_b", emergency_flag=True, priority_level=4, next_edge="x"),
             ProtocolMessage(vehicle_id="car", priority_level=1, next_edge="y"),
             ProtocolMessage(vehicle_id="yielder", yield_request=True, priority_level=1, next_edge="z")]
    edges = ("x", "y", "z", "corridor", "silent")
    on, off = HybridRouter(), HybridRouter(emergency_layer_enabled=False)

    def weights(router):
        return [router.protocol_weight_for_edge(e, heard, is_emergency_vehicle=responder, corridor_edge="corridor")
                for e in edges]

    assert weights(off) == [NEUTRAL_WEIGHT] * len(edges)
    expected_on = ([1.0, 0.6, NEUTRAL_WEIGHT, RESPONDER_CORRIDOR_WEIGHT, NEUTRAL_WEIGHT] if responder else
                   [ORDINARY_AVOID_EMERGENCY_WEIGHT, 0.6, NEUTRAL_WEIGHT, NEUTRAL_WEIGHT, NEUTRAL_WEIGHT])
    assert weights(on) == pytest.approx(expected_on)


class YieldTraci:
    """A responder on two-lane edge E with one moving car 30 m ahead of it in its lane (lane 1), inside the yield
    radius: under the unified rule (2026-09-15) the car can make room by moving right."""

    TraCIException = FakeTraCIError

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
                return 1

            @staticmethod
            def getLaneID(v):
                return "E_1"

            @staticmethod
            def getVehicleClass(v):
                return "emergency" if v == "resp" else "passenger"

            @staticmethod
            def getLeader(v, dist):
                return ("car", 22.5) if v == "resp" else None

            @staticmethod
            def getBestLanes(v):
                return (("E_0", 100.0, 0.0, 0, True, ()), ("E_1", 100.0, 0.0, 0, True, ()))

            @staticmethod
            def changeLane(v, lane, duration):
                outer.lane_changes.append((v, lane))

        class Edge:
            @staticmethod
            def getLastStepVehicleIDs(e):
                return ["resp", "car"] if e == "E" else []

            @staticmethod
            def getLaneNumber(e):
                return 2

        class Lane:
            @staticmethod
            def getAllowed(lane_id):
                return ()

            @staticmethod
            def getDisallowed(lane_id):
                return ()

        self.vehicle, self.edge, self.lane = Vehicle(), Edge(), Lane()


@ARMS
def test_gate_2_only_the_on_arm_yields(monkeypatch, layer):
    fake = YieldTraci()
    monkeypatch.setattr(p3_runtime, "traci", fake)
    c = p3_runtime.P3RuntimeCoordinator(emergency_layer=layer)
    c._active_responders = {"resp"}
    c.protocol_state.update(5.0, build_emergency_message(
        vehicle_id="resp", current_time=5.0, vehicle_type="emergency_ambulance", position_edge="E",
        speed=12.0, route_intent="F", next_edge="F"))
    c._apply_yield_behaviour(5.0)
    assert fake.slowed == []                            # the unified rule never brakes, in either arm
    assert fake.lane_changes == ([("car", 0)] if layer else [])
    # No yield in the OFF arm, so no car there is asked, and none raises the yield flag (read from last_yield).
    assert c.last_yield == ({"car": 5.0} if layer else {})
    assert c.last_asked_to_yield == ({"car": 5.0} if layer else {})


@ARMS
def test_gate_3_only_the_on_arm_scores_a_responder_as_one(route_fake, monkeypatch, layer):
    route_utils.set_blocked_edges({"closed"})
    c = coordinator(monkeypatch, layer, ["a", "b"])
    c._active_responders = {"resp"}
    c._apply_hybrid_decision("resp", 30.0)
    assert (c.seen[0]["as_emergency"], c.seen[0]["corridor"]) == ((True, "a") if layer else (False, None))
