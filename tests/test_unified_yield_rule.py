"""The unified yield rule (user decision 2026-09-15): make room if you can, never brake in the responder's lane.

- A single-lane road (fewer than two lanes the car's class may use): no braking and no lane change.
- A road with two or more such lanes: a car is asked to move right only if it is in the responder's lane path
  ahead of it and a usable lane lies to its right. It goes to the rightmost usable lane, for
  YIELD_DURATION_S. Every other car keeps driving.
- No car is ever braked, not even to open a gap for a lane change.
- Every response starts the 5 s cooldown, so who is asked and when is unchanged.
- `yield_request` is raised only by an actual move-aside, and only that counts as a yield action.

Evidence: results/make_room_yield_2026-09-15/README.md.
"""

import pytest

import p3_runtime
from p3_protocol import build_emergency_message

ALL = ((), ())                     # a lane every class may use: (allowed, disallowed)
BUS_ONLY = (("bus",), ())


class FakeTraCIError(Exception):
    """Distinct from Exception, so a real error inside the code under test is not swallowed as a TraCI one."""


class World:
    """A responder `resp` and one car `car` 30 m ahead of it. Roads have configurable lanes. Every command sent to
    SUMO is recorded; a car is never expected to be slowed."""

    TraCIException = FakeTraCIError

    def __init__(self, lanes, car=("E", 1, 40.0), resp=("E", 1, 10.0), leader=None, continuation=(),
                 car_class="passenger"):
        self.lanes = lanes
        self.where = {"car": car, "resp": resp}
        self.slowed, self.lane_changes = [], []
        w = self

        class vehicle:
            @staticmethod
            def getIDList():
                return ["resp", "car"]

            @staticmethod
            def getPosition(v):
                return {"resp": (0.0, 0.0), "car": (30.0, 0.0)}[v]

            @staticmethod
            def getRoadID(v):
                return w.where[v][0]

            @staticmethod
            def getLaneIndex(v):
                return w.where[v][1]

            @staticmethod
            def getLaneID(v):
                return f"{w.where[v][0]}_{w.where[v][1]}"

            @staticmethod
            def getLanePosition(v):
                return w.where[v][2]

            @staticmethod
            def getSpeed(v):
                return 12.0

            @staticmethod
            def getVehicleClass(v):
                return "emergency" if v == "resp" else car_class

            @staticmethod
            def getTypeID(v):
                return "emergency_ambulance" if v == "resp" else "normal_passenger"

            @staticmethod
            def getLeader(v, dist):
                return (leader, 5.0) if v == "resp" and leader else None

            @staticmethod
            def getBestLanes(v):
                edge, index, _ = w.where[v]
                return tuple((f"{edge}_{i}", 100.0, 0.0, 0, True, tuple(continuation) if i == index else ())
                             for i in range(len(w.lanes[edge])))

            @staticmethod
            def slowDown(v, speed, duration):
                w.slowed.append(v)

            @staticmethod
            def changeLane(v, index, duration):
                w.lane_changes.append((v, index, duration))

        class edge:
            @staticmethod
            def getLastStepVehicleIDs(e):
                return [v for v in ("resp", "car") if w.where[v][0] == e]

            @staticmethod
            def getLaneNumber(e):
                return len(w.lanes[e])

            @staticmethod
            def getLastStepVehicleNumber(e):
                return 1

            @staticmethod
            def getLastStepMeanSpeed(e):
                return 5.0

        class lane:
            @staticmethod
            def getAllowed(lane_id):
                e, i = lane_id.rsplit("_", 1)
                return w.lanes[e][int(i)][0]

            @staticmethod
            def getDisallowed(lane_id):
                e, i = lane_id.rsplit("_", 1)
                return w.lanes[e][int(i)][1]

            @staticmethod
            def getMaxSpeed(lane_id):
                return 13.89

        self.vehicle, self.edge, self.lane = vehicle, edge, lane


class NoIncident:
    blocked_edges = ()

    def get_nearest_emergency(self, vehicle_id):
        return None


def coordinator(monkeypatch, world, layer=True):
    monkeypatch.setattr(p3_runtime, "traci", world)
    monkeypatch.setattr(p3_runtime, "get_emergency_manager", lambda: NoIncident())
    monkeypatch.setattr(p3_runtime, "remaining_route", lambda v: ["E", "F", "dest"])
    c = p3_runtime.P3RuntimeCoordinator(emergency_layer=layer)
    c._active_responders = {"resp"}
    return c


def hear_responder(c, world, t):
    """The responder broadcasts at `t`, announcing F as the road it enters next."""
    c.protocol_state.update(t, build_emergency_message(
        vehicle_id="resp", current_time=t, vehicle_type="emergency_ambulance", position_edge=world.where["resp"][0],
        speed=12.0, route_intent="F", next_edge="F"))


def yield_at(monkeypatch, world, t=5.0, layer=True):
    c = coordinator(monkeypatch, world, layer)
    hear_responder(c, world, t)
    c._apply_yield_behaviour(t)
    return c


def flag_after(c, t):
    """What the car's own next broadcast says: (yield_request, priority)."""
    c._broadcast_vehicle_messages(t)
    m = c.protocol_state.neighbors["car"].message
    return m.yield_request, m.priority_level


def assert_kept_driving(c, world):
    assert world.slowed == [] and world.lane_changes == []
    assert c.last_asked_to_yield == {"car": 5.0}          # asked, so the 5 s cooldown runs
    assert c.last_yield == {} and c.yield_actions == 0 and c.metrics.yield_actions_performed == 0
    assert flag_after(c, 6.0) == (False, 0)               # it is not yielding, so it does not say so


def assert_moved_right(c, world, lane):
    assert world.slowed == []                              # never braked, not even to open a gap
    assert world.lane_changes == [("car", lane, p3_runtime.YIELD_DURATION_S)]
    assert c.last_asked_to_yield == {"car": 5.0} and c.last_yield == {"car": 5.0}
    assert c.yield_actions == 1 and c.metrics.yield_actions_performed == 1
    assert flag_after(c, 6.0) == (True, 1)


# ------------------------------------------------------------------ single-lane roads

def test_on_a_single_lane_road_the_car_neither_brakes_nor_moves_and_says_nothing(monkeypatch):
    world = World({"E": [ALL]}, car=("E", 0, 40.0), resp=("E", 0, 10.0))
    assert_kept_driving(yield_at(monkeypatch, world), world)


def test_a_road_whose_other_lane_the_car_may_not_use_is_single_lane_for_it(monkeypatch):
    # Lane 0 is a bus lane: the car has nowhere to go, so it is not sent into the bus lane either.
    world = World({"E": [BUS_ONLY, ALL]}, car=("E", 1, 40.0), resp=("E", 1, 10.0))
    assert_kept_driving(yield_at(monkeypatch, world), world)


def test_a_lane_a_closure_disallows_does_not_count(monkeypatch):
    closed = ((), ("passenger", "authority"))
    world = World({"E": [closed, ALL]}, car=("E", 1, 40.0), resp=("E", 1, 10.0))
    assert_kept_driving(yield_at(monkeypatch, world), world)


# ------------------------------------------------------------------ multi-lane roads

def test_a_car_ahead_in_the_responders_lane_moves_right_without_braking(monkeypatch):
    world = World({"E": [ALL, ALL]}, car=("E", 1, 40.0), resp=("E", 1, 10.0))
    assert_moved_right(yield_at(monkeypatch, world), world, lane=0)


def test_the_responders_immediate_leader_is_in_its_path(monkeypatch):
    # The car is on the road the responder enters next, and is the vehicle SUMO puts directly ahead of it.
    world = World({"E": [ALL], "F": [ALL, ALL]}, car=("F", 1, 5.0), resp=("E", 0, 10.0), leader="car")
    assert_moved_right(yield_at(monkeypatch, world), world, lane=0)


def test_a_car_in_the_lane_the_responder_continues_into_is_in_its_path(monkeypatch):
    world = World({"E": [ALL], "F": [ALL, ALL]}, car=("F", 1, 5.0), resp=("E", 0, 10.0), continuation=("F_1",))
    assert_moved_right(yield_at(monkeypatch, world), world, lane=0)


def test_the_target_is_the_rightmost_lane_the_car_may_use(monkeypatch):
    world = World({"E": [BUS_ONLY, ALL, ALL]}, car=("E", 2, 40.0), resp=("E", 2, 10.0))
    assert_moved_right(yield_at(monkeypatch, world), world, lane=1)


def test_a_car_in_another_lane_is_not_in_the_way_and_keeps_driving(monkeypatch):
    # The old rule moved it to lane 0, which here is the responder's own lane.
    world = World({"E": [ALL, ALL]}, car=("E", 1, 40.0), resp=("E", 0, 10.0))
    assert_kept_driving(yield_at(monkeypatch, world), world)


def test_a_car_on_the_next_road_but_not_in_the_responders_continuation_keeps_driving(monkeypatch):
    world = World({"E": [ALL], "F": [ALL, ALL]}, car=("F", 0, 5.0), resp=("E", 0, 10.0), continuation=("F_1",))
    assert_kept_driving(yield_at(monkeypatch, world), world)


def test_a_car_in_the_path_already_in_the_rightmost_lane_keeps_driving(monkeypatch):
    world = World({"E": [ALL, ALL]}, car=("E", 0, 40.0), resp=("E", 0, 10.0))
    assert_kept_driving(yield_at(monkeypatch, world), world)


# ------------------------------------------------------------------ what the method reports

@pytest.mark.parametrize("lanes, car, resp, leader, continuation, expected", [
    ({"E": [ALL]}, ("E", 0, 40.0), ("E", 0, 10.0), None, (), "keep_driving_single_lane"),
    ({"E": [ALL, ALL]}, ("E", 1, 40.0), ("E", 0, 10.0), None, (), "keep_driving_not_in_lane"),
    ({"E": [ALL, ALL]}, ("E", 0, 40.0), ("E", 0, 10.0), None, (), "keep_driving_cannot_move"),
    ({"E": [ALL, ALL]}, ("E", 1, 40.0), ("E", 1, 10.0), None, (), "move_right"),
])
def test_the_method_returns_what_the_car_did(monkeypatch, lanes, car, resp, leader, continuation, expected):
    world = World(lanes, car=car, resp=resp, leader=leader, continuation=continuation)
    c = coordinator(monkeypatch, world)
    assert c._yield_to_emergency("car", car[0], "resp", 5.0) == expected
    assert world.slowed == []


# ------------------------------------------------------------------ the trigger's timing is unchanged

@pytest.mark.parametrize("lanes, car, resp", [
    ({"E": [ALL]}, ("E", 0, 40.0), ("E", 0, 10.0)),              # keeps driving
    ({"E": [ALL, ALL]}, ("E", 1, 40.0), ("E", 1, 10.0)),         # moves right
], ids=["keep_driving", "move_right"])
def test_every_response_starts_the_cooldown(monkeypatch, lanes, car, resp):
    world = World(lanes, car=car, resp=resp)
    c = coordinator(monkeypatch, world)
    asked, real = [], c._yield_to_emergency

    def counting(vehicle_id, edge_id, responder_id, sim_time):
        asked.append(sim_time)
        return real(vehicle_id, edge_id, responder_id, sim_time)

    monkeypatch.setattr(c, "_yield_to_emergency", counting)
    for t in (5.0, 6.0, 9.0, 10.0, 11.0):
        world.where["car"] = car                                  # a requested move is not simulated here
        hear_responder(c, world, t)
        c._apply_yield_behaviour(t)
    assert asked == [5.0, 10.0]                                   # YIELD_COOLDOWN_S = 5 s, as before


def test_the_flag_lasts_the_yield_duration_after_a_move_and_then_clears(monkeypatch):
    world = World({"E": [ALL, ALL]}, car=("E", 1, 40.0), resp=("E", 1, 10.0))
    c = yield_at(monkeypatch, world)
    assert flag_after(c, 8.0) == (True, 1)                        # 3 s after the move
    assert flag_after(c, 10.0) == (False, 0)


def test_the_responder_itself_is_never_asked(monkeypatch):
    world = World({"E": [ALL, ALL]}, car=("E", 1, 40.0), resp=("E", 1, 10.0))
    c = yield_at(monkeypatch, world)
    assert "resp" not in c.last_asked_to_yield and "resp" not in c.last_yield


def test_with_the_emergency_layer_off_nobody_is_asked(monkeypatch):
    world = World({"E": [ALL, ALL]}, car=("E", 1, 40.0), resp=("E", 1, 10.0))
    c = yield_at(monkeypatch, world, layer=False)
    assert world.slowed == [] and world.lane_changes == []
    assert c.last_asked_to_yield == {} and c.last_yield == {} and c.yield_actions == 0
    assert flag_after(c, 6.0) == (False, 0)


def test_the_removed_braking_constants_are_gone():
    # The brake-to-40% action of 2026-09-07 is gone. Its two constants must not linger as dead parameters.
    assert not hasattr(p3_runtime, "YIELD_MIN_SPEED_MS") and not hasattr(p3_runtime, "YIELD_SPEED_FACTOR")
