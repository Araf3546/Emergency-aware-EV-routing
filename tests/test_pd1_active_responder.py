"""PD-1, decided by the supervisor on 2026-09-13: "the vehicle class alone must not trigger
emergency routing".

The emergency manager keeps an explicit flag - `active_responder_ids` - holding only a vehicle
dispatched to an incident that has started and that it has not yet reached. The reroute
trigger's every-step clause reads that flag, never the vehicle type. These tests pin:

* the flag's lifecycle: empty before the incident, set from its start, cleared once the
  responder's arrival is recorded, never containing an ambient emergency vehicle;
* that it is reset for every new run (so it cannot leak between runs or tuning trials);
* that `should_reroute` fires off the 30 s clock for an active responder on a closed edge and
  for nobody else - in particular not for an emergency-class vehicle that is not responding.
"""

import pytest

import emergency_manager
from controller import reroute_logic
from emergency_manager import EmergencyManager

RESPONDER = "emergency_dispatch_401"
AMBIENT = "emergency_00062"          # an ambient emergency-class vehicle, as in the real runs
ORDINARY = "normal_00001"


class _Vehicles:
    def __init__(self, positions):
        self.positions = positions

    def getIDList(self):
        return tuple(self.positions)

    def getRoadID(self, vehicle_id):
        return self.positions[vehicle_id]

    def getTypeID(self, vehicle_id):
        # Every vehicle here except the ordinary car is emergency-class, so a clause that
        # still read the type would fire for the ambient one.
        return "normal_passenger" if vehicle_id == ORDINARY else "ambulance"

    def getVehicleClass(self, vehicle_id):
        return "passenger" if vehicle_id == ORDINARY else "emergency"


class _Simulation:
    def __init__(self, time=0.0):
        self.time = time

    def getTime(self):
        return self.time

    def getStartingTeleportIDList(self):
        return ()


class _Edge:
    def __init__(self, speed=10.0, count=0):
        self.speed, self.count = speed, count

    def getLastStepMeanSpeed(self, edge_id):
        return self.speed

    def getLastStepVehicleNumber(self, edge_id):
        return self.count


class _VehicleTypes:
    """No vType loaded: the PD-1 question 2 class switch has nothing to do here."""

    def getIDList(self):
        return ()


class FakeTraci:
    class TraCIException(Exception):
        pass

    def __init__(self, positions, time=0.0, edge_speed=10.0, edge_count=0):
        self.vehicle = _Vehicles(positions)
        self.simulation = _Simulation(time)
        self.edge = _Edge(edge_speed, edge_count)
        self.vehicletype = _VehicleTypes()


def build_manager(monkeypatch, positions):
    monkeypatch.setattr(emergency_manager, "traci", FakeTraci(positions))
    manager = EmergencyManager()
    # Lane closure is SUMO's business; the flag logic does not depend on it.
    monkeypatch.setattr(manager, "_apply_incident", lambda event: 0)
    monkeypatch.setattr(manager, "_restore_incident", lambda event: 0)
    manager.set_planned_events([{
        "event_id": 401,
        "start_time": 900,
        "end_time": 1230,
        "incident_type": "accident",
        "center_edge": "E_centre",
        "affected_edges": ["E_centre"],
        "dispatch": {"vehicle_id": RESPONDER, "vehicle_type": "ambulance"},
    }])
    manager.generate_emergencies()
    return manager


def test_flag_follows_the_incident_not_the_vehicle_class(monkeypatch):
    positions = {RESPONDER: "E_approach", AMBIENT: "E_centre", ORDINARY: "E_other"}
    manager = build_manager(monkeypatch, positions)

    manager.update(899)
    assert manager.active_responder_ids == frozenset()          # incident not started

    manager.update(900)
    assert manager.active_responder_ids == {RESPONDER}
    assert manager.is_active_responder(RESPONDER)
    assert not manager.is_active_responder(AMBIENT)             # emergency class, not responding
    assert not manager.is_active_responder(ORDINARY)

    # The responder reaches the centre edge: its response is over (decision 13.3).
    positions[RESPONDER] = "E_centre"
    manager.record_responder_arrivals(950, vehicle_ids=tuple(positions), arrived_ids=())
    manager.update(951)
    assert manager.active_responder_ids == frozenset()


def test_flag_is_reset_for_every_new_run(monkeypatch):
    manager = build_manager(monkeypatch, {RESPONDER: "E_approach"})
    manager.update(900)
    assert manager.active_responder_ids == {RESPONDER}

    manager.load_canonical_scenario_file()       # what every runner and tuning trial does first
    assert manager.active_responder_ids == frozenset()


class _Manager:
    def __init__(self, responders, blocked):
        self.responders, self.blocked = set(responders), set(blocked)

    def is_active_responder(self, vehicle_id):
        return vehicle_id in self.responders

    def is_edge_blocked(self, edge_id):
        return edge_id in self.blocked


@pytest.fixture
def trigger(monkeypatch):
    def install(positions, time, responders=(), blocked=("E_centre",), edge_speed=10.0, edge_count=0):
        monkeypatch.setattr(reroute_logic, "traci", FakeTraci(positions, time, edge_speed, edge_count))
        monkeypatch.setattr(reroute_logic, "get_emergency_manager",
                            lambda: _Manager(responders, blocked))
        return reroute_logic.should_reroute
    return install


def test_ambient_emergency_vehicle_on_closed_edge_is_not_rerouted_off_clock(trigger):
    should_reroute = trigger({AMBIENT: "E_centre"}, time=947)
    assert should_reroute(AMBIENT) is False      # fired every step before PD-1


def test_active_responder_on_closed_edge_is_rerouted_off_clock(trigger):
    should_reroute = trigger({RESPONDER: "E_centre"}, time=947, responders=(RESPONDER,))
    assert should_reroute(RESPONDER) is True


def test_responder_on_open_edge_waits_for_the_clock(trigger):
    should_reroute = trigger({RESPONDER: "E_open"}, time=947, responders=(RESPONDER,))
    assert should_reroute(RESPONDER) is False


def test_ordinary_congestion_rule_is_unchanged(trigger):
    # On the 30 s clock, a congested edge triggers every vehicle alike.
    should_reroute = trigger({ORDINARY: "E_other", AMBIENT: "E_other"}, time=960, edge_speed=1.0)
    assert should_reroute(ORDINARY) is True
    assert should_reroute(AMBIENT) is True
    # Off the clock, nothing - for an ordinary car and an ambient emergency vehicle alike.
    should_reroute = trigger({ORDINARY: "E_centre", AMBIENT: "E_centre"}, time=961, edge_speed=1.0)
    assert should_reroute(ORDINARY) is False
    assert should_reroute(AMBIENT) is False
