"""Regression tests for the emergency response-time definition.

The bug these guard against (audit finding M9): response detection used to run only
while an incident was still active, so a responder that reached the incident after the
closure window closed was silently recorded as "no response". In the 4,000-vehicle
scenario every one of the three responders arrives after its own window closes, so that
gate deleted the thesis's entire headline metric rather than part of it.

Contract sections 7 and 8 put no window condition on the measurement: response time is
incident activation until the incident's own dispatched responder reaches the centre
edge, however long that takes.
"""

import pytest

import emergency_manager
from emergency_manager import EmergencyManager


class FakeVehicleDomain:
    def __init__(self, positions, classes=None):
        self.positions = positions
        self.classes = classes or {}

    def getIDList(self):
        return tuple(self.positions)

    def getRoadID(self, vehicle_id):
        return self.positions[vehicle_id]

    def getVehicleClass(self, vehicle_id):
        return self.classes.get(vehicle_id, "passenger")


class FakeTraci:
    class TraCIException(Exception):
        pass

    def __init__(self, positions, classes=None):
        self.vehicle = FakeVehicleDomain(positions, classes)


@pytest.fixture
def patched_traci(monkeypatch):
    def install(positions, classes=None):
        fake = FakeTraci(positions, classes)
        monkeypatch.setattr(emergency_manager, "traci", fake)
        return fake
    return install


def build_manager(**overrides):
    event = {
        "event_id": 401,
        "start_time": 900,
        "end_time": 1230,
        "incident_type": "accident",
        "center_edge": "E_centre",
        "affected_edges": ["E_centre"],
        "dispatch": {"vehicle_id": "emergency_dispatch_401", "vehicle_type": "ambulance"},
    }
    event.update(overrides)
    manager = EmergencyManager()
    manager.set_planned_events([event])
    manager.generate_emergencies()
    return manager


def test_dispatch_vehicle_id_is_read_from_the_scenario(patched_traci):
    patched_traci({})
    manager = build_manager()
    assert manager.events[401].dispatch_vehicle_id == "emergency_dispatch_401"


def test_arrival_after_incident_end_is_still_measured(patched_traci):
    """The whole point of the fix: t=1488 is 258 s past the window, and still counts."""
    patched_traci({"emergency_dispatch_401": "E_centre"})
    manager = build_manager()

    recorded = manager.record_responder_arrivals(1488.0)

    assert recorded == {401: 588.0}
    state = manager.get_response_summary()[401]
    assert state["measured"] is True
    assert state["response_time_s"] == 588.0
    assert state["arrived_after_incident_end"] is True
    assert state["first_arrival_vehicle"] == "emergency_dispatch_401"


def test_arrival_inside_the_window_is_not_flagged(patched_traci):
    patched_traci({"emergency_dispatch_401": "E_centre"})
    manager = build_manager()

    manager.record_responder_arrivals(1000.0)

    state = manager.get_response_summary()[401]
    assert state["response_time_s"] == 100.0
    assert state["arrived_after_incident_end"] is False


def test_response_is_not_attributed_to_another_emergency_vehicle(patched_traci):
    """Contract section 7: only the incident's own dispatch vehicle may supply its time."""
    patched_traci(
        {"ambient_emergency_07": "E_centre", "emergency_dispatch_401": "E_far_away"},
        classes={"ambient_emergency_07": "emergency", "emergency_dispatch_401": "emergency"},
    )
    manager = build_manager()

    assert manager.record_responder_arrivals(950.0) == {}
    assert manager.get_response_summary()[401]["measured"] is False


def test_arrival_before_incident_start_is_ignored(patched_traci):
    """A negative response time is physically meaningless; it must not be recorded."""
    patched_traci({"emergency_dispatch_401": "E_centre"})
    manager = build_manager()

    assert manager.record_responder_arrivals(800.0) == {}
    assert manager.get_response_summary()[401]["measured"] is False


def test_only_the_first_arrival_counts(patched_traci):
    patched_traci({"emergency_dispatch_401": "E_centre"})
    manager = build_manager()

    assert manager.record_responder_arrivals(1000.0) == {401: 100.0}
    assert manager.record_responder_arrivals(1100.0) == {}
    assert manager.get_response_summary()[401]["response_time_s"] == 100.0


def test_responder_finishing_in_the_same_step_is_still_caught(patched_traci):
    """The route's last edge is the centre edge, so the vehicle can leave the network
    in the step it reaches the incident and never appear in getIDList()."""
    patched_traci({})
    manager = build_manager()

    recorded = manager.record_responder_arrivals(
        1488.0, vehicle_ids=[], arrived_ids=["emergency_dispatch_401"]
    )

    assert recorded == {401: 588.0}
    assert manager.get_response_summary()[401]["arrival_edge"] == "E_centre"


def test_events_without_a_named_dispatch_fall_back_to_any_emergency_vehicle(patched_traci):
    """Randomly generated events have no dispatch block and must still be measurable."""
    patched_traci(
        {"ambient_emergency_07": "E_centre"},
        classes={"ambient_emergency_07": "emergency"},
    )
    manager = build_manager(dispatch=None)

    assert manager.events[401].dispatch_vehicle_id is None
    assert manager.record_responder_arrivals(950.0) == {401: 50.0}
