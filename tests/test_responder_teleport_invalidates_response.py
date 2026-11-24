"""Regression tests for audit finding M12: a teleport must not manufacture a response.

SUMO teleports a vehicle that has waited too long in a jam and reinserts it further along
its own route. On 2026-09-08 responder `emergency_dispatch_401` was teleported directly
onto incident 401's own centre edge:

    Warning: Teleporting vehicle 'emergency_dispatch_401'; waited too long (jam),
             lane='190128487#2_0', time=2260.00.
    Warning: Vehicle 'emergency_dispatch_401' ends teleporting on edge '734630276',
             time=2260.00.

The arrival test in contract decision 13.3 is "responder road ID equals center_edge", so
that registered as an arrival and reported 1,361 s for a trip that was never driven.

The rule these tests hold in place: a response reached after the responder teleported is
recorded and reported, but it is not a measurement. `response_time_s` is None for it, so
nothing that filters on that field can average an artifact.
"""

import pytest

import emergency_manager
from emergency_manager import EmergencyManager


class FakeVehicleDomain:
    def __init__(self, positions):
        self.positions = positions

    def getIDList(self):
        return tuple(self.positions)

    def getRoadID(self, vehicle_id):
        return self.positions[vehicle_id]

    def getVehicleClass(self, vehicle_id):
        return "emergency"


class FakeSimulationDomain:
    def __init__(self, teleporting):
        self.teleporting = teleporting

    def getStartingTeleportIDList(self):
        return tuple(self.teleporting)


class FakeVehicleTypeDomain:
    """No vType loaded: the PD-1 question 2 class switch has nothing to do here."""

    def getIDList(self):
        return ()


class FakeTraci:
    class TraCIException(Exception):
        pass

    def __init__(self, positions, teleporting=()):
        self.vehicle = FakeVehicleDomain(positions)
        self.simulation = FakeSimulationDomain(teleporting)
        self.vehicletype = FakeVehicleTypeDomain()


@pytest.fixture
def patched_traci(monkeypatch):
    def install(positions, teleporting=()):
        fake = FakeTraci(positions, teleporting)
        monkeypatch.setattr(emergency_manager, "traci", fake)
        return fake
    return install


def build_manager():
    manager = EmergencyManager()
    manager.set_planned_events([{
        "event_id": 401,
        "start_time": 900,
        "end_time": 1230,
        "incident_type": "accident",
        "center_edge": "E_centre",
        "affected_edges": ["E_centre"],
        "dispatch": {"vehicle_id": "emergency_dispatch_401", "vehicle_type": "ambulance"},
    }])
    manager.generate_emergencies()
    return manager


def test_a_clean_arrival_is_still_measured(patched_traci):
    """The control case: no teleport, so nothing changes from before M12."""
    patched_traci({"emergency_dispatch_401": "E_centre"})
    manager = build_manager()

    assert manager.record_responder_arrivals(1488.0) == {401: 588.0}
    state = manager.get_response_summary()[401]
    assert state["response_valid"] is True
    assert state["response_time_s"] == 588.0
    assert state["responder_teleported"] is False
    assert manager.get_teleport_summary()["responder_teleport_count"] == 0


def test_a_teleported_responder_reports_no_response_time(patched_traci):
    """The 2026-09-08 case: teleported onto the centre edge, arrival on the same step."""
    patched_traci({"emergency_dispatch_401": "E_centre"})
    manager = build_manager()

    manager.note_teleports(2260.0, ["emergency_dispatch_401"])
    assert manager.record_responder_arrivals(2261.0) == {}

    state = manager.get_response_summary()[401]
    assert state["response_time_s"] is None
    assert state["response_valid"] is False
    assert state["responder_teleported"] is True
    assert state["invalid_reason"] == "responder_teleported"
    # Recorded, not silently dropped: the arrival and the number it would have produced
    # are both still visible.
    assert state["measured"] is True
    assert state["unvalidated_response_time_s"] == 1361.0
    assert state["responder_teleport_times_s"] == [2260.0]


def test_update_polls_sumo_so_a_same_step_teleport_is_caught(patched_traci):
    """A teleport can start and end inside one step, so the poll must precede the test."""
    patched_traci({"emergency_dispatch_401": "E_centre"}, teleporting=["emergency_dispatch_401"])
    manager = build_manager()

    manager.update(2260.0)
    assert manager.record_responder_arrivals(2260.0) == {}
    assert manager.get_response_summary()[401]["response_valid"] is False


def test_a_teleport_of_another_vehicle_changes_nothing(patched_traci):
    """Ordinary traffic teleports constantly; only the dispatched responder matters."""
    patched_traci({"emergency_dispatch_401": "E_centre"}, teleporting=["veh_123", "ambient_7"])
    manager = build_manager()

    # t=800 is outside the incident window, so update() polls teleports without trying to
    # close lanes through a traci stub that has no edge domain.
    manager.update(800.0)
    assert manager.record_responder_arrivals(1488.0) == {401: 588.0}
    assert manager.get_response_summary()[401]["response_valid"] is True
    assert manager.get_teleport_summary()["responder_teleport_count"] == 0


def test_a_teleport_after_arrival_is_counted_but_does_not_invalidate(patched_traci):
    """It cannot have manufactured an arrival that was already recorded."""
    patched_traci({"emergency_dispatch_401": "E_centre"})
    manager = build_manager()

    assert manager.record_responder_arrivals(1488.0) == {401: 588.0}
    manager.note_teleports(1600.0, ["emergency_dispatch_401"])

    state = manager.get_response_summary()[401]
    assert state["response_time_s"] == 588.0
    assert state["response_valid"] is True
    summary = manager.get_teleport_summary()
    assert summary["responder_teleport_count"] == 1
    assert summary["events_invalidated_by_teleport"] == []


def test_the_teleport_summary_names_the_invalidated_events(patched_traci):
    patched_traci({})
    manager = build_manager()

    manager.note_teleports(2260.0, ["emergency_dispatch_401"])
    summary = manager.get_teleport_summary()
    assert summary["events_invalidated_by_teleport"] == [401]
    assert summary["responder_teleports"] == [{
        "event_id": 401,
        "vehicle_id": "emergency_dispatch_401",
        "sim_time": 2260.0,
        "before_arrival": True,
    }]


def test_poll_survives_a_traci_without_a_simulation_domain(monkeypatch):
    """Runners and tests that stub traci must not crash on the new poll."""
    class BareTraci:
        class TraCIException(Exception):
            pass

    monkeypatch.setattr(emergency_manager, "traci", BareTraci())
    manager = build_manager()
    assert manager.poll_teleports(100.0) == []


def test_a_new_run_does_not_inherit_the_previous_runs_teleports(patched_traci):
    """2026-09-13: responder_teleports and incident_errors were never reset between runs.

    Two runs in one process - scripts/16 runs baseline then P3, and a tuning study runs every
    trial - used to report run 1's teleport in run 2's summary. Every runner and tuning trial
    starts by loading the scenario (set_planned_events), which is the per-run reset.
    """
    patched_traci({})
    manager = build_manager()
    manager.note_teleports(2260.0, ["emergency_dispatch_401"])      # run 1: the responder teleports
    manager.incident_errors.append("event 401 lane x: simulated failure")
    assert manager.get_teleport_summary()["responder_teleport_count"] == 1

    manager.load_canonical_scenario_file()                            # run 2 starts
    manager.generate_emergencies()
    summary = manager.get_teleport_summary()
    assert summary["responder_teleport_count"] == 0
    assert summary["responder_teleports"] == []
    assert summary["events_invalidated_by_teleport"] == []
    assert manager.incident_errors == []

    # ... and run 2 still records its own teleports normally.
    responder = next(e.dispatch_vehicle_id for e in manager.events.values() if e.dispatch_vehicle_id)
    manager.note_teleports(3000.0, [responder])
    assert manager.get_teleport_summary()["responder_teleport_count"] == 1
