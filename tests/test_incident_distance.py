"""The distance from an ordinary car to an active incident must be a real road distance.

Defect found 2026-09-11. While an incident is active its centre edge is closed to every
vehicle class except emergency. SUMO's `findRoute` from an ordinary car to that edge then
returns an EMPTY route with length 0.0 instead of raising, and `_route_distance` read that as
"0 metres away". Every ordinary car in the network reported itself inside the 200 m
threshold for the whole incident: 124,472 of 132,285 queries in a full run returned exactly 0.

The fix has two parts, both pinned here:

* an empty route means "no route", i.e. infinitely far, as `_route_distance` always promised;
* the query for a passenger-class car is made with a runtime twin of its own type whose only
  change is vehicle class `custom1`, which closures exempt - so it measures the road distance
  a car would drive if the incident edge were open.

The last two tests guard the two facts that make the twin exact on this network. If either
ever fails, the fix has silently stopped measuring what it claims to.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import emergency_manager
from emergency_manager import (
    CLOSURE_DISALLOWED_CLASSES,
    DISTANCE_PROBE_VCLASS,
    EMERGENCY_EXEMPT_VCLASS,
    EmergencyManager,
    SUMO_VEHICLE_CLASSES,
)

ROOT = Path(__file__).resolve().parent.parent


class Route:
    def __init__(self, edges, length):
        self.edges = edges
        self.length = length
        self.travelTime = length / 10.0


class FakeTraci:
    class TraCIException(Exception):
        pass

    def __init__(self, answers):
        self.answers = answers        # (from, to, type) -> Route
        self.calls = []
        self.types = {"normal_passenger": "passenger", "emergency_ambulance": "emergency"}
        self.copies = []

        outer = self

        class Simulation:
            @staticmethod
            def findRoute(frm, to, vtype, *rest):
                outer.calls.append((frm, to, vtype))
                return outer.answers[(frm, to, vtype)]

        class VehicleType:
            @staticmethod
            def getVehicleClass(type_id):
                if type_id not in outer.types:
                    raise outer.TraCIException(type_id)
                return outer.types[type_id]

            @staticmethod
            def getIDList():
                return tuple(outer.types)

            @staticmethod
            def copy(src, dst):
                outer.copies.append((src, dst))
                outer.types[dst] = outer.types[src]

            @staticmethod
            def setVehicleClass(type_id, vclass):
                outer.types[type_id] = vclass

        self.simulation = Simulation
        self.vehicletype = VehicleType


@pytest.fixture
def fake(monkeypatch):
    twin = "normal_passenger__incident_distance"
    stub = FakeTraci({
        ("a", "centre", "normal_passenger"): Route([], 0.0),          # what SUMO really returns
        ("a", "centre", twin): Route(["a", "b", "centre"], 1827.55),
        ("a", "centre", "emergency_ambulance"): Route(["a", "b", "centre"], 1827.55),
    })
    monkeypatch.setattr(emergency_manager, "traci", stub)
    return stub


def test_an_empty_route_is_no_route_not_zero_metres(fake):
    manager = EmergencyManager()
    assert manager._route_distance("a", "centre", "normal_passenger") == float("inf")


def test_a_car_is_measured_with_its_closure_exempt_twin(fake):
    manager = EmergencyManager()
    query_type = manager._distance_query_type("normal_passenger")

    assert query_type == "normal_passenger__incident_distance"
    assert fake.types[query_type] == DISTANCE_PROBE_VCLASS
    assert manager._route_distance("a", "centre", query_type) == pytest.approx(1827.55)


def test_the_twin_is_created_once_and_uncached_queries_are_one_per_distance(fake):
    """Without the cache, one route query per distance, exactly as the 2026-09-11 fix made."""
    manager = EmergencyManager()
    manager.cache_incident_distances = False
    for _ in range(3):
        manager._route_distance("a", "centre", manager._distance_query_type("normal_passenger"))

    assert len(fake.calls) == 3
    assert len(fake.copies) == 1          # the twin is created once per run, then reused


# The incident-distance cache (2026-09-12). A repeated question is answered from memory; the
# key holds everything SUMO's answer depends on. Asking SUMO less often is harmless only
# because the speed factors are frozen (contract 13.13) - the full-run identity check in
# THESIS_CODE_AUDIT_FIXES.md is what proves the cache changes nothing; these pin its rules.

def test_a_repeated_distance_is_asked_of_sumo_once(fake):
    manager = EmergencyManager()
    query_type = manager._distance_query_type("normal_passenger")
    answers = [manager._route_distance("a", "centre", query_type) for _ in range(3)]

    assert answers == [pytest.approx(1827.55)] * 3
    assert fake.calls == [("a", "centre", query_type)]


def test_the_cache_gives_exactly_the_uncached_answers(fake):
    cached, uncached = EmergencyManager(), EmergencyManager()
    uncached.cache_incident_distances = False
    for vehicle_type in ("normal_passenger", "normal_passenger__incident_distance", "emergency_ambulance"):
        for _ in range(2):
            assert cached._route_distance("a", "centre", vehicle_type) == \
                uncached._route_distance("a", "centre", vehicle_type)


def test_no_route_is_remembered_as_no_route(fake):
    manager = EmergencyManager()
    for _ in range(2):
        assert manager._route_distance("a", "centre", "normal_passenger") == float("inf")
    assert len(fake.calls) == 1


def test_the_vehicle_type_is_part_of_the_key(fake):
    manager = EmergencyManager()
    manager._route_distance("a", "centre", "normal_passenger__incident_distance")
    manager._route_distance("a", "centre", "emergency_ambulance")
    assert len(fake.calls) == 2


def test_opening_or_closing_an_incident_is_a_new_key(fake):
    """A closure changes lane permissions, so an answer from before it must not be reused."""
    manager = EmergencyManager()
    manager._route_distance("a", "centre", "emergency_ambulance")
    manager.active_events = {404}
    manager._route_distance("a", "centre", "emergency_ambulance")
    manager.active_events = {404, 401}
    manager._route_distance("a", "centre", "emergency_ambulance")
    manager.active_events = {404}
    manager._route_distance("a", "centre", "emergency_ambulance")   # same state as before: cached
    assert len(fake.calls) == 3


def test_the_distance_cache_is_reset_for_a_new_run(fake):
    """Its answers belong to one SUMO instance, like the twin types."""
    manager = EmergencyManager()
    manager._route_distance("a", "centre", "emergency_ambulance")
    manager.set_planned_events([])
    assert manager._distance_cache == {}
    manager._route_distance("a", "centre", "emergency_ambulance")
    assert len(fake.calls) == 2


def test_emergency_types_are_queried_exactly_as_before(fake):
    """Responders and ambient emergency vehicles were already answered correctly."""
    manager = EmergencyManager()
    assert manager._distance_query_type("emergency_ambulance") == "emergency_ambulance"
    assert fake.copies == []


def test_the_twin_cache_is_reset_for_a_new_run(fake):
    """A runtime vehicle type does not survive into the next SUMO instance."""
    manager = EmergencyManager()
    manager._distance_query_type("normal_passenger")
    manager.set_planned_events([])
    assert manager._distance_query_types == {}


def test_closures_still_bar_every_real_vehicle_class_except_emergency():
    assert EMERGENCY_EXEMPT_VCLASS not in CLOSURE_DISALLOWED_CLASSES
    assert DISTANCE_PROBE_VCLASS not in CLOSURE_DISALLOWED_CLASSES
    assert "passenger" in CLOSURE_DISALLOWED_CLASSES
    assert set(CLOSURE_DISALLOWED_CLASSES) == set(SUMO_VEHICLE_CLASSES) - {"emergency", "custom1"}


def test_no_real_vehicle_uses_the_distance_class():
    """If a vehicle ever used custom1, closures would stop applying to it."""
    used = set()
    for route_file in (ROOT / "demand").glob("*.rou.xml"):
        for vtype in ET.parse(route_file).getroot().iter("vType"):
            used.add(vtype.get("vClass"))
    assert DISTANCE_PROBE_VCLASS not in used


def test_the_distance_class_has_exactly_car_permissions_on_this_network():
    """The reason the twin measures a car's road distance and not some other vehicle's."""
    all_classes = set(SUMO_VEHICLE_CLASSES) | {"private", "emergency", "authority", "army",
                                               "vip", "pedestrian"}

    def allowed(element):
        allow, disallow = element.get("allow"), element.get("disallow")
        if allow is not None:
            return set(allow.split())
        if disallow is not None:
            return all_classes - set(disallow.split())
        return all_classes

    root = ET.parse(ROOT / "network" / "porto.net.xml").getroot()
    differing = [
        lane.get("id")
        for edge in root.findall("edge") for lane in edge.findall("lane")
        if ("passenger" in allowed(lane)) != (DISTANCE_PROBE_VCLASS in allowed(lane))
    ]
    differing += [
        f"{c.get('from')}->{c.get('to')}"
        for c in root.findall("connection")
        if (c.get("allow") or c.get("disallow"))
        and ("passenger" in allowed(c)) != (DISTANCE_PROBE_VCLASS in allowed(c))
    ]
    assert differing == []
