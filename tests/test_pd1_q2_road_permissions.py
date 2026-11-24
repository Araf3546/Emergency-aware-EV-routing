"""PD-1 question 2, decided by the supervisor on 2026-09-13: "Ambient emergency vehicles should
NOT retain emergency-only road permissions."

The only road permission unique to the emergency class on this network is the incident-closure
exemption. At the start of every run the four ambient emergency vTypes are switched to SUMO class
`authority`, which has exactly emergency's permissions on every lane and connection here but is
blocked by closures. The three responder types keep `emergency`. These tests pin:

* the premise: `authority` equals `emergency` on this network, and closures block it;
* the switch: all four ambient types, never a responder type or a car, before the first closure,
  again in every new run, and a type SUMO loads later is switched when it appears;
* that no emergency vType in the demand escapes the switch except the dispatched responders';
* the incident-distance twin: an ambient type is still measured with its original permissions;
* the router: the closure exemption follows the vehicle's live class.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

import emergency_manager
from algorithms import route_utils
from emergency_manager import (
    AMBIENT_EMERGENCY_ROAD_CLASS,
    CLOSURE_DISALLOWED_CLASSES,
    DISTANCE_TWIN_SUFFIX,
    EMERGENCY_EXEMPT_VCLASS,
    SUMO_VEHICLE_CLASSES,
    EmergencyManager,
)
from p3_simulation_config import AMBIENT_EMERGENCY_TYPES

ROOT = Path(__file__).resolve().parent.parent
RESPONDER_TYPES = ("ambulance", "fire_truck", "police")


class FakeTraci:
    """Just enough of TraCI for update(): vTypes, one closable lane, and the call order."""

    class TraCIException(Exception):
        pass

    def __init__(self, types):
        self.types = dict(types)          # vType id -> vClass
        self.calls = []
        self.id_list_calls = 0
        fake = self

        class VehicleType:
            @staticmethod
            def getIDList():
                fake.id_list_calls += 1
                return tuple(fake.types)

            @staticmethod
            def getVehicleClass(type_id):
                if type_id not in fake.types:
                    raise FakeTraci.TraCIException(f"type {type_id} not known")
                return fake.types[type_id]

            @staticmethod
            def setVehicleClass(type_id, vehicle_class):
                fake.calls.append(("setVehicleClass", type_id, vehicle_class))
                fake.types[type_id] = vehicle_class

            @staticmethod
            def copy(type_id, new_id):
                fake.types[new_id] = fake.types[type_id]

        class Simulation:
            @staticmethod
            def getStartingTeleportIDList():
                return ()

        class Edge:
            @staticmethod
            def getLaneNumber(edge_id):
                return 1

        class Lane:
            @staticmethod
            def getDisallowed(lane_id):
                return ()

            @staticmethod
            def getMaxSpeed(lane_id):
                return 13.89

            @staticmethod
            def setDisallowed(lane_id, classes):
                fake.calls.append(("setDisallowed", lane_id))

        self.vehicletype = VehicleType()
        self.simulation = Simulation()
        self.edge = Edge()
        self.lane = Lane()


def scenario_types():
    types = {t: "emergency" for t in AMBIENT_EMERGENCY_TYPES + RESPONDER_TYPES}
    types["normal_passenger"] = "passenger"
    return types


def manager_with(monkeypatch, fake, start_time=1):
    monkeypatch.setattr(emergency_manager, "traci", fake)
    manager = EmergencyManager()
    manager.set_planned_events([{
        "event_id": 404, "start_time": start_time, "end_time": 900, "incident_type": "fire",
        "center_edge": "E_centre", "affected_edges": ["E_centre"],
        "dispatch": {"vehicle_id": "emergency_dispatch_404", "vehicle_type": "fire_truck"},
    }])
    return manager


# ---------------------------------------------------------------- the premise


def allowed(element, all_classes):
    allow, disallow = element.get("allow"), element.get("disallow")
    if allow is not None:
        return set(allow.split())
    if disallow is not None:
        return all_classes - set(disallow.split())
    return set(all_classes)


def test_authority_has_exactly_emergency_permissions_on_this_network():
    """Why switching to `authority` removes the closure exemption and nothing else."""
    all_classes = set(SUMO_VEHICLE_CLASSES)
    root = ET.parse(ROOT / "network" / "porto.net.xml").getroot()
    elements = [lane for edge in root.findall("edge") for lane in edge.findall("lane")]
    elements += [c for c in root.findall("connection") if c.get("allow") or c.get("disallow")]
    differing = [e.get("id") or f"{e.get('from')}->{e.get('to')}" for e in elements
                 if ("authority" in allowed(e, all_classes)) != ("emergency" in allowed(e, all_classes))]
    assert differing == []
    assert len(elements) > 20000


def test_closures_block_the_ambient_class_and_exempt_the_responders():
    assert AMBIENT_EMERGENCY_ROAD_CLASS == "authority"
    assert AMBIENT_EMERGENCY_ROAD_CLASS in CLOSURE_DISALLOWED_CLASSES
    assert EMERGENCY_EXEMPT_VCLASS not in CLOSURE_DISALLOWED_CLASSES


def test_every_emergency_vtype_in_the_demand_is_switched_or_a_dispatched_responder():
    """A new ambient type name that the switch does not know would keep the exemption."""
    emergency_types = set()
    for route_file in (ROOT / "demand").glob("*.rou.xml"):
        for vtype in ET.parse(route_file).getroot().iter("vType"):
            if vtype.get("vClass") == "emergency":
                emergency_types.add(vtype.get("id"))
    scenario = yaml.safe_load((ROOT / "scenarios" / "emergency_scenarios.yaml").read_text(encoding="utf-8"))
    responder_types = {event["dispatch"]["vehicle_type"] for event in scenario["events"]}
    assert responder_types == set(RESPONDER_TYPES)
    assert emergency_types == set(AMBIENT_EMERGENCY_TYPES) | responder_types
    assert not set(AMBIENT_EMERGENCY_TYPES) & responder_types


# ---------------------------------------------------------------- the switch


def test_ambient_types_switch_to_authority_and_nothing_else_changes(monkeypatch):
    fake = FakeTraci(scenario_types())
    manager = manager_with(monkeypatch, fake, start_time=600)
    manager.update(1)
    for vehicle_type in AMBIENT_EMERGENCY_TYPES:
        assert fake.types[vehicle_type] == "authority"
    for vehicle_type in RESPONDER_TYPES:
        assert fake.types[vehicle_type] == "emergency"
    assert fake.types["normal_passenger"] == "passenger"
    assert manager.ambient_road_classes == {t: "authority" for t in AMBIENT_EMERGENCY_TYPES}


def test_the_switch_comes_before_the_first_closure(monkeypatch):
    fake = FakeTraci(scenario_types())
    manager = manager_with(monkeypatch, fake, start_time=1)
    manager.update(1)                   # the incident starts in the very first update
    kinds = [call[0] for call in fake.calls]
    assert kinds.index("setDisallowed") > max(i for i, k in enumerate(kinds) if k == "setVehicleClass")


def test_a_type_sumo_loads_later_is_switched_when_it_appears(monkeypatch):
    types = scenario_types()
    late = types.pop("emergency_police")
    fake = FakeTraci(types)
    manager = manager_with(monkeypatch, fake, start_time=600)
    manager.update(1)
    assert "emergency_police" not in manager.ambient_road_classes
    fake.types["emergency_police"] = late
    manager.update(2)
    assert fake.types["emergency_police"] == "authority"
    calls = fake.id_list_calls
    manager.update(3)                   # all four switched: no more TraCI calls for this
    assert fake.id_list_calls == calls


def test_every_new_run_switches_again(monkeypatch):
    fake = FakeTraci(scenario_types())
    manager = manager_with(monkeypatch, fake, start_time=600)
    manager.update(1)
    second = FakeTraci(scenario_types())            # a new SUMO instance starts as the files say
    monkeypatch.setattr(emergency_manager, "traci", second)
    manager.load_canonical_scenario_file()          # what every runner and tuning trial does first
    assert manager.ambient_road_classes == {}
    manager.update(1)
    assert all(second.types[t] == "authority" for t in AMBIENT_EMERGENCY_TYPES)


def test_the_before_after_switch_leaves_the_classes_alone_when_off(monkeypatch):
    monkeypatch.setattr(EmergencyManager, "restrict_ambient_road_access", False)
    fake = FakeTraci(scenario_types())
    manager = manager_with(monkeypatch, fake, start_time=600)
    manager.update(1)
    assert all(fake.types[t] == "emergency" for t in AMBIENT_EMERGENCY_TYPES)
    assert manager.ambient_road_classes == {}


# ---------------------------------------------------------------- distance twin and router


def test_an_ambient_type_is_measured_with_its_original_permissions(monkeypatch):
    fake = FakeTraci(scenario_types())
    manager = manager_with(monkeypatch, fake, start_time=600)
    manager.update(1)
    ambient = manager._distance_query_type("emergency_ambulance")
    assert ambient == f"emergency_ambulance{DISTANCE_TWIN_SUFFIX}"
    assert fake.types[ambient] == "emergency"          # exempt, and the class it had before
    car = manager._distance_query_type("normal_passenger")
    assert fake.types[car] == "custom1"                # unchanged
    assert manager._distance_query_type("ambulance") == "ambulance"   # responders unchanged


class RouterTraci:
    class TraCIException(Exception):
        pass

    def __init__(self, classes):
        outer = self

        class Vehicle:
            @staticmethod
            def getVehicleClass(veh_id):
                if veh_id not in classes:
                    raise outer.TraCIException("unknown")
                return classes[veh_id]

        self.vehicle = Vehicle()


def test_router_closure_exemption_follows_the_live_class(monkeypatch):
    monkeypatch.setattr(route_utils, "traci", RouterTraci({
        "emergency_dispatch_404": "emergency", "emergency_00062": "authority",
        "normal_00001": "passenger"}))
    assert route_utils.vehicle_is_closure_exempt("emergency_dispatch_404") is True
    assert route_utils.vehicle_is_closure_exempt("emergency_00062") is False
    assert route_utils.vehicle_is_closure_exempt("normal_00001") is False
    assert route_utils.vehicle_is_closure_exempt("gone") is False
    assert route_utils.vehicle_is_closure_exempt(None) is False
