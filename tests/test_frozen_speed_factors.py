"""Every compared arm must drive the identical vehicle population (contract v0.7, 13.13).

SUMO draws an ordinary car's speed factor from a random stream that `findRoute` also
consumes, so arms making different numbers of route queries used to drive different
populations at the same seed - 72% of cars differed between an E3 run and the baseline in
the first 700 s. The fix freezes each car's speed factor into `demand/porto.rou.xml`, using
the value SUMO itself drew in a query-free baseline run.

These tests fail if the demand file is regenerated without re-running
`scripts/08b_freeze_speed_factors.py`, or if the frozen file drifts from its manifest.
"""

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEMAND = ROOT / "demand" / "porto.rou.xml"
MANIFEST = ROOT / "demand" / "porto.speed_factors.json"


def load():
    vehicles = list(ET.parse(DEMAND).getroot().iter("vehicle"))
    return vehicles, json.loads(MANIFEST.read_text(encoding="utf-8"))


def test_every_ordinary_vehicle_carries_an_explicit_speed_factor():
    vehicles, _ = load()
    ordinary = [v for v in vehicles if v.get("type") == "normal_passenger"]
    assert len(ordinary) == 3760
    assert all(v.get("speedFactor") is not None for v in ordinary)


def test_the_demand_file_is_the_one_the_manifest_describes():
    _, manifest = load()
    assert hashlib.sha256(DEMAND.read_bytes()).hexdigest() == manifest["frozen_sha256"]


def test_frozen_values_are_exactly_the_manifest_values():
    vehicles, manifest = load()
    for vehicle in vehicles:
        if vehicle.get("type") == "normal_passenger":
            assert float(vehicle.get("speedFactor")) == float(manifest["speed_factors"][vehicle.get("id")])


def test_the_distribution_was_frozen_not_changed():
    """SUMO's default passenger speed factor has mean 1 and deviation 0.1."""
    _, manifest = load()
    assert abs(manifest["mean"] - 1.0) < 0.01
    assert abs(manifest["sd"] - 0.1) < 0.01
    assert manifest["min"] < manifest["max"]        # not collapsed onto one value


def test_placeholders_only_for_cars_that_can_never_depart():
    _, manifest = load()
    assert set(manifest["placeholders"]) == {"normal_00646"}
    assert manifest["harvested_from_sumo"] == manifest["vehicles"] - len(manifest["placeholders"])


def test_nothing_but_the_speed_factor_differs_from_the_unfrozen_file():
    vehicles, manifest = load()
    unfrozen = list(ET.parse(ROOT / manifest["unfrozen_backup"]).getroot().iter("vehicle"))
    assert len(unfrozen) == len(vehicles)
    for old, new in zip(unfrozen, vehicles):
        attrs = dict(new.attrib)
        attrs.pop("speedFactor", None)
        assert attrs == dict(old.attrib)
        assert [ET.tostring(c) for c in old] == [ET.tostring(c) for c in new]
