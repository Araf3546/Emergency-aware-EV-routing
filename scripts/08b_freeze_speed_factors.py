#!/usr/bin/env python3
"""Freeze every ordinary vehicle's speed factor into the demand file (contract v0.7, 13.13).

Run after `08_generate_baseline_routes.py`, which writes `demand/porto.rou.xml` without them.

Why this step exists
--------------------
`normal_passenger` sets no `speedFactor`, so SUMO draws one for every ordinary car from its
default passenger distribution when it loads the car. Every `traci.simulation.findRoute` call
consumes a draw from that same random stream. Algorithms make very different numbers of route
queries - the plain-SUMO baseline makes none, an E3 arm starts at its first reroute - so at the
same seed they were driving different populations: 508 of 704 ordinary cars (72%) in the first
700 s of an E3 run had a different speed factor from the same cars in the baseline run
(measured 2026-09-11). That broke contract section 5, "routing logic is the only permitted
difference between compared runs", and made paired seeds barely paired.

What it does
------------
It does not re-sample and it does not change the distribution. It runs the scenario exactly as
the plain-SUMO baseline does - same configuration, same SUMO seed, incidents applied, and no
route queries of any kind - and reads the speed factor SUMO itself drew for every ordinary car,
at full double precision, through TraCI. Those values are then written into each car's
`<vehicle>` element. With an explicit value SUMO no longer draws one, so route queries can no
longer reach the traffic, and every arm drives the identical population.

Because the values are the ones the baseline already drove with, the baseline run itself is
unchanged - which is checked afterwards rather than assumed.

One realisation per SUMO seed
-----------------------------
The draws depend on the SUMO seed. Freezing one realisation and reusing it for every evaluation
seed would give all seeds the same drivers and remove real between-seed variation. So the SUMO
seed is an argument and is recorded in the output: the five-seed evaluation (work order item 4)
must run this once per evaluation seed and give each seed its own frozen file.

The seed of simulation.sumocfg (20260722) freezes `demand/porto.rou.xml` in place, as it always
did. Any other seed writes `demand/seeds/porto.rou.<seed>.xml` and its manifest, and is
harvested from the archived UNFROZEN demand file, passed to SUMO explicitly. That matters:
`simulation.sumocfg` loads `demand/porto.rou.xml`, which is already frozen, and SUMO does not
draw for a car that carries a speed factor - so a harvest through the config would silently
return seed 20260722's values under every seed.

A per-seed file is verified by a second run before it is kept: the baseline is run again on
the new file, under the same seed, and must reproduce the harvest run trip for trip, with every
vehicle driving exactly its frozen value. Otherwise nothing is written.
`evaluation_seeds.py` is what the runners use to select a seed's file.

    python scripts/08b_freeze_speed_factors.py                   # seed from simulation.sumocfg
    python scripts/08b_freeze_speed_factors.py --seed 20260723   # demand/seeds/porto.rou.20260723.xml
    python scripts/08b_freeze_speed_factors.py --check           # verify, write nothing
    python scripts/08b_freeze_speed_factors.py --check --seed 20260723
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import statistics
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from evaluation_seeds import config_route_files, frozen_demand_paths  # noqa: E402

DEMAND = REPO_ROOT / "demand" / "porto.rou.xml"
MANIFEST = REPO_ROOT / "demand" / "porto.speed_factors.json"
BACKUP_DIR = REPO_ROOT / "archive_2026-09-11"
SUMO_CFG = REPO_ROOT / "simulation.sumocfg"
ORDINARY_DEMAND_ENTRY = "demand/porto.rou.xml"
ORDINARY_TYPE = "normal_passenger"
# Only for a car proven never to depart in any arm; see departs_into_a_closure.
PLACEHOLDER_SPEED_FACTOR = 1.0
VEHICLE_TAG = re.compile(r'<vehicle id="(?P<id>[^"]+)"(?P<rest>[^>]*?)(?P<close>/?>)')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sumo_seed_from_config() -> int:
    seed = ET.parse(SUMO_CFG).getroot().find(".//seed")
    return int(seed.get("value"))


def ordinary_vehicle_ids(route_file: Path) -> list:
    return [v.get("id") for v in ET.parse(route_file).getroot().iter("vehicle")
            if v.get("type") == ORDINARY_TYPE]


def already_frozen(route_file: Path) -> bool:
    return any(v.get("speedFactor") is not None
               for v in ET.parse(route_file).getroot().iter("vehicle"))


def route_files_with(demand_file: Path) -> str:
    """simulation.sumocfg's route files, with `demand_file` in place of the ordinary demand."""
    return ",".join(
        str(demand_file) if entry == ORDINARY_DEMAND_ENTRY else str(REPO_ROOT / entry)
        for entry in config_route_files(SUMO_CFG)
    )


def harvest(seed: int, duration: int, demand_file: Path = None, out: Path = None) -> dict:
    """SUMO's own speed-factor draw for every vehicle, from a query-free baseline run.

    `demand_file` replaces demand/porto.rou.xml for this run. It must be given whenever that
    file is already frozen, or SUMO draws nothing and the run just reads the frozen values back.
    Its tripinfo.xml is left in `out` so a second run can be compared against it.
    """
    os.chdir(REPO_ROOT)
    import traci
    from emergency_manager import get_emergency_manager

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    out = out or Path(tempfile.mkdtemp(prefix="freeze_speed_factors_"))
    out.mkdir(parents=True, exist_ok=True)
    route_override = [] if demand_file is None else ["--route-files", route_files_with(demand_file)]
    traci.start([
        "sumo", "-c", str(SUMO_CFG), "--seed", str(seed), "--no-step-log", "true",
        "--end", str(duration),
        "--tripinfo-output", str(out / "tripinfo.xml"),
        "--summary-output", str(out / "summary.xml"),
        "--emission-output", str(out / "emissions.xml"),
        "--queue-output", str(out / "queue.xml"),
        "--message-log", str(out / "messages.log"),
        "--error-log", str(out / "errors.log"),
        *route_override,
    ])
    factors = {}
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            t = traci.simulation.getTime()
            # Exactly what the plain-SUMO baseline arm does each step, and nothing more.
            manager.update(t)
            manager.record_responder_arrivals(
                t, vehicle_ids=traci.vehicle.getIDList(),
                arrived_ids=traci.simulation.getArrivedIDList(),
            )
            # Reading a speed factor is not a route query and draws nothing. Pending vehicles
            # (loaded, not yet inserted) are read too, so cars that never get onto the road
            # within the hour are still covered; their value is the one they would drive with
            # (checked: 168 of 168 identical before and after insertion).
            for vehicle_id in (*traci.simulation.getPendingVehicles(), *traci.vehicle.getIDList()):
                if vehicle_id not in factors:
                    factors[vehicle_id] = traci.vehicle.getSpeedFactor(vehicle_id)
    finally:
        traci.close()
    return factors


def departs_into_a_closure(route_file: Path, vehicle_ids) -> dict:
    """{vehicle: event} for vehicles whose first edge is closed by an incident at departure."""
    from emergency_manager import get_emergency_manager

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    wanted = set(vehicle_ids)
    found = {}
    for vehicle in ET.parse(route_file).getroot().iter("vehicle"):
        if vehicle.get("id") not in wanted:
            continue
        depart = float(vehicle.get("depart"))
        first_edge = vehicle.find("route").get("edges").split()[0]
        for event in manager.events.values():
            if event.start_time <= depart < event.end_time and first_edge in event.blocked_edges:
                found[vehicle.get("id")] = event.event_id
    return found


def write_frozen(source: Path, target: Path, factors: dict) -> int:
    """Add speedFactor to every ordinary vehicle's opening tag. Nothing else changes."""
    text = source.read_text(encoding="utf-8")
    written = 0

    def add(match):
        nonlocal written
        vehicle_id = match.group("id")
        rest = match.group("rest")
        if f'type="{ORDINARY_TYPE}"' not in rest:
            return match.group(0)
        if "speedFactor=" in rest:
            raise SystemExit(f"{vehicle_id} already has a speedFactor - refusing to overwrite")
        written += 1
        # repr() is the shortest decimal that parses back to the identical double, so SUMO
        # reads exactly the value it drew.
        return f'<vehicle id="{vehicle_id}"{rest} speedFactor="{repr(factors[vehicle_id])}"{match.group("close")}'

    frozen = VEHICLE_TAG.sub(add, text)
    header = (f"<!-- speedFactor of every {ORDINARY_TYPE} vehicle frozen by "
              f"scripts/08b_freeze_speed_factors.py from SUMO seed {factors['__seed__']}; "
              f"see demand/porto.speed_factors.json and contract decision 13.13 -->\n")
    frozen = frozen.replace("<routes", header + "<routes", 1)
    target.write_text(frozen, encoding="utf-8")
    return written


def verify(unfrozen: Path, frozen: Path, factors: dict) -> list:
    """Every vehicle identical except the added attribute, and every value exact."""
    problems = []
    a = list(ET.parse(unfrozen).getroot().iter("vehicle"))
    b = list(ET.parse(frozen).getroot().iter("vehicle"))
    if len(a) != len(b):
        problems.append(f"vehicle count {len(a)} -> {len(b)}")
    for old, new in zip(a, b):
        old_attrs = dict(old.attrib)
        new_attrs = dict(new.attrib)
        value = new_attrs.pop("speedFactor", None)
        if old_attrs != new_attrs:
            problems.append(f"{old.get('id')}: attributes changed")
        if [ET.tostring(c) for c in old] != [ET.tostring(c) for c in new]:
            problems.append(f"{old.get('id')}: route changed")
        if old.get("type") == ORDINARY_TYPE:
            if value is None:
                problems.append(f"{old.get('id')}: no speedFactor written")
            elif float(value) != factors[old.get("id")]:
                problems.append(f"{old.get('id')}: {value} does not round-trip")
        elif value is not None:
            problems.append(f"{old.get('id')}: speedFactor written on a non-ordinary vehicle")
    return problems


def tripinfo_records(path: Path) -> list:
    """Every tripinfo record, in file order, with all of its attributes."""
    return [dict(element.attrib) for element in ET.parse(path).getroot().iter("tripinfo")]


def verify_by_run(seed: int, duration: int, frozen_file: Path, harvested: dict, harvest_out: Path) -> dict:
    """Run the baseline again on the frozen file and compare it with the harvest run.

    The harvest run is the unfrozen baseline under this seed. If the frozen values are exactly
    SUMO's own draws, and nothing else SUMO draws at load reaches the traffic, the two runs are
    identical trip for trip and every vehicle drives its frozen value.
    """
    check_out = harvest_out.parent / (harvest_out.name + "_frozen_check")
    driven = harvest(seed, duration, demand_file=frozen_file, out=check_out)
    differing = sorted(v for v in harvested if v in driven and driven[v] != harvested[v])
    missing = sorted(v for v in harvested if v not in driven)
    before = tripinfo_records(harvest_out / "tripinfo.xml")
    after = tripinfo_records(check_out / "tripinfo.xml")
    identical = sum(1 for a, b in zip(before, after) if a == b)
    return {
        "vehicles_compared": len(harvested) - len(missing),
        "speed_factor_differences": len(differing),
        "vehicles_missing_in_check_run": len(missing),
        "tripinfo_records": [len(before), len(after)],
        "tripinfo_records_identical": identical,
        "passed": not differing and not missing and len(before) == len(after) == identical,
        "first_differences": differing[:5] or missing[:5],
    }


def per_seed_freeze(seed: int, duration: int) -> int:
    """Freeze a seed other than simulation.sumocfg's into demand/seeds/, verified by a second run."""
    target, manifest_path = frozen_demand_paths(seed)
    if target == DEMAND:
        raise SystemExit("per_seed_freeze is for seeds other than the canonical one")
    if target.exists() or manifest_path.exists():
        print(f"{target.name} already exists; this step never freezes twice. Delete it and its "
              f"manifest first if it really must be regenerated.")
        return 2

    canonical = json.loads(MANIFEST.read_text(encoding="utf-8"))
    source = REPO_ROOT / canonical["unfrozen_backup"]
    if sha256(source) != canonical["unfrozen_sha256"]:
        print(f"{source} does not match the unfrozen hash recorded in {MANIFEST.name}; refusing.")
        return 2

    wanted = ordinary_vehicle_ids(source)
    work = Path(tempfile.mkdtemp(prefix=f"freeze_{seed}_"))
    print(f"harvesting SUMO's own speed factors, seed {seed}, {duration} s, {len(wanted)} ordinary "
          f"vehicles, from the unfrozen file {source.relative_to(REPO_ROOT)} ...", flush=True)
    factors = harvest(seed, duration, demand_file=source, out=work / "harvest")
    missing = [v for v in wanted if v not in factors]
    undepartable = departs_into_a_closure(source, missing)
    unexplained = [v for v in missing if v not in undepartable]
    if unexplained:
        print(f"FAILED: {len(unexplained)} ordinary vehicles were never loaded, e.g. "
              f"{unexplained[:5]}, and none departs from a closed edge. Nothing written.")
        return 2
    for vehicle_id in undepartable:
        factors[vehicle_id] = PLACEHOLDER_SPEED_FACTOR

    ordinary = {v: factors[v] for v in wanted}
    ordinary["__seed__"] = seed
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.with_suffix(".frozen.tmp")
    written = write_frozen(source, staging, ordinary)
    ordinary.pop("__seed__")
    problems = verify(source, staging, ordinary)
    if written != len(wanted) or problems:
        staging.unlink()
        print(f"FAILED: wrote {written} of {len(wanted)}; problems: {problems[:5]}. Nothing written.")
        return 2

    print("verifying: the baseline again on the frozen file, same seed ...", flush=True)
    harvested = {v: f for v, f in factors.items() if v not in undepartable}
    check = verify_by_run(seed, duration, staging, harvested, work / "harvest")
    print(f"  speed-factor differences {check['speed_factor_differences']} over "
          f"{check['vehicles_compared']} vehicles; tripinfo {check['tripinfo_records_identical']} of "
          f"{check['tripinfo_records'][0]} records identical")
    if not check["passed"]:
        staging.unlink()
        print(f"FAILED verification: {check}. Nothing written.")
        return 2

    staging.replace(target)
    values = [v for k, v in ordinary.items() if k not in undepartable]
    manifest_path.write_text(json.dumps({
        "contract_decision": "13.13, 13.15",
        "sumo_seed": seed,
        "vehicle_type": ORDINARY_TYPE,
        "vehicles": len(ordinary),
        "harvested_from_sumo": len(values),
        "placeholders": {v: {"value": PLACEHOLDER_SPEED_FACTOR,
                             "reason": f"departs from incident {e}'s closed edge while it is closed; "
                                       f"refused by SUMO in every arm"}
                         for v, e in sorted(undepartable.items())},
        "statistics_cover": "harvested values only",
        "mean": statistics.mean(values),
        "sd": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "unfrozen_sha256": canonical["unfrozen_sha256"],
        "unfrozen_backup": canonical["unfrozen_backup"],
        "frozen_sha256": sha256(target),
        "verified_by_second_run": check,
        "speed_factors": {k: repr(v) for k, v in ordinary.items()},
    }, indent=1), encoding="utf-8")
    print(f"froze {len(values)} speed factors: mean {statistics.mean(values):.4f}, "
          f"sd {statistics.pstdev(values):.4f}, range {min(values):.4f}-{max(values):.4f}")
    print(f"written: {target.relative_to(REPO_ROOT)} and {manifest_path.relative_to(REPO_ROOT)}")
    return 0


def check_frozen(seed: int) -> int:
    """Verify a frozen file against its manifest and the unfrozen original. Writes nothing."""
    target, manifest_path = frozen_demand_paths(seed)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    unfrozen = REPO_ROOT / manifest["unfrozen_backup"]
    factors = {k: float(v) for k, v in manifest["speed_factors"].items()}
    problems = verify(unfrozen, target, factors)
    if int(manifest["sumo_seed"]) != int(seed):
        problems.append(f"manifest is for seed {manifest['sumo_seed']}, not {seed}")
    print(f"seed {seed}: {target.relative_to(REPO_ROOT)}")
    print(f"frozen file sha256 matches manifest: {sha256(target) == manifest['frozen_sha256']}")
    print(f"problems: {problems[:5] if problems else 'none'}")
    return 1 if problems or sha256(target) != manifest["frozen_sha256"] else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, default=None, help="SUMO seed (default: simulation.sumocfg)")
    parser.add_argument("--duration", type=int, default=3600)
    parser.add_argument("--check", action="store_true", help="verify the frozen file, write nothing")
    args = parser.parse_args()
    seed = args.seed if args.seed is not None else sumo_seed_from_config()

    if args.check:
        return check_frozen(seed)

    # Any seed other than the config's gets its own file under demand/seeds/.
    if seed != sumo_seed_from_config():
        return per_seed_freeze(seed, args.duration)

    if already_frozen(DEMAND):
        print(f"{DEMAND.name} already carries speed factors. Regenerate it with "
              f"08_generate_baseline_routes.py first; this step never freezes twice.")
        return 2

    BACKUP_DIR.mkdir(exist_ok=True)
    backup = BACKUP_DIR / "porto.rou.unfrozen.xml"
    if not backup.exists():
        shutil.copy2(DEMAND, backup)
    elif sha256(backup) != sha256(DEMAND):
        print(f"{backup} exists and differs from the current demand file - refusing to overwrite it.")
        return 2

    wanted = ordinary_vehicle_ids(DEMAND)
    print(f"harvesting SUMO's own speed factors, seed {seed}, {args.duration} s, "
          f"{len(wanted)} ordinary vehicles ...", flush=True)
    factors = harvest(seed, args.duration)
    missing = [v for v in wanted if v not in factors]

    # A car scheduled to depart from an incident's closed edge while it is closed is refused
    # by SUMO at insertion and never exists in ANY arm - the closure and the departure time
    # are both fixed. Found: normal_00646, departing t=644 from incident 404's centre edge.
    # It has no speed factor to harvest and can never use one, so it gets the distribution
    # mean as a labelled placeholder, which also stops SUMO drawing for it at load. Any
    # other missing car is a real failure.
    undepartable = departs_into_a_closure(DEMAND, missing)
    unexplained = [v for v in missing if v not in undepartable]
    if unexplained:
        print(f"FAILED: {len(unexplained)} ordinary vehicles were never loaded, e.g. "
              f"{unexplained[:5]}, and none departs from a closed edge. Nothing written.")
        return 2
    for vehicle_id in undepartable:
        factors[vehicle_id] = PLACEHOLDER_SPEED_FACTOR
    if undepartable:
        print(f"placeholder {PLACEHOLDER_SPEED_FACTOR} for {len(undepartable)} vehicle(s) that "
              f"depart from a closed incident edge in every arm: {sorted(undepartable)}")

    ordinary = {v: factors[v] for v in wanted}
    ordinary["__seed__"] = seed
    staging = DEMAND.with_suffix(".frozen.tmp")
    written = write_frozen(DEMAND, staging, ordinary)
    ordinary.pop("__seed__")
    problems = verify(DEMAND, staging, ordinary)
    if written != len(wanted) or problems:
        staging.unlink()
        print(f"FAILED: wrote {written} of {len(wanted)}; problems: {problems[:5]}. Nothing written.")
        return 2

    original_hash = sha256(DEMAND)
    staging.replace(DEMAND)
    values = [v for k, v in ordinary.items() if k not in undepartable]
    MANIFEST.write_text(json.dumps({
        "contract_decision": "13.13",
        "sumo_seed": seed,
        "vehicle_type": ORDINARY_TYPE,
        "vehicles": len(ordinary),
        "harvested_from_sumo": len(values),
        "placeholders": {v: {"value": PLACEHOLDER_SPEED_FACTOR,
                             "reason": f"departs from incident {e}'s closed edge while it is closed; "
                                       f"refused by SUMO in every arm"}
                         for v, e in sorted(undepartable.items())},
        "statistics_cover": "harvested values only",
        "mean": statistics.mean(values),
        "sd": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "unfrozen_sha256": original_hash,
        "unfrozen_backup": str(backup.relative_to(REPO_ROOT)).replace("\\", "/"),
        "frozen_sha256": sha256(DEMAND),
        "speed_factors": {k: repr(v) for k, v in ordinary.items()},
    }, indent=1), encoding="utf-8")
    print(f"froze {len(values)} speed factors: mean {statistics.mean(values):.4f}, "
          f"sd {statistics.pstdev(values):.4f}, range {min(values):.4f}-{max(values):.4f}")
    print(f"backup of the unfrozen file: {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
