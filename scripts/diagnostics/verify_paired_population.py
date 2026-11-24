#!/usr/bin/env python3
"""Do all arms drive the identical vehicle population? (contract v0.7, decision 13.13)

The check behind true paired conditions. Every ordinary vehicle's speed factor is frozen in
demand/porto.rou.xml; this runs one arm and compares what SUMO actually gave each vehicle
against demand/porto.speed_factors.json. The target is zero differences, for every arm.

Measured 2026-09-11 on seed 20260722: baseline and both P3 arms over 3,600 s, 3,759 of 3,759
departing cars matching; Dijkstra, A*, ACO, ACO-PSO and E3 over 1,200 s, 1,288 of 1,288 each;
0 disagreements on the 1,381 vehicles common to all eight arms.

Runs one arm and records the SUMO speed factor of every vehicle the first time it exists -
pending or running - by wrapping traci.simulationStep from outside, so the arm's own code is
untouched and any runner can be checked. Reads only; draws nothing.

Arms: baseline, dijkstra, astar, aco, aco_pso, e3 (the six-algorithm comparison table, via
metrics_logger.run_metrics) and p3off, p3on (the P3 coordinator with the emergency layer off
and on).

    python scripts/diagnostics/verify_paired_population.py --arm e3 --duration 1200 --out DIR
    python scripts/diagnostics/verify_paired_population.py --arm p3on --seed 20260723 --out DIR

Then compare the recorded speed factors with demand/porto.speed_factors.json; any difference
means that arm is not driving the frozen population. With `--seed` (an evaluation seed,
contract 13.15) the arm runs that seed's frozen file, and the script itself compares against
that seed's manifest and prints the number of differences.
"""
import argparse
import json
import os
import sys
import traceback
from pathlib import Path

ROOT = Path(os.environ.get("PROBE_ROOT", Path(__file__).resolve().parent.parent.parent))
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

import traci  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--arm", required=True)
parser.add_argument("--duration", type=int, default=1200)
parser.add_argument("--out", required=True)
parser.add_argument("--seed", type=int, default=None, help="evaluation seed (contract 13.15)")
args = parser.parse_args()
out = Path(args.out)
out.mkdir(parents=True, exist_ok=True)

from evaluation_seeds import frozen_demand_paths, route_choice_seed, sumo_seed_args  # noqa: E402

seed_args = sumo_seed_args(args.seed)

speed_factors = {}
_original_step = traci.simulationStep


def recording_step(*a, **k):
    result = _original_step(*a, **k)
    for vehicle_id in (*traci.simulation.getPendingVehicles(), *traci.vehicle.getIDList()):
        if vehicle_id not in speed_factors:
            speed_factors[vehicle_id] = traci.vehicle.getSpeedFactor(vehicle_id)
    return result


traci.simulationStep = recording_step
counters = {}

try:
    if args.arm in ("baseline", "dijkstra", "astar", "aco", "aco_pso", "e3"):
        from metrics.metrics_logger import run_metrics
        run_metrics(args.arm, duration=args.duration, seed=args.seed)
    else:
        from emergency_manager import get_emergency_manager
        from p3_runtime import P3RuntimeCoordinator
        manager = get_emergency_manager()
        manager.load_canonical_scenario_file()
        manager.generate_emergencies()
        coordinator = P3RuntimeCoordinator(emergency_layer=(args.arm == "p3on"),
                                           route_choice_seed=route_choice_seed(args.seed))
        traci.start(["sumo", "-c", "simulation.sumocfg", "--no-step-log", "true",
                     "--end", str(args.duration),
                     "--tripinfo-output", str(out / "tripinfo.xml"),
                     "--summary-output", str(out / "summary.xml"),
                     "--emission-output", str(out / "emissions.xml"),
                     "--queue-output", str(out / "queue.xml"),
                     "--message-log", str(out / "messages.log"),
                     "--error-log", str(out / "errors.log"),
                     *seed_args])
        try:
            while traci.simulation.getTime() < args.duration:
                traci.simulationStep()
                t = traci.simulation.getTime()
                manager.update(t)
                coordinator.update_step(t)
        finally:
            traci.close()
        counters = {"reroute_decisions": coordinator.metrics.reroute_decisions,
                    "reroute_executed": coordinator.metrics.reroute_executed,
                    "yield_requests": coordinator.metrics.yield_requests,
                    "yield_actions": coordinator.metrics.yield_actions_performed}
except Exception:
    traceback.print_exc()
    raise

(out / "result.json").write_text(json.dumps({"arm": args.arm, "evaluation_seed": args.seed,
                                             "speed_factors": speed_factors,
                                             "counters": counters}), encoding="utf-8")
print(f"[{args.arm}] vehicles recorded: {len(speed_factors)}")

_, manifest_path = frozen_demand_paths(args.seed if args.seed is not None else
                                       json.loads((ROOT / "demand" / "porto.speed_factors.json")
                                                  .read_text(encoding="utf-8"))["sumo_seed"])
frozen = {k: float(v) for k, v in json.loads(manifest_path.read_text(encoding="utf-8"))["speed_factors"].items()}
common = [v for v in frozen if v in speed_factors]
differing = [v for v in common if speed_factors[v] != frozen[v]]
print(f"[{args.arm}] {len(common)} ordinary vehicles compared with {manifest_path.name}: "
      f"{len(differing)} differ{' - e.g. ' + ', '.join(differing[:5]) if differing else ''}")
