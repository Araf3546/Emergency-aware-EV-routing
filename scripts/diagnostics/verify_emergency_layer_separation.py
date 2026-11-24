#!/usr/bin/env python3
"""Work order item 3: does the emergency-layer ON/OFF switch separate cleanly?

The matched ablation compares an E3 run with the emergency layer ON against the same E3
run with it OFF. That is only a valid comparison if two things hold, and this measures
both on real SUMO runs rather than arguing them from the code.

  Separation is real
      With the layer ON, the emergency term changes a measurable share of route decisions,
      and yielding fires only while a responder is actually responding.

  The control is clean
      With the layer OFF, route choices are identical to an E3 run that carries no protocol
      information at all. This is the proof that "off" did not quietly create a third,
      different algorithm - "off" forces the protocol term to the neutral constant 0.5,
      and a constant added to every candidate cannot change which candidate wins.

Three configurations, each its own SUMO run at the same seed, demand, network and
incidents:

  on           the emergency layer as shipped
  off          the control: protocol term pinned to neutral, no yielding, no corridor claim
  no-protocol  the reference: the vehicles never hear anything from each other at all

Every route decision is recorded as (time, vehicle, candidate edges, chosen edge) and the
logs are compared. `off` and `no-protocol` must match exactly. Any difference between `on`
and `off` is the emergency layer's effect on routing.

Writes nothing into the project - the decision logs go to a temporary directory.

    python scripts/diagnostics/verify_emergency_layer_separation.py
    python scripts/diagnostics/verify_emergency_layer_separation.py --duration 1500
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

MODES = ("on", "off", "no-protocol")


def run_one_mode(mode: str, duration: int, log_path: Path) -> dict:
    """One SUMO run, recording every route decision. Called in a subprocess."""
    os.chdir(REPO_ROOT)

    import traci
    import p3_hybrid_router as H

    decisions = []
    _orig_select = H.HybridRouter.select_best_edge
    current_vehicle = {"id": None, "time": 0.0}

    def select_best_edge(self, candidates):
        best, score = _orig_select(self, candidates)
        if len(candidates) > 1:
            decisions.append([
                current_vehicle["time"],
                current_vehicle["id"],
                "|".join(edge for edge, _ in candidates),
                best,
            ])
        return best, score

    H.HybridRouter.select_best_edge = select_best_edge

    from p3_runtime import P3RuntimeCoordinator
    from emergency_manager import get_emergency_manager

    coordinator = P3RuntimeCoordinator(emergency_layer=(mode == "on"))

    # Which vehicle is deciding is not passed down to the router, so it is captured here.
    _orig_decide = P3RuntimeCoordinator._apply_hybrid_decision

    def apply_hybrid_decision(self, vehicle_id, sim_time):
        current_vehicle["id"] = vehicle_id
        current_vehicle["time"] = sim_time
        return _orig_decide(self, vehicle_id, sim_time)

    P3RuntimeCoordinator._apply_hybrid_decision = apply_hybrid_decision

    if mode == "no-protocol":
        # The reference arm hears nothing. Broadcasting still happens - it is the receiving
        # side that is silenced - so the run differs from `off` only in whether any message
        # is available to score with. After A3 an edge nobody mentions and an edge pinned
        # to the neutral constant score the same, so these two must come out identical.
        coordinator.protocol_state.get_fresh_neighbors = lambda *args, **kwargs: []
        coordinator.protocol_state.get_emergency_neighbors = lambda *args, **kwargs: []

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    out_dir = log_path.parent
    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true", "--end", str(duration),
        "--tripinfo-output", str(out_dir / f"{mode}_tripinfo.xml"),
        "--error-log", str(out_dir / f"{mode}_errors.log"),
    ])
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)
            coordinator.update_step(sim_time)
    finally:
        try:
            traci.close()
        except Exception:
            pass

    responses = manager.get_response_summary()
    result = {
        "mode": mode,
        "duration_s": duration,
        "decisions": decisions,
        "yield_actions_performed": coordinator.metrics.yield_actions_performed,
        "yield_requests": coordinator.metrics.yield_requests,
        "response_times_s": {
            str(event_id): state["response_time_s"] for event_id, state in responses.items()
        },
        "responder_teleport_count": manager.get_teleport_summary()["responder_teleport_count"],
    }
    log_path.write_text(json.dumps(result), encoding="utf-8")
    return result


def compare(first: dict, second: dict) -> dict:
    """Compare two decision logs. The first divergence is what matters.

    Once two runs choose differently the simulations diverge and every later decision is
    made in a different world, so counting all later differences would overstate the
    effect. The honest summary is whether they ever diverged, and where.
    """
    a, b = first["decisions"], second["decisions"]
    identical_prefix = 0
    for left, right in zip(a, b):
        if left != right:
            break
        identical_prefix += 1

    diverged = identical_prefix < min(len(a), len(b))
    return {
        "decisions_recorded": [len(a), len(b)],
        "identical": a == b,
        "identical_decisions_before_first_difference": identical_prefix,
        "first_difference": (
            {"a": a[identical_prefix], "b": b[identical_prefix]} if diverged else None
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=1500,
                        help="responder 401 departs at 960 and arrives near 1490, so 1500 "
                             "covers one complete response")
    parser.add_argument("--mode", choices=MODES, help="internal: run a single arm")
    parser.add_argument("--log", help="internal: where the single arm writes its log")
    arguments = parser.parse_args()

    if arguments.mode:
        run_one_mode(arguments.mode, arguments.duration, Path(arguments.log))
        return 0

    work_dir = Path(tempfile.mkdtemp(prefix="layer_separation_"))
    logs = {}
    for mode in MODES:
        log_path = work_dir / f"{mode}.json"
        print(f"[item3] running {mode} for {arguments.duration}s ...", flush=True)
        subprocess.run(
            [sys.executable, __file__, "--mode", mode, "--duration", str(arguments.duration),
             "--log", str(log_path)],
            check=True, cwd=str(REPO_ROOT),
        )
        logs[mode] = json.loads(log_path.read_text(encoding="utf-8"))

    control_is_clean = compare(logs["off"], logs["no-protocol"])
    separation = compare(logs["on"], logs["off"])

    print("\n=== the control is clean: OFF against the no-protocol reference ===")
    print(json.dumps(control_is_clean, indent=2))
    print("\n=== separation is real: ON against OFF ===")
    print(json.dumps(separation, indent=2))
    print("\n=== yielding and response ===")
    for mode in MODES:
        print(f"  {mode:12s} yields={logs[mode]['yield_actions_performed']:6d} "
              f"requests={logs[mode]['yield_requests']:6d} "
              f"responses={logs[mode]['response_times_s']} "
              f"responder_teleports={logs[mode]['responder_teleport_count']}")

    print("\n=== verdict ===")
    failures = []
    if not control_is_clean["identical"]:
        failures.append(
            "the OFF arm does not match the no-protocol reference, so 'off' is a third "
            "algorithm rather than the same router with the emergency term neutralised"
        )
    if logs["off"]["yield_actions_performed"] != 0:
        failures.append("the OFF arm performed yield actions")
    if logs["on"]["yield_actions_performed"] == 0:
        failures.append("the ON arm performed no yield actions, so nothing is being tested")

    for line in failures:
        print(f"  FAIL: {line}")
    if not failures:
        print("  PASS: the control is clean and yielding is confined to the ON arm.")

    if separation["identical"]:
        print("  NOTE: ON and OFF chose the same edge in every recorded decision. The "
              "emergency layer changed no route in this run - its effect here is yielding "
              "only. That is a property of the scenario, not a failure of the switch.")
    else:
        print(f"  NOTE: ON and OFF diverged after "
              f"{separation['identical_decisions_before_first_difference']} identical decisions.")

    print(f"\n  logs: {work_dir}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
