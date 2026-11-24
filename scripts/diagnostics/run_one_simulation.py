#!/usr/bin/env python3
"""Run ONE simulation: any routing method, any seed, into a fresh folder of its own.

A convenience entry point for running the simulation by hand (2026-10-02). It is not a new
implementation and it is not the evidence path: it calls exactly the functions the frozen V1
comparison and the eight-seed extension call -

    baseline   tune_e3_weights.build_reference           (no rerouting)
    dijkstra   tune_e3_weights.evaluate_comparator("dijkstra")
    astar      tune_e3_weights.evaluate_comparator("astar")
    aco        tune_e3_weights.evaluate_comparator("aco")
    aco_pso    tune_e3_weights.evaluate_comparator("aco_pso")
    e3_off     tune_e3_weights.evaluate_weights(T16, emergency_layer=False)
    e3_on      tune_e3_weights.evaluate_weights(T16, emergency_layer=True)

- with the same SUMO configuration, the seed's frozen demand, the same incidents and the T16
weights. Output goes to a NEW folder (default `results/manual_runs/<method>_seed_<seed>_<time>s_<UTC>/`);
an existing folder is refused, so no stored result can be overwritten. When the seed already
has a stored no-rerouting reference for the same duration, J is computed against it (read only).

    python scripts/diagnostics/run_one_simulation.py --method e3_on --seed 20260722
    python scripts/diagnostics/run_one_simulation.py --method dijkstra --seed 20260723 --sim-time 1200
    python scripts/diagnostics/run_one_simulation.py --list-seeds

It lives in scripts/diagnostics/ so that, like the other runners there, it is outside the code
fingerprint of the frozen chain.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
WEIGHTS_FILE = REPO_ROOT / "results/final_weight_candidates_2026-09-16/trial_016_weights.json"
METHODS = ("baseline", "dijkstra", "astar", "aco", "aco_pso", "e3_off", "e3_on")
INCIDENTS = ("404", "401", "405")


def _setup() -> None:
    os.chdir(REPO_ROOT)
    for path in (REPO_ROOT, REPO_ROOT / "scripts"):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))


def available_seeds() -> list:
    """Seeds that have a frozen demand file (the only seeds a run can use)."""
    seeds = set()
    canonical = REPO_ROOT / "demand" / "porto.speed_factors.json"
    if canonical.is_file():
        seeds.add(int(json.loads(canonical.read_text(encoding="utf-8"))["sumo_seed"]))
    for manifest in (REPO_ROOT / "demand" / "seeds").glob("porto.speed_factors.*.json"):
        seeds.add(int(manifest.stem.rsplit(".", 1)[-1]))
    return sorted(seeds)


def stored_result(method: str, seed: int, sim_time: int):
    """The thesis's own stored run of the same method and seed (read only), or None."""
    if sim_time != 3600:
        return None
    tag = {"e3_off": "e3_off_t16", "e3_on": "e3_on_t16"}.get(method, method)
    candidates = [
        f"results/tuning_reference/seed_{seed}_3600s/reference.json",
        f"results/comparison_v1_2026-09-17/runs/{tag}_seed_{seed}/trial/trial.json",
        f"results/heldout_eval_2026-09-16/runs/trial_016_seed_{seed}/trial/trial.json",
        f"results/eightseed_extension_2026-09-20/runs/{tag}_seed_{seed}/trial/trial.json",
    ]
    if method == "baseline":
        candidates = candidates[:1]
    elif method == "e3_on":
        candidates = candidates[2:]
    else:
        candidates = candidates[1:2] + candidates[3:]
    for rel_path in candidates:
        path = REPO_ROOT / rel_path
        if path.is_file():
            record = json.loads(path.read_text(encoding="utf-8"))
            objective = record.get("objective")
            if isinstance(objective, dict):
                objective = objective.get("J")
            return rel_path, record["metrics"], objective
    return None


def summarise(metrics: dict, objective) -> dict:
    responses = metrics.get("responses", {})
    return {
        "vehicles_scheduled": metrics.get("vehicles_scheduled"),
        "finished": metrics.get("finished"),
        "mean_travel_time_s_censored": None if metrics.get("travel_s") is None else round(metrics["travel_s"], 1),
        "mean_time_loss_s": None if metrics.get("time_loss_s") is None else round(metrics["time_loss_s"], 1),
        "teleports": metrics.get("teleports"),
        "response_time_s": {e: responses.get(e, {}).get("value_s") for e in INCIDENTS},
        "response_status": {e: responses.get(e, {}).get("status") for e in INCIDENTS},
        "objective_J": objective,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--method", choices=METHODS)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--sim-time", type=int, default=3600, help="simulated seconds (3600 = the thesis runs)")
    parser.add_argument("--out", type=Path, default=None, help="a NEW folder for this run's files")
    parser.add_argument("--list-seeds", action="store_true", help="print the seeds that can be run, then stop")
    parser.add_argument("--gui", action="store_true", help="open the SUMO-GUI window and show the cars")
    parser.add_argument("--delay", type=int, default=20, help="with --gui: milliseconds per simulated second")
    parser.add_argument("--autostart", action="store_true",
                        help="with --gui: start at once instead of waiting for the Play button")
    args = parser.parse_args()

    if args.list_seeds:
        print("seeds with a frozen demand file:", ", ".join(map(str, available_seeds())))
        return 0
    if not args.method or args.seed is None:
        parser.error("--method and --seed are required")
    if args.seed not in available_seeds():
        raise SystemExit(f"seed {args.seed} has no frozen demand file. Seeds available: {available_seeds()}. "
                         f"A new seed must first be prepared with: "
                         f"python scripts/08b_freeze_speed_factors.py --seed {args.seed}")

    # Every stored run was made with PYTHONHASHSEED=0. It must be set before Python starts,
    # so if it is not, run this same command again in a child process that has it.
    if os.environ.get("PYTHONHASHSEED") != "0":
        env = dict(os.environ, PYTHONHASHSEED="0")
        return subprocess.call([sys.executable, *sys.argv], env=env)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = (args.out or REPO_ROOT / "results" / "manual_runs"
           / f"{args.method}_seed_{args.seed}_{args.sim_time}s_{stamp}").resolve()
    if out.exists():
        raise SystemExit(f"{out} already exists; choose a new --out (nothing is ever overwritten)")
    out.mkdir(parents=True)

    _setup()
    import tune_e3_weights as tuner

    if args.gui:
        # Display only: the binary becomes sumo-gui and view options are added; every other
        # argument, and so the run itself, is unchanged. The look is the thesis screenshots' view.
        sys.path.insert(0, str(REPO_ROOT / "scripts" / "visualization"))
        from e3_gui_run import gui_settings_file
        settings = gui_settings_file(out / "gui_view.xml", args.delay)
        original_command = tuner._sumo_command

        def gui_command(sumo_cfg, sim_time, out_dir, seed_args):
            command = list(original_command(sumo_cfg, sim_time, out_dir, seed_args))
            command[0] = "sumo-gui"
            return command + ["--start", "true" if args.autostart else "false", "--quit-on-end", "true",
                              "--gui-settings-file", str(settings), "--delay", str(args.delay)]

        tuner._sumo_command = gui_command
        if not args.autostart:
            print("[gui] the SUMO window opens paused: set up your recording, then press the green Play button",
                  flush=True)

    cfg = Path("simulation.sumocfg")
    print(f"[run] {args.method}, seed {args.seed}, {args.sim_time} s -> {out}", flush=True)
    clock, rejected, objective, metrics = time.time(), None, None, {}
    meta = {"method": args.method, "seed": args.seed, "sim_time": args.sim_time, "gui": args.gui,
            "started_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "pythonhashseed": os.environ.get("PYTHONHASHSEED")}

    if args.method == "baseline":
        reference = tuner.build_reference(cfg, args.seed, args.sim_time, out)
        metrics, objective = reference["metrics"], 1.0
    else:
        ref_file = tuner.default_reference_path(args.seed, args.sim_time)
        reference = tuner.load_reference(ref_file, args.seed, args.sim_time) if ref_file.is_file() else None
        meta["reference"] = str(ref_file) if reference else None
        try:
            if args.method in ("e3_off", "e3_on"):
                from p3_hybrid_router import load_weights_file
                weights = load_weights_file(WEIGHTS_FILE)
                layer = args.method == "e3_on"
                meta.update({"emergency_layer": layer, "weights": weights,
                             "weights_file": WEIGHTS_FILE.relative_to(REPO_ROOT).as_posix()})
                result = tuner.evaluate_weights(weights, cfg, args.sim_time, args.seed, trial_dir=out,
                                                reference=reference, emergency_layer=layer)
            else:
                result = tuner.evaluate_comparator(args.method, cfg, args.sim_time, args.seed, trial_dir=out,
                                                   reference=reference)
            metrics, objective = result.metrics, result.objective
        except tuner.TrialRejected as exc:
            rejected = str(exc)

    summary = summarise(metrics, objective)
    meta.update({"rejected": rejected, "wall_seconds": round(time.time() - clock, 1), "summary": summary})
    (out / "run_summary.json").write_text(json.dumps(meta, indent=1, default=str), encoding="utf-8")

    print("\n================ RESULT ================")
    print(f"method {args.method}   seed {args.seed}   {args.sim_time} s   wall {meta['wall_seconds']:.0f} s")
    if rejected:
        print(f"REJECTED (not a valid run): {rejected}")
    for e in INCIDENTS:
        print(f"  incident {e}: response {summary['response_time_s'][e]} s  ({summary['response_status'][e]})")
    print(f"  finished {summary['finished']} / {summary['vehicles_scheduled']}   "
          f"teleports {summary['teleports']}")
    print(f"  mean travel time (censored) {summary['mean_travel_time_s_censored']}   "
          f"mean time loss {summary['mean_time_loss_s']}")
    print(f"  J {round(objective, 4) if objective is not None else '- (no stored baseline for this seed and duration)'}")
    stored = stored_result(args.method, args.seed, args.sim_time)
    if stored:
        rel_path, s_metrics, s_objective = stored
        s = summarise(s_metrics, s_objective if args.method != "baseline" else 1.0)
        same = all(s["response_time_s"][e] == summary["response_time_s"][e] for e in INCIDENTS) \
            and s["finished"] == summary["finished"] and s["teleports"] == summary["teleports"]
        print(f"\nstored thesis run of the same method and seed ({rel_path}):")
        print("  responses " + ", ".join(f"{e}: {s['response_time_s'][e]} s" for e in INCIDENTS)
              + f"   finished {s['finished']}   teleports {s['teleports']}   J {round(s['objective_J'], 4)}")
        print("  -> SAME as the stored run" if same else "  -> DIFFERENT from the stored run")
    print(f"files: {out}")
    return 0 if rejected is None else 1


if __name__ == "__main__":
    sys.exit(main())
