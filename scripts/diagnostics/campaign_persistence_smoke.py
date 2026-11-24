#!/usr/bin/env python3
"""Persistence and resume smoke test of the real tuner (2026-09-13). NOT tuning: 1,250 s trials,
4 trials per study, and the weights these studies produce must never be used.

    python scripts/diagnostics/campaign_persistence_smoke.py --out results/campaign_persistence_smoke_2026-09-13

Runs the real CLI, exactly as the campaign will be run, in two folders at once:

* `chunked/`: `--new --stop-after 2` (a clean stop after 2 trials), then `--resume` hard-killed
  mid-trial together with its SUMO process (what a power cut does), then `--resume` to the end,
  then `--resume` once more (must add nothing);
* `continuous/`: the same campaign with `--new` and no interruption.

Then checks that the chunked study resumed rather than restarted, kept its completed trials and
their folders byte for byte, marked the killed trial FAIL and evaluated its weights again, and
evaluated exactly the continuous study's weights, in the same order, with the same values and
identical SUMO outputs. Also checks the guards: --new on an existing campaign and --resume with a
changed setting are refused and change nothing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import optuna  # noqa: E402
import psutil  # noqa: E402

TUNER = [sys.executable, "scripts/tune_e3_weights.py"]
SIM_TIME, TRIALS, STARTUP = 1250, 4, 1
NAMES = ("pheromone", "heuristic", "protocol", "bco", "velocity")
OUTPUT_CHECK = ("and identical SUMO outputs trial for trial (every tripinfo record, every summary step "
                "without its wall-clock field, every coordinator counter)")
OLD_OUTPUT_CHECK = "and identical SUMO outputs (tripinfo and summary) trial for trial"


def rejudge(out: Path) -> int:
    """Recompute only the output-identity check from the saved trial files (the other checks were
    taken live during the run and are kept as they were recorded)."""
    verdict = json.loads((out / "verdict.json").read_text(encoding="utf-8"))
    (out / "verdict_first_attempt_raw_bytes.json").write_text(json.dumps(verdict, indent=1), encoding="utf-8")
    checks = {k: v for k, v in verdict["checks"].items() if k != OLD_OUTPUT_CHECK}
    a = verdict["facts"]["evaluated (weights, J, trial) chunked"]
    b = verdict["facts"]["evaluated (weights, J, trial) continuous"]
    pairs = same_outputs(out / "chunked", out / "continuous", a, b)
    checks[OUTPUT_CHECK] = all(pairs) and len(pairs) == TRIALS
    ok = all(checks.values())
    verdict.update({"pass": ok, "checks": checks, "rejudged": (
        "the output check was recomputed from the saved files by comparing parsed records; the first "
        "attempt compared raw bytes, which differ in SUMO's header comment and wall-clock field "
        "(verdict_first_attempt_raw_bytes.json)")})
    (out / "verdict.json").write_text(json.dumps(verdict, indent=1, default=str), encoding="utf-8")
    for key, value in checks.items():
        print(f"{'PASS' if value else 'FAIL'}  {key}")
    print("campaign persistence:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def cli(folder: Path, *extra: str, log: Path, wait: bool = True):
    handle = open(log, "a", encoding="utf-8")
    handle.write(f"\n$ tune_e3_weights.py {' '.join(extra)} --output-dir {folder}\n")
    handle.flush()
    proc = subprocess.Popen([*TUNER, *extra, "--output-dir", str(folder)], stdout=handle, stderr=subprocess.STDOUT)
    if not wait:
        return proc
    code = proc.wait()
    handle.close()
    return code


def study(folder: Path):
    url = f"sqlite:///{(folder.resolve() / 'optuna_study.db').as_posix()}"
    return optuna.load_study(study_name=optuna.get_all_study_names(url)[0], storage=url)


def states(folder: Path):
    return [(t.number, t.state.name, t.user_attrs.get("fail_reason")) for t in study(folder).trials]


def evaluated(folder: Path):
    return [t for t in study(folder).trials if t.state.name == "COMPLETE"
            or (t.state.name == "FAIL" and t.user_attrs.get("fail_reason") == "rejected")]


def same_outputs(chunked: Path, continuous: Path, a: list, b: list) -> list:
    """Per trial pair: every tripinfo record, every summary step and the coordinator counters equal.

    Compared as parsed records, not bytes. Every SUMO output starts with a header comment holding
    the generation time and the output paths, and summary.xml's `duration` is the step's
    wall-clock computing time in milliseconds, so two identical runs never share their bytes.
    (The first verdict of 2026-09-13 compared bytes and failed on exactly that; it is kept as
    verdict_first_attempt_raw_bytes.json.)"""
    import xml.etree.ElementTree as ET

    def records(folder, number):
        trial = folder / "trials" / f"trial_{number:03d}"
        trips = [dict(t.attrib) for t in ET.parse(trial / "tripinfo.xml").getroot().iter("tripinfo")]
        steps = [{k: v for k, v in s.attrib.items() if k != "duration"}
                 for s in ET.parse(trial / "summary.xml").getroot().iter("step")]
        coordinator = json.loads((trial / "trial.json").read_text(encoding="utf-8"))["coordinator"]
        return trips, steps, coordinator

    return [records(chunked, na) == records(continuous, nb) for (_, _, na), (_, _, nb) in zip(a, b)]


def folder_hashes(path: Path) -> dict:
    return {str(p.relative_to(path)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(path.rglob("*")) if p.is_file()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--rejudge", action="store_true",
                        help="recompute the output-identity check of an existing run from its files")
    parsed = parser.parse_args()
    out = Path(parsed.out)
    if parsed.rejudge:
        return rejudge(out)
    chunked, continuous = out / "chunked", out / "continuous"
    for folder in (chunked, continuous):
        if folder.exists():
            raise SystemExit(f"{folder} exists; use a new --out")
    out.mkdir(parents=True, exist_ok=True)
    common = ("--trials", str(TRIALS), "--sim-time", str(SIM_TIME), "--startup-trials", str(STARTUP))
    checks, facts = {}, {}

    # the continuous study runs alongside, uninterrupted
    reference_run = cli(continuous, "--new", *common, log=out / "continuous.log", wait=False)

    # 1. first launch, clean stop after 2 trials
    facts["chunk 1 exit code"] = cli(chunked, "--new", *common, "--stop-after", "2", log=out / "chunked.log")
    facts["after chunk 1"] = states(chunked)
    checks["chunk 1: 2 trials COMPLETE, then a clean stop (exit 0)"] = (
        facts["chunk 1 exit code"] == 0 and facts["after chunk 1"] == [(0, "COMPLETE", None), (1, "COMPLETE", None)])
    checks["chunk 1: no best_e3_weights.json while the campaign is incomplete"] = not (chunked / "best_e3_weights.json").exists()
    kept = {n: folder_hashes(chunked / "trials" / f"trial_{n:03d}") for n in (0, 1)}

    # guards, while the campaign is paused
    db_before = (chunked / "optuna_study.db").read_bytes()
    facts["--new on the existing campaign: exit code"] = cli(chunked, "--new", *common, log=out / "chunked.log")
    facts["--resume with a changed --sim-time: exit code"] = cli(chunked, "--resume", "--sim-time", "3600", log=out / "chunked.log")
    checks["guards: --new on an existing campaign and a changed resume are refused and change nothing"] = (
        facts["--new on the existing campaign: exit code"] != 0
        and facts["--resume with a changed --sim-time: exit code"] != 0
        and (chunked / "optuna_study.db").read_bytes() == db_before)

    # 2. resume, then a hard kill of the process tree mid-trial: what a power cut does
    proc = cli(chunked, "--resume", log=out / "chunked.log", wait=False)
    deadline = time.time() + 600
    while time.time() < deadline:
        running = [t for t in study(chunked).trials if t.state.name == "RUNNING"]
        tripinfo = chunked / "trials" / "trial_002" / "summary.xml"
        if running and tripinfo.exists() and tripinfo.stat().st_size > 50_000:
            break
        time.sleep(2)
    tree = [proc.pid] + [c.pid for c in psutil.Process(proc.pid).children(recursive=True)]
    facts["killed process tree (python + sumo)"] = [(p, psutil.Process(p).name()) for p in tree if psutil.pid_exists(p)]
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True)
    proc.wait()
    time.sleep(2)
    facts["processes of the killed run still alive"] = [p for p in tree if psutil.pid_exists(p)]
    facts["after the kill (what Optuna leaves)"] = states(chunked)
    checks["kill: the killed trial is left RUNNING by Optuna, completed trials untouched"] = (
        facts["after the kill (what Optuna leaves)"] == [(0, "COMPLETE", None), (1, "COMPLETE", None), (2, "RUNNING", None)])
    checks["kill: no process of the killed run survives"] = facts["processes of the killed run still alive"] == []
    killed_params = dict(study(chunked).trials[2].params)

    # 3. resume to the end
    facts["chunk 3 exit code"] = cli(chunked, "--resume", log=out / "chunked.log")
    facts["after chunk 3"] = states(chunked)
    checks["resume: the killed trial is marked FAIL (interrupted) and not counted"] = (
        facts["after chunk 3"][2] == (2, "FAIL", "interrupted") and len(evaluated(chunked)) == TRIALS)
    checks["resume: numbering continues (3, 4), nothing restarted"] = [n for n, _, _ in facts["after chunk 3"]] == [0, 1, 2, 3, 4]
    checks["resume: the killed trial's weights are evaluated again as trial 3"] = study(chunked).trials[3].params == killed_params
    # TRIAL_STATE.json of a COMPLETE trial holds no timestamp, so the whole folder must match.
    checks["resume: completed trial folders byte-identical after two resumes"] = all(
        folder_hashes(chunked / "trials" / f"trial_{n:03d}") == kept[n] for n in (0, 1))
    marker = json.loads((chunked / "trials" / "trial_002" / "TRIAL_STATE.json").read_text(encoding="utf-8"))
    checks["resume: the killed trial's folder is kept and marked, never a result"] = (
        marker["state"] == "FAIL" and marker["fail_reason"] == "interrupted" and not marker["counts_as_a_result"]
        and (chunked / "trials" / "trial_002" / "summary.xml").exists())
    checks["resume: best_e3_weights.json written once the target is reached"] = (chunked / "best_e3_weights.json").exists()

    # 4. one more resume adds nothing
    db_before = (chunked / "optuna_study.db").read_bytes()
    facts["resume of a finished campaign: exit code"] = cli(chunked, "--resume", log=out / "chunked.log")
    checks["a resume of a finished campaign runs no trial"] = (
        facts["resume of a finished campaign: exit code"] == 0 and len(study(chunked).trials) == 5)

    # 5. the continuous study
    facts["continuous exit code"] = reference_run.wait()
    facts["continuous"] = states(continuous)
    a = [(tuple(t.params[n] for n in NAMES), t.value, t.number) for t in evaluated(chunked)]
    b = [(tuple(t.params[n] for n in NAMES), t.value, t.number) for t in evaluated(continuous)]
    facts["evaluated (weights, J, trial) chunked"] = a
    facts["evaluated (weights, J, trial) continuous"] = b
    checks["the chunked+killed study evaluated exactly the continuous study's weights, in order, with the same J"] = (
        [x[:2] for x in a] == [x[:2] for x in b] and len(a) == TRIALS)
    pairs = same_outputs(chunked, continuous, a, b)
    checks[OUTPUT_CHECK] = all(pairs) and len(pairs) == TRIALS
    trial_json = [json.loads((chunked / "trials" / f"trial_{n:03d}" / "trial.json").read_text(encoding="utf-8")) for _, _, n in a]
    checks["every evaluated trial: no exception, 0 speed-factor differences, ambient types switched"] = all(
        not t["exceptions_logged"] and t["pairing"]["speed_factor_differences"] == 0
        and set(t["ambient_road_classes"].values()) == {"authority"} for t in trial_json)
    events = [json.loads(line)["event"] for line in (chunked / "campaign_events.jsonl").read_text(encoding="utf-8").splitlines()]
    facts["chunked campaign_events.jsonl"] = events
    from p3_hybrid_router import load_weights_file
    try:
        load_weights_file(chunked / "best_e3_weights.json")
        refused = False
    except ValueError:
        refused = True
    checks["the smoke study's weights are refused by the runtime (1,250 s, 4 trials)"] = refused
    logs = [p.name for p in out.glob("*.log") if "Traceback" in p.read_text(encoding="utf-8", errors="replace")]
    checks["no traceback in any log"] = not logs

    ok = all(checks.values())
    (out / "verdict.json").write_text(json.dumps({"pass": ok, "checks": checks, "facts": facts}, indent=1, default=str),
                                      encoding="utf-8")
    for key, value in checks.items():
        print(f"{'PASS' if value else 'FAIL'}  {key}")
    print("campaign persistence:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
