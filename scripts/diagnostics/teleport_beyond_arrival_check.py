#!/usr/bin/env python3
"""The "teleports beyond arrival edge" accounting rule, validated on real outputs (2026-09-14).

    python scripts/diagnostics/teleport_beyond_arrival_check.py revalidate --out results/teleport_beyond_arrival_2026-09-14
    python scripts/diagnostics/teleport_beyond_arrival_check.py rerun --trial 1 --out results/teleport_beyond_arrival_2026-09-14

`revalidate` runs no simulation and only reads. It covers the canonical baseline and the five
chunk-1 trials. For each it compares every metric and J as stored when the run was made (the old
rule) with today's `run_metrics`, which reads the run's SUMO log (the new rule). It checks that:
- only vehicles SUMO reported as "teleports beyond arrival edge" change class. Each is a
  vaporized="teleport" record at the reported time, with its arrival lane on the reported edge;
- every other vehicle keeps its class, every ordinary teleport included;
- finished equals summary arrived under the new rule;
- teleports, time loss, never-inserted and responses are unchanged. Travel moves by exactly the
  sum of (removal time - T) / n over the reclassified cars, and U by exactly 1 / n per car;
- the baseline and trials 0 and 2 are identical in every stored metric and in J;
- each trial passes the tuner's four rejection tests.

`rerun` runs one 3,600 s simulation: the real tuning-trial function on the named chunk-1 trial's
weights, with today's code, into <out>/rerun_trial_NNN. It checks that the trial is accepted, that
its J equals the revalidated J, and that its SUMO outputs are identical record by record to the
campaign's trial, which shows the simulation itself did not change.

Neither command writes to the campaign: the rejected trials stay rejected in its database. The campaign is now
read from its archive, results/_archive/e3_tuning_campaign_2026-09-14_pre_fix.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
os.chdir(REPO_ROOT)

import tune_e3_weights as tuner  # noqa: E402
from evaluation_seeds import sumo_seed_args  # noqa: E402
from metrics.tuning_objective import (  # noqa: E402
    ended_beyond_arrival, objective, run_metrics, teleports_beyond_arrival)

SEED, T = 20260721, 3600
CANONICAL = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s"
# The chunk-1 campaign was archived on 2026-09-14 (it ran on the pre-fix accounting rule). This script only reads
# it; `revalidate` and `rerun` write into --out, never into the archive (path updated 2026-09-15, user decision).
CAMPAIGN = REPO_ROOT / "results/_archive/e3_tuning_campaign_2026-09-14_pre_fix"
TELEPORT_START = re.compile(r"Warning: Teleporting vehicle '([^']+)'; waited too long")
TABLE = ["finished", "unfinished_in_network", "never_inserted", "summary_arrived", "teleports",
         "incomplete_share", "travel_s", "time_loss_s", "reported_travel_s_completed",
         "reported_time_loss_s_completed", "reported_waiting_s_completed"]
UNCHANGED = ["never_inserted", "summary_arrived", "teleports", "teleports_per_vehicle", "time_loss_s",
             "vehicles_scheduled", "records_outside_population", "responses"]


def population() -> dict:
    return tuner.scheduled_population(tuner._route_files(sumo_seed_args(SEED, Path("simulation.sumocfg"))), T)


def old_finished(record) -> bool:
    """The rule every stored number was computed with, until 2026-09-14."""
    return float(record["arrival"]) >= 0 and not record.get("vaporized")


def revalidate(out: Path) -> dict:
    pop = population()
    n = len(pop)
    ref_stored = json.loads((CANONICAL / "reference.json").read_text(encoding="utf-8"))["metrics"]
    runs = [("baseline", CANONICAL, ref_stored, None)]
    for i in range(5):
        folder = CAMPAIGN / "trials" / f"trial_{i:03d}"
        record = json.loads((folder / "trial.json").read_text(encoding="utf-8"))
        runs.append((f"trial {i}", folder, record["metrics"], record))

    checks, rows = [], {}

    def check(name, ok, detail=""):
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    ref_new = None
    for name, folder, stored, trial in runs:
        new = run_metrics(folder / "tripinfo.xml", folder / "summary.xml", pop, T, {},
                          sumo_log=folder / "errors.log")
        new["responses"] = stored["responses"]      # an input to run_metrics, never reclassified
        if ref_new is None:
            ref_new = new
        records = {e.get("id"): e.attrib for e in ET.parse(folder / "tripinfo.xml").getroot().iter("tripinfo")}
        reported = teleports_beyond_arrival(folder / "errors.log")
        log = (folder / "errors.log").read_text(encoding="utf-8", errors="replace").splitlines()
        starts = [m.group(1) for m in map(TELEPORT_START.match, log) if m]
        changed = sorted(v for v, r in records.items()
                         if v in pop and old_finished(r) != (old_finished(r) or ended_beyond_arrival(r, reported)))
        ordinary = sorted(set(starts) - set(reported))
        # The baseline's own J is 1.0 up to float rounding (0.9999999999999999), so it is compared
        # with itself under the same code, not with the literal 1.0.
        j_before = objective(stored, ref_stored)["J"] if trial is None else trial["objective"]["J"]
        j_after = objective(new, ref_new)["J"]
        rows[name] = {"before": {k: stored[k] for k in TABLE}, "after": {k: new[k] for k in TABLE},
                      "responses": {e: r["value_s"] for e, r in stored["responses"].items()},
                      "J_before": j_before, "J_after": j_after,
                      "reclassified": changed, "reported_by_sumo": reported,
                      "ordinary_teleport_vehicles": len(ordinary), "teleport_starts_logged": len(starts)}
        tag = name
        check(f"{tag}: only the vehicles SUMO reported change class", changed == sorted(reported),
              f"changed {changed}, reported {sorted(reported)}")
        check(f"{tag}: each reported vehicle is a vaporized=teleport record at that time, on that edge",
              all(ended_beyond_arrival(records[v], reported) for v in reported))
        check(f"{tag}: every ordinary teleport keeps its class ({len(ordinary)} vehicles)",
              not set(ordinary) & set(changed))
        check(f"{tag}: SUMO's teleport count equals the teleports it logged, the reported ones included",
              new["teleports"] == len(starts) and set(reported) <= set(starts), f"{new['teleports']} / {len(starts)}")
        check(f"{tag}: finished equals summary arrived under the new rule", new["finished"] == new["summary_arrived"],
              f"{new['finished']} / {new['summary_arrived']}")
        check(f"{tag}: teleports, time loss, never-inserted, responses and population unchanged",
              all(new[k] == stored[k] for k in UNCHANGED))
        k = len(changed)
        travel_shift = sum(float(records[v]["arrival"]) - T for v in changed) / n
        check(f"{tag}: finished +{k}, unfinished -{k}, U -{k}/n, travel by the removal times exactly",
              new["finished"] - stored["finished"] == k
              and stored["unfinished_in_network"] - new["unfinished_in_network"] == k
              and abs((stored["incomplete_share"] - new["incomplete_share"]) - k / n) < 1e-12
              and abs((new["travel_s"] - stored["travel_s"]) - travel_shift) < 1e-9,
              f"travel {stored['travel_s']:.6f} -> {new['travel_s']:.6f} (expected shift {travel_shift:.6f})")
        if not changed:
            check(f"{tag}: identical in every stored metric and in J",
                  {key: new[key] for key in stored} == stored and j_after == j_before)
        if trial is not None:
            check(f"{tag}: passes the tuner's four rejection tests under the new rule",
                  trial["exceptions_logged"] == [] and trial["weights_used"] == trial["weights_requested"]
                  and trial["pairing"]["speed_factor_differences"] == 0
                  and new["finished"] == new["summary_arrived"] and new["records_outside_population"] == 0)
    check("canonical baseline recomputed with the new rule equals reference.json in every metric",
          {k: ref_new[k] for k in ref_stored} == ref_stored and ref_new["finished_by_teleport_beyond_arrival"] == [])

    passed = sum(c["pass"] for c in checks)
    verdict = {"verdict": "PASS" if passed == len(checks) else "FAIL", "passed": passed, "total": len(checks),
               "scheduled_vehicles": n, "runs": rows, "checks": checks}
    out.mkdir(parents=True, exist_ok=True)
    (out / "revalidate.json").write_text(json.dumps(verdict, indent=1, default=str), encoding="utf-8")

    for name, row in rows.items():
        print(f"\n{name}: J {row['J_before']:.6f} -> {row['J_after']:.6f}; reclassified {row['reclassified']}; "
              f"ordinary-teleport vehicles {row['ordinary_teleport_vehicles']}")
        for key in TABLE:
            b, a = row["before"][key], row["after"][key]
            mark = "" if b == a else "   <- changed"
            print(f"  {key:32} {b!s:>22} {a!s:>22}{mark}")
        print(f"  {'responses (value_s)':32} {row['responses']!s:>45}   unchanged")
    for c in checks:
        if not c["pass"]:
            print("FAIL", c["check"], c["detail"])
    print(f"\nteleport-beyond-arrival revalidation: {verdict['verdict']} ({passed} of {len(checks)} checks)")
    return verdict


def _summary_steps(path: Path) -> list:
    return [{k: v for k, v in s.attrib.items() if k != "duration"}     # wall-clock, not simulation
            for s in ET.parse(path).getroot().findall("step")]


def rerun(trial: int, out: Path) -> dict:
    source = CAMPAIGN / "trials" / f"trial_{trial:03d}"
    record = json.loads((source / "trial.json").read_text(encoding="utf-8"))
    target = out / f"rerun_trial_{trial:03d}"
    if target.exists() and any(target.iterdir()):
        raise SystemExit(f"{target} already holds files; evidence is never overwritten")
    reference = tuner.load_reference(CANONICAL / "reference.json", SEED, T)
    accepted, detail = True, ""
    try:
        result = tuner.evaluate_weights(record["weights_requested"], Path("simulation.sumocfg"), T, SEED,
                                        trial_dir=target, reference=reference)
        j = result.objective
    except tuner.TrialRejected as exc:
        accepted, detail, j = False, str(exc), None
    new = json.loads((target / "trial.json").read_text(encoding="utf-8"))
    revalidated = json.loads((out / "revalidate.json").read_text(encoding="utf-8"))["runs"][f"trial {trial}"]

    def records(folder):
        return {e.get("id"): e.attrib for e in ET.parse(folder / "tripinfo.xml").getroot().iter("tripinfo")}

    checks = {
        "trial accepted by the tuner (no TrialRejected)": accepted,
        "J equals the revalidated J exactly": j is not None and j == revalidated["J_after"],
        "the reported car is counted as finished by teleport beyond arrival":
            new["metrics"]["finished_by_teleport_beyond_arrival"] == revalidated["reclassified"],
        "finished equals summary arrived": new["finished_equals_summary_arrived"] is True,
        "tripinfo identical to the campaign trial, record by record": records(target) == records(source),
        "summary identical to the campaign trial, step by step (wall-clock duration excluded)":
            _summary_steps(target / "summary.xml") == _summary_steps(source / "summary.xml"),
        "SUMO warnings identical": (target / "errors.log").read_text(encoding="utf-8").splitlines()
            == (source / "errors.log").read_text(encoding="utf-8").splitlines(),
        "E3 counters, route statistics, learning and pairing identical":
            all(new[k] == record[k] for k in ("coordinator", "route_stats", "learning", "pairing")),
        "the trial's own weights only": new["weights_used"] == new["weights_requested"] == record["weights_requested"],
        "no exception logged": new["exceptions_logged"] == [],
    }
    verdict = {"verdict": "PASS" if all(checks.values()) else "FAIL", "trial": trial, "J": j,
               "J_recorded_by_the_campaign_under_the_old_rule": record["objective"]["J"],
               "rejection": detail, "checks": checks}
    (out / f"rerun_trial_{trial:03d}_verdict.json").write_text(json.dumps(verdict, indent=1), encoding="utf-8")
    for name, ok in checks.items():
        print(("PASS " if ok else "FAIL ") + name)
    print(f"rerun of trial {trial}: {verdict['verdict']}; J {j} (old rule, never scored: {record['objective']['J']})")
    return verdict


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["revalidate", "rerun"])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--trial", type=int, default=1)
    args = parser.parse_args()
    verdict = revalidate(args.out) if args.command == "revalidate" else rerun(args.trial, args.out)
    return 0 if verdict["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
