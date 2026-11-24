#!/usr/bin/env python3
"""Status and integrity check of a tuning campaign, run between chunks (2026-09-14).

    python scripts/diagnostics/campaign_status.py results/e3_tuning_campaign_2026-09-14
    python scripts/diagnostics/campaign_status.py results/e3_tuning_campaign_2026-09-14 --save

Read-only. It opens optuna_study.db in SQLite read-only mode and changes no campaign file; --save
adds one new JSON file under <campaign>/status_checks/. While it runs it holds the campaign lock, so
a tuner cannot start mid-check; if a tuner is already running it reports that and checks the files
anyway. It is in scripts/diagnostics/, which the campaign's code fingerprint excludes, and no trial
imports it.

It reports the trial count, the provisional best J with its trial and weights, the J range and
every failed trial, and it checks that:
- the stored design is the frozen one: seed 20260721, 3,600 s, 20 trials (contract 13.19, amended from
  25 on 2026-09-15; read from `p3_hybrid_router.CONTRACT_TUNING_TRIALS`), 10 start-up trials, the
  13.17 objective weights, the production evaluator, the PD-1 question 2 road rule, the canonical
  corrected baseline (unchanged since the campaign was created);
- a resume would be accepted as things stand: the settings and the code/input fingerprint computed
  now equal the stored ones, which is the comparison `--resume` makes;
- trial numbers are contiguous, each trial has its own folder and no folder lacks a trial, and each
  folder's TRIAL_STATE.json mirrors the database;
- every COMPLETE trial ran under the corrected PD-1 rule with its own weights only (the database's
  parameters, the weights requested and the weights the router still held at the end are the same
  numbers, and the runtime had loaded no weight file), logged no exception, kept every car's frozen
  speed factor, and has counters that agree (scheduled = finished + still driving + never
  inserted; tripinfo recounted; finished = summary arrivals). Its J is recomputed from its metrics
  and the canonical baseline with today's objective code and equals the stored value;
- every counted trial's sampler seed is the tuning seed plus its evaluation index;
- progress.json, e3_trials.csv and campaign_events.jsonl agree with the database; no
  best_e3_weights.json exists before the target; no console or SUMO log holds a traceback.

The provisional best is NOT a result: the tuning seed is only the training environment, and the
thesis claim comes from the held-out seeds 20260722-20260726.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sqlite3
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
os.chdir(REPO_ROOT)          # the campaign stores sumo_cfg relative to the repository root

import tune_e3_weights as tuner  # noqa: E402
from metrics.tuning_objective import ended_beyond_arrival, objective, teleports_beyond_arrival  # noqa: E402
from p3_simulation_config import AMBIENT_EMERGENCY_TYPES  # noqa: E402

# The trial budget comes from the one contract constant the tuner and the weight loader also use, so the three
# cannot disagree (20 since 2026-09-15; it was a literal 25 here).
FROZEN_DESIGN = {"seed": 20260721, "sim_time": 3600, "trials_target": tuner.DEFAULT_TRIALS, "startup_trials": 10,
                 "objective_weights": {"travel": 0.35, "response": 0.30, "time_loss": 0.2625,
                                       "incomplete": 0.07, "teleports": 0.0175}}
CANONICAL_REFERENCE = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"
TRIAL_FILES = ("tripinfo.xml", "summary.xml", "errors.log", "messages.log", "trial.json", "TRIAL_STATE.json")


class Checks:
    def __init__(self) -> None:
        self.rows = []

    def add(self, name: str, ok: bool, detail: str = "") -> None:
        self.rows.append({"check": name, "pass": bool(ok), "detail": detail})

    @property
    def failed(self) -> list:
        return [r for r in self.rows if not r["pass"]]


def read_database(folder: Path) -> dict:
    path = (folder / tuner.STORAGE_FILE).resolve()
    con = sqlite3.connect(f"file:{path.as_posix()}?mode=ro", uri=True)
    try:
        studies = con.execute("SELECT study_id, study_name FROM studies").fetchall()
        study_id = studies[0][0] if len(studies) == 1 else None
        attrs = {k: json.loads(v) for k, v in con.execute(
            "SELECT key, value_json FROM study_user_attributes WHERE study_id = ?", (study_id,))}
        trials = []
        for trial_id, number, state, start, end in con.execute(
                "SELECT trial_id, number, state, datetime_start, datetime_complete FROM trials "
                "WHERE study_id = ? ORDER BY number", (study_id,)).fetchall():
            values = con.execute("SELECT value, value_type FROM trial_values WHERE trial_id = ?",
                                 (trial_id,)).fetchall()
            trials.append({
                "number": number, "state": state, "start": start, "end": end,
                "value": values[0][0] if values else None,
                "params": dict(con.execute("SELECT param_name, param_value FROM trial_params "
                                           "WHERE trial_id = ?", (trial_id,)).fetchall()),
                "attrs": {k: json.loads(v) for k, v in con.execute(
                    "SELECT key, value_json FROM trial_user_attributes WHERE trial_id = ?", (trial_id,))},
            })
        return {"studies": [name for _, name in studies], "attrs": attrs, "trials": trials}
    finally:
        con.close()


def has_traceback(path: Path) -> bool:
    return path.is_file() and "Traceback" in path.read_text(encoding="utf-8", errors="replace")


def sumo_error_lines(path: Path) -> list:
    if not path.is_file():
        return []
    return [line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.startswith("Error")]


def check_complete_trial(trial: dict, folder: Path, settings: dict, reference: dict, digest: str,
                         checks: Checks) -> dict:
    n = trial["number"]
    tag = f"trial {n}"
    missing = [f for f in TRIAL_FILES if not (folder / f).is_file()]
    checks.add(f"{tag}: all evidence files present", not missing, f"missing {missing}" if missing else "")
    if missing:
        return {}
    record = json.loads((folder / "trial.json").read_text(encoding="utf-8"))
    state = json.loads((folder / "TRIAL_STATE.json").read_text(encoding="utf-8"))
    metrics = record["metrics"]

    checks.add(f"{tag}: TRIAL_STATE.json mirrors the database",
               state["trial"] == n and state["state"] == "COMPLETE" and state["counts_as_a_result"] is True
               and state["value"] == trial["value"])
    checks.add(f"{tag}: production evaluator, seed, duration and frozen demand of the campaign",
               record["evaluator"] == tuner.TUNER_EVALUATOR and record["seed"] == settings["seed"]
               and record["sim_time"] == settings["sim_time"]
               and record["frozen_demand"] == settings["frozen_demand"])
    checks.add(f"{tag}: corrected PD-1 rule, all four ambient types ran as authority",
               record["road_permissions"] == tuner.ROAD_PERMISSION_POLICY
               and record["ambient_road_classes"] == {t: "authority" for t in AMBIENT_EMERGENCY_TYPES})
    from_params = tuner.trial_weights(trial["params"])    # stick parameters since B7, raw before
    checks.add(f"{tag}: only the trial's own weights scored (database = requested = held at the end)",
               from_params == trial["attrs"].get("weights") == record["weights_requested"]
               == record["weights_used"])
    checks.add(f"{tag}: the runtime had loaded no weight file before the trial's weights were set",
               record["weights_source_overridden"] == "DEFAULT_WEIGHTS", record["weights_source_overridden"])
    checks.add(f"{tag}: no exception logged", record["exceptions_logged"] == [],
               "; ".join(record["exceptions_logged"][:2]))
    checks.add(f"{tag}: frozen speed factors kept",
               record["pairing"]["speed_factor_differences"] == 0
               and record["pairing"]["ordinary_vehicles_compared"] > 0,
               f"{record['pairing']['ordinary_vehicles_compared']} cars compared, "
               f"{record['pairing']['speed_factor_differences']} differ")

    records = list(ET.parse(folder / "tripinfo.xml").getroot().iter("tripinfo"))
    # Recounted here with the objective's own arrival rule (2026-09-14): a normal arrival, or SUMO's
    # "teleports beyond arrival edge" confirmed by the trial's log.
    reported = teleports_beyond_arrival(folder / "errors.log")
    arrived = sum(1 for r in records if (float(r.get("arrival")) >= 0 and not r.get("vaporized"))
                  or ended_beyond_arrival(r.attrib, reported))
    counts_ok = (metrics["finished"] + metrics["unfinished_in_network"] + metrics["never_inserted"]
                 == metrics["vehicles_scheduled"] == reference["metrics"]["vehicles_scheduled"]
                 and len(records) == metrics["finished"] + metrics["unfinished_in_network"]
                 and arrived == metrics["finished"] == metrics["summary_arrived"]
                 and record["finished_equals_summary_arrived"] and record["no_tripinfo_record_outside_population"])
    checks.add(f"{tag}: counters agree (scheduled, finished, driving, never inserted, tripinfo, summary)",
               counts_ok, f"{metrics['finished']} + {metrics['unfinished_in_network']} + "
               f"{metrics['never_inserted']} = {metrics['vehicles_scheduled']}; tripinfo records {len(records)}, "
               f"arrived {arrived}, summary arrived {metrics['summary_arrived']}")
    learning = record.get("learning", {})
    if "outcomes_in_running_mean" in learning:           # recorded since A2 "mean protected" (2026-09-14)
        # Every measured outcome reaches PSO. The ones kept out of the running mean are the windows
        # closed by a new decision that were measured: the tracker counts a closed window before it
        # may discard it (too short, no geometry, implausible), so that is at most
        # settled_by_new_decision and at least that minus all discards.
        o = learning["outcomes"]
        kept_out = learning["outcomes_applied_to_pso"] - learning["outcomes_in_running_mean"]
        discards = o["discarded_too_short"] + o["discarded_no_geometry"] + o["discarded_implausible"]
        checks.add(f"{tag}: outcome learning counters agree (every measured outcome reached PSO; only windows "
                   "closed by a new decision stayed out of the running mean)",
                   learning["outcomes_applied_to_pso"] == o["outcomes_reported"]
                   and o["settled_by_new_decision"] - discards <= kept_out <= o["settled_by_new_decision"],
                   f"measured {o['outcomes_reported']}, applied {learning['outcomes_applied_to_pso']}, kept out of "
                   f"the mean {kept_out}, closed by a new decision {o['settled_by_new_decision']}, discards {discards}")
    checks.add(f"{tag}: the same incidents as the baseline",
               sorted(metrics["responses"]) == sorted(reference["metrics"]["responses"]),
               str(sorted(metrics["responses"])))
    recomputed = objective(metrics, reference["metrics"])["J"]
    checks.add(f"{tag}: J recomputed from its metrics and the canonical baseline equals the stored J",
               abs(recomputed - trial["value"]) <= 1e-12 and record["objective"]["J"] == trial["value"]
               and trial["attrs"].get("metrics", {}).get("objective") == trial["value"],
               f"stored {trial['value']!r}, recomputed {recomputed!r}")
    checks.add(f"{tag}: ran on the campaign's code fingerprint",
               trial["attrs"].get("code_fingerprint_digest") == digest)
    errors = sumo_error_lines(folder / "errors.log")
    checks.add(f"{tag}: no SUMO error and no traceback in its logs",
               not errors and not has_traceback(folder / "errors.log") and not has_traceback(folder / "messages.log"),
               "; ".join(errors[:2]))
    return {"record": record, "metrics": metrics}


def run(folder: Path, save: bool) -> int:
    checks = Checks()
    lock = tuner.CampaignLock(folder)
    try:
        lock.__enter__()
        tuner_running = False
    except SystemExit:
        tuner_running = True
    try:
        db = read_database(folder)
    finally:
        if not tuner_running:
            lock.__exit__(None, None, None)

    # ---------------------------------------------------------------- the campaign
    checks.add("exactly one study in the database", len(db["studies"]) == 1, str(db["studies"]))
    settings = db["attrs"]["campaign"]
    design = {k: settings[k] for k in FROZEN_DESIGN}
    checks.add(f"stored design is the frozen one (seed, 3,600 s, {FROZEN_DESIGN['trials_target']} trials, "
               "10 start-up, 13.17 weights)",
               design == FROZEN_DESIGN, "" if design == FROZEN_DESIGN else str(design))
    checks.add("production evaluator and corrected PD-1 road rule recorded",
               settings["evaluator"] == tuner.TUNER_EVALUATOR
               and settings["road_permissions"] == tuner.ROAD_PERMISSION_POLICY)
    reference_path = Path(settings["reference"])
    checks.add("baseline is the canonical corrected reference, unchanged since the campaign was created",
               reference_path.resolve() == CANONICAL_REFERENCE.resolve()
               and settings["reference_sha256"] == tuner._sha256(CANONICAL_REFERENCE), str(reference_path))
    reference = tuner.load_reference(reference_path, settings["seed"], settings["sim_time"])
    current = tuner.campaign_settings(settings["seed"], settings["sim_time"], settings["trials_target"],
                                      settings["startup_trials"], Path(settings["sumo_cfg"]), reference_path)
    changed = sorted(k for k in settings if k != "code_fingerprint" and settings[k] != current[k])
    old, new = settings["code_fingerprint"], current["code_fingerprint"]
    files = sorted(k for k in set(old) | set(new) if old.get(k) != new.get(k))
    checks.add("a resume would be accepted: settings and code/input fingerprint unchanged",
               not changed and not files, f"settings {changed}, files {files}" if changed or files
               else f"{len(new)} files fingerprinted")
    checks.add("no code change was ever accepted into this campaign", "code_changes" not in db["attrs"],
               str(db["attrs"].get("code_changes", "")))
    digest = tuner._digest(old)

    # ---------------------------------------------------------------- the trials
    trials = db["trials"]
    numbers = [t["number"] for t in trials]
    checks.add("trial numbers are contiguous from 0, none reused", numbers == list(range(len(trials))), str(numbers))
    trials_dir = folder / "trials"
    folders = sorted(p.name for p in trials_dir.iterdir()) if trials_dir.is_dir() else []
    orphan_folders = sorted(set(folders) - {tuner.trial_folder(folder, n).name for n in numbers})
    checks.add("every trial folder belongs to a database trial", not orphan_folders, str(orphan_folders))

    counted, rows, failed_trials = [], [], []
    for trial in trials:
        n, state = trial["number"], trial["state"]
        tfolder = tuner.trial_folder(folder, n)
        reason = trial["attrs"].get("fail_reason")
        is_counted = state == "COMPLETE" or (state == "FAIL" and reason == "rejected")
        if is_counted:
            counted.append(trial)
        if state == "COMPLETE":
            detail = check_complete_trial(trial, tfolder, settings, reference, digest, checks)
            if detail:
                m = detail["metrics"]
                rows.append({"trial": n, "J": trial["value"], "finished": m["finished"],
                             "scheduled": m["vehicles_scheduled"], "travel_s": m["travel_s"],
                             "travel_s_completed": m["reported_travel_s_completed"],
                             "time_loss_s": m["time_loss_s"], "incomplete_share": m["incomplete_share"],
                             "teleports": m["teleports"],
                             "reroutes": detail["record"]["coordinator"]["reroute_executed"],
                             "yields": detail["record"]["coordinator"]["yield_actions"],
                             "responses": {e: (r["value_s"], r["status"]) for e, r in m["responses"].items()},
                             "relative": detail["record"]["objective"]["relative"],
                             "weights": trial["attrs"]["weights"],
                             "wall_minutes": None})
        elif state == "FAIL":
            info = {"trial": n, "fail_reason": reason, "detail": trial["attrs"].get("fail_detail")}
            record_path = tfolder / "trial.json"
            if record_path.is_file():
                # A rejected run still wrote its measurements; its J never enters the study.
                record = json.loads(record_path.read_text(encoding="utf-8"))
                info.update({
                    "J_recorded_not_counted": (record.get("objective") or {}).get("J"),
                    "finished": record["metrics"]["finished"],
                    "summary_arrived": record["metrics"]["summary_arrived"],
                    "teleports": record["metrics"]["teleports"],
                    "exceptions_logged": len(record["exceptions_logged"]),
                    "removed_by_teleport_past_destination": [
                        r.get("id") for r in ET.parse(tfolder / "tripinfo.xml").getroot().iter("tripinfo")
                        if r.get("vaporized") == "teleport"] if (tfolder / "tripinfo.xml").is_file() else None,
                    "weights": record["weights_requested"],
                })
            failed_trials.append(info)
            state_file = tfolder / "TRIAL_STATE.json"
            if state_file.is_file():
                mirrored = json.loads(state_file.read_text(encoding="utf-8"))
                checks.add(f"trial {n}: FAIL mirrored, not a result",
                           mirrored["state"] == "FAIL" and mirrored["counts_as_a_result"] is False)
        elif state == "RUNNING":
            live = tuner_running and n == numbers[-1]
            checks.add(f"trial {n}: RUNNING " + ("(the live trial)" if live else
                       "with no tuner running - a resume marks it interrupted and re-runs its weights"),
                       True)
        else:
            checks.add(f"trial {n}: unexpected state {state}", False)
    for row in rows:
        trial = trials[row["trial"]]
        if trial["start"] and trial["end"]:
            fmt = "%Y-%m-%d %H:%M:%S.%f"
            row["wall_minutes"] = round((datetime.strptime(trial["end"], fmt)
                                         - datetime.strptime(trial["start"], fmt)).total_seconds() / 60, 1)

    indices = [t["attrs"].get("evaluation_index") for t in counted]
    seeds_ok = all(t["attrs"].get("sampler_seed") == settings["seed"] + t["attrs"].get("evaluation_index", -1)
                   for t in counted)
    checks.add("counted trials are evaluations 0..k-1 in order, each with sampler seed = 20260721 + index",
               indices == list(range(len(counted))) and seeds_ok, str(indices))

    complete = [t for t in trials if t["state"] == "COMPLETE"]
    best = min(complete, key=lambda t: t["value"]) if complete else None
    progress_path = folder / "progress.json"
    if progress_path.is_file():
        progress = json.loads(progress_path.read_text(encoding="utf-8"))
        pbest = progress.get("provisional_best") or {}
        checks.add("progress.json agrees with the database",
                   progress["trials_evaluated"] == len(counted) and progress["trials_complete"] == len(complete)
                   and (best is None or (pbest.get("trial") == best["number"] and pbest.get("value") == best["value"])))
    else:
        checks.add("progress.json present", False)
    with open(folder / "e3_trials.csv", newline="", encoding="utf-8") as handle:
        csv_rows = list(csv.DictReader(handle))
    # The tuner refreshes the CSV when a trial ends, so while it runs the live trial has no row yet.
    settled = [t for t in trials if not (tuner_running and t["state"] == "RUNNING" and t["number"] == numbers[-1])]
    checks.add("e3_trials.csv has one row per database trial, same states and values",
               [int(r["trial"]) for r in csv_rows] == [t["number"] for t in settled]
               and all(r["state"] == t["state"] and (t["value"] is None or float(r["value"]) == t["value"])
                       for r, t in zip(csv_rows, settled)))
    target_reached = len(counted) >= settings["trials_target"]
    checks.add("best_e3_weights.json exists only once the target is reached",
               (folder / "best_e3_weights.json").is_file() == target_reached)
    events = [json.loads(line) for line in
              (folder / "campaign_events.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    completes = [e["trial"] for e in events if e["event"] == "trial_complete"]
    checks.add("campaign_events.jsonl records every COMPLETE trial once, and no trial error",
               completes == [t["number"] for t in complete]
               and not any(e["event"] == "trial_error" for e in events), f"{len(events)} events")
    logs = sorted((folder / "console_logs").glob("*")) if (folder / "console_logs").is_dir() else []
    bad = [p.name for p in logs if has_traceback(p)]
    checks.add("no traceback in any console log", not bad, f"{len(logs)} logs; traceback in {bad}" if bad
               else f"{len(logs)} logs")

    errors = [f for f in failed_trials if f["fail_reason"] == "error"]
    safe = (not checks.failed and not tuner_running and not target_reached and not errors)
    status = {
        "campaign": str(folder), "checked_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "tuner_running": tuner_running,
        "trials_target": settings["trials_target"], "trials_evaluated": len(counted),
        "trials_complete": len(complete), "failed_trials": failed_trials,
        "provisional_best": None if best is None else {
            "trial": best["number"], "J": best["value"], "weights": best["attrs"]["weights"]},
        "J_range": None if not complete else [min(t["value"] for t in complete), max(t["value"] for t in complete)],
        "baseline": {k: reference["metrics"][k] for k in ("finished", "vehicles_scheduled", "travel_s",
                                                          "time_loss_s", "teleports")},
        "trials": rows,
        "checks_passed": len(checks.rows) - len(checks.failed), "checks_total": len(checks.rows),
        "checks": checks.rows,
        "safe_to_resume": safe,
        "note": "PROVISIONAL: the tuning seed is only the training environment; this best is not a thesis "
                "result, and its weights are never loaded before the campaign completes.",
    }

    print(f"Campaign {folder} - study {db['studies']} - tuner running: {'YES' if tuner_running else 'no'}")
    print(f"Trials evaluated {len(counted)} of {settings['trials_target']} (complete {len(complete)}, "
          f"rejected {sum(1 for f in failed_trials if f['fail_reason'] == 'rejected')}); "
          f"not counted: {[f for f in failed_trials if f['fail_reason'] != 'rejected'] or 'none'}")
    if best is not None:
        print(f"Provisional best (NOT a result): trial {best['number']}, J = {best['value']:.4f}")
        print("  weights: " + ", ".join(f"{k} {v:.4f}" for k, v in best["attrs"]["weights"].items()))
        print(f"J range over complete trials: {status['J_range'][0]:.4f} - {status['J_range'][1]:.4f}")
    b = reference["metrics"]
    print(f"\nbaseline: finished {b['finished']}/{b['vehicles_scheduled']} ({b['finished'] / b['vehicles_scheduled']:.1%}), "
          f"travel {b['travel_s']:.2f} s censored, time loss {b['time_loss_s']:.2f} s, teleports {b['teleports']}")
    print(f"{'trial':>5} {'J':>7} {'finished':>15} {'travel(cens)':>12} {'time loss':>9} {'telep':>5} "
          f"{'reroutes':>8} {'yields':>6} {'resp 404/401/405':>18} {'wall min':>8}")
    for r in rows:
        resp = "/".join(f"{r['responses'][e][0]:.0f}" for e in ("404", "401", "405") if e in r["responses"])
        print(f"{r['trial']:>5} {r['J']:>7.4f} {r['finished']:>5}/{r['scheduled']} ({r['finished'] / r['scheduled']:.1%})"
              f" {r['travel_s']:>12.2f} {r['time_loss_s']:>9.2f} {r['teleports']:>5} {r['reroutes']:>8} "
              f"{r['yields']:>6} {resp:>18} {r['wall_minutes']!s:>8}")
    print(f"\n{'trial':>5} " + " ".join(f"{k:>10}" for k in ("rel_travel", "rel_resp", "rel_tloss", "rel_incomp", "rel_telep"))
          + "   weights pheromone/heuristic/protocol/bco/velocity")
    for r in rows:
        rel = r["relative"]
        print(f"{r['trial']:>5} " + " ".join(f"{rel[k]:>10.4f}" for k in ("travel", "response", "time_loss",
                                                                          "incomplete", "teleports"))
              + "   " + "/".join(f"{r['weights'][k]:.3f}" for k in tuner.WEIGHT_NAMES))
    if failed_trials:
        print("\nFailed trials (never results; a rejected run's J is kept in its trial.json only):")
        for f in failed_trials:
            line = f"  trial {f['trial']}: {f['fail_reason']} - {f['detail']}"
            if f.get("J_recorded_not_counted") is not None:
                line += (f"; J would have been {f['J_recorded_not_counted']:.4f}; finished {f['finished']} vs "
                         f"summary arrived {f['summary_arrived']}; teleports {f['teleports']}; exceptions "
                         f"{f['exceptions_logged']}; removed by a teleport past the destination: "
                         f"{f['removed_by_teleport_past_destination']}")
            print(line)
        recorded = [f["J_recorded_not_counted"] for f in failed_trials if f.get("J_recorded_not_counted") is not None]
        if recorded and best is not None:
            beats = [j for j in recorded if j < best["value"]]
            status["rejected_trial_would_beat_best"] = bool(beats)
            print(f"  would any rejected trial have beaten the provisional best? {'YES' if beats else 'no'}")
    print(f"\nIntegrity: {status['checks_passed']} of {status['checks_total']} checks pass")
    for row in checks.failed:
        print(f"  FAIL  {row['check']}  {row['detail']}")
    print(f"Safe to resume: {'YES' if safe else 'NO'}" + ("" if safe else
          " (" + "; ".join(x for x in ["integrity check failed" if checks.failed else "",
                                         "a tuner is running" if tuner_running else "",
                                         "target already reached" if target_reached else "",
                                         "a trial hit an execution error" if errors else ""] if x) + ")"))
    if save:
        out = folder / "status_checks"
        out.mkdir(exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = out / f"status_{len(counted):02d}_of_{settings['trials_target']}_{stamp}.json"
        path.write_text(json.dumps(status, indent=1, default=str), encoding="utf-8")
        print(f"saved {path}")
    return 0 if not checks.failed else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("campaign", type=Path, help="the campaign folder")
    parser.add_argument("--save", action="store_true", help="also write the status JSON into <campaign>/status_checks/")
    args = parser.parse_args()
    if not (args.campaign / tuner.STORAGE_FILE).is_file():
        parser.error(f"no campaign in {args.campaign}")
    return run(args.campaign, args.save)


if __name__ == "__main__":
    raise SystemExit(main())
