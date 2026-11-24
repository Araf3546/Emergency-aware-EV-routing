#!/usr/bin/env python3
"""Baseline-fairness and experimental-consistency check before the tuning campaign (2026-09-14).

    python scripts/diagnostics/baseline_fairness_check.py --out results/baseline_fairness_2026-09-14

Cheap: two 1,250 s runs in parallel, plus comparisons of files that already exist. It runs the
REAL baseline function (`tune_e3_weights.build_reference`, which builds every tuning reference) and
the REAL tuning trial function (`tune_e3_weights.evaluate_weights`, which every campaign trial calls,
here with DEFAULT_WEIGHTS) on the tuning seed. The only thing added to them is a recorder that, after
each `traci.simulationStep`, reads each departing vehicle's type, class, speed factor and route with
TraCI getters. Getters draw no random numbers and change no vehicle.

It checks:
1. both arms load exactly the same SUMO configuration and input files. Read from the effective
   configuration SUMO writes into every output file, for the canonical references, these runs and
   earlier E3 trials, and compared apart from output paths and the end time;
2. every vehicle that departs in both arms has the same type, class, speed factor, route and
   scheduled departure, and matches the demand files. Ordinary cars carry their frozen speed
   factor; emergency vehicles carry their type's fixed 1.2;
3. the corrected PD-1 permissions hold in both arms (ambient `authority`, responders
   `emergency`, cars `passenger`), and E3 keeps every vehicle's destination;
4. the canonical baseline is reproducible. This baseline run equals the canonical 1,250 s
   reference, whose first 1,250 steps equal those of the canonical 3,600 s reference. The old
   pre-correction reference is refused by the tuner;
5. the objective is unchanged, and the arms differ in the treatment: E3 reroutes, the baseline
   never does.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402

SEED, T = 20260721, 1250
REF_3600 = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s"
REF_1250 = REPO_ROOT / "results/tuning_reference/seed_20260721_1250s"
OLD_REF = REPO_ROOT / "results/_invalid/superseded_tuning_reference_pre_pd1_q2_2026-09-13/seed_20260721_3600s"
E3_TRIALS = [REPO_ROOT / "results/campaign_persistence_smoke_2026-09-13/chunked/trials/trial_000",
             REPO_ROOT / "results/campaign_persistence_smoke_2026-09-13/continuous/trials/trial_003",
             REPO_ROOT / "results/final_sanity_pd1q2_2026-09-13/trial_300s"]


def kind(vehicle_id: str) -> str:
    if vehicle_id.startswith("emergency_dispatch_"):
        return "responder"
    if vehicle_id.startswith("emergency_"):
        return "ambient"
    return "car"


def sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# ------------------------------------------------------------------ the two runs


def run(arm: str, out: Path) -> None:
    import scripts.tune_e3_weights as tuner
    from p3_hybrid_router import DEFAULT_WEIGHTS

    out.mkdir(parents=True, exist_ok=True)
    departed = {}
    original_step = traci.simulationStep

    def recording_step(*args, **kwargs):
        result = original_step(*args, **kwargs)
        t = traci.simulation.getTime()
        for v in traci.simulation.getDepartedIDList():
            departed[v] = {"t": t, "type": traci.vehicle.getTypeID(v), "class": traci.vehicle.getVehicleClass(v),
                           "speed_factor": traci.vehicle.getSpeedFactor(v), "route": list(traci.vehicle.getRoute(v))}
        return result

    traci.simulationStep = recording_step
    record = {"arm": arm}
    if arm == "baseline":
        reference = tuner.build_reference(Path("simulation.sumocfg"), SEED, T, out)
        record["reference_kind"] = reference["kind"]
        from algorithms.route_utils import get_route_stats
        record["route_stats"] = {k: v for k, v in get_route_stats().items() if isinstance(v, (int, float))}
    else:
        reference = tuner.load_reference(REF_1250 / "reference.json", SEED, T)
        result = tuner.evaluate_weights(dict(DEFAULT_WEIGHTS), Path("simulation.sumocfg"), T, SEED,
                                        trial_dir=out, reference=reference)
        record["objective"] = result.objective
        record["coordinator"] = result.checks["coordinator"]
        record["route_stats"] = result.checks["route_stats"]
        record["pairing"] = result.checks["pairing"]
    record["departed"] = departed
    (out / "capture.json").write_text(json.dumps(record, indent=1), encoding="utf-8")


# ------------------------------------------------------------------ comparisons


def effective_config(output_file: Path) -> dict:
    """The configuration SUMO actually ran, from the comment it writes at the top of every output."""
    text = Path(output_file).read_text(encoding="utf-8", errors="replace")[:20000]
    body = text[text.index("<!--") + 4:text.index("-->")]
    body = body[body.index("<", 1):] if body.lstrip().startswith("generated") else body
    body = body[body.index("<"):]
    root = ET.fromstring(body)
    options = {}
    for section in root:
        for option in section:
            options[f"{section.tag}/{option.tag}"] = option.get("value")
    return options


def run_options(options: dict) -> dict:
    """Everything that can influence the simulation: drop where outputs go, when it ends, and the
    TraCI port (a free port picked per run; it carries the connection, not the scenario)."""
    return {k: v for k, v in options.items()
            if not (k.startswith("output/") or k.endswith("-log") or k == "time/end"
                    or k == "traci_server/remote-port")}


def sumo_version(output_file: Path) -> str:
    text = Path(output_file).read_text(encoding="utf-8", errors="replace")[:400]
    return text[text.index("by Eclipse SUMO"):].splitlines()[0].strip()


def demand() -> dict:
    """vehicle id -> (type, scheduled depart, route edges, explicit speedFactor) from the route files."""
    from evaluation_seeds import sumo_seed_args
    from scripts.tune_e3_weights import _route_files

    vehicles = {}
    for path in _route_files(sumo_seed_args(SEED, Path("simulation.sumocfg"))):
        root = ET.parse(path).getroot()
        routes = {r.get("id"): r.get("edges").split() for r in root.iter("route") if r.get("id")}
        for v in root.iter("vehicle"):
            r = v.find("route")
            edges = r.get("edges").split() if r is not None else routes[v.get("route")]
            vehicles[v.get("id")] = {"type": v.get("type"), "depart": float(v.get("depart")), "route": edges,
                                     "speed_factor": None if v.get("speedFactor") is None else float(v.get("speedFactor"))}
    return vehicles


def tripinfo(path: Path) -> dict:
    return {t.get("id"): dict(t.attrib) for t in ET.parse(path).getroot().iter("tripinfo")}


def summary_steps(path: Path, before: float) -> list:
    return [{k: v for k, v in s.attrib.items() if k != "duration"}
            for s in ET.parse(path).getroot().iter("step") if float(s.get("time")) < before]


def compare(out: Path) -> dict:
    from emergency_manager import ROAD_PERMISSION_POLICY
    from metrics.tuning_objective import OBJECTIVE_WEIGHTS
    from scripts.tune_e3_weights import SCENARIO_FILE, load_reference

    base = json.loads((out / "baseline" / "capture.json").read_text(encoding="utf-8"))
    e3 = json.loads((out / "e3" / "capture.json").read_text(encoding="utf-8"))
    ref3600 = json.loads((REF_3600 / "reference.json").read_text(encoding="utf-8"))
    ref1250 = json.loads((REF_1250 / "reference.json").read_text(encoding="utf-8"))
    checks, facts = {}, {}

    # 1. the same configuration and input files
    configs = {"canonical reference 3,600 s": REF_3600 / "tripinfo.xml",
               "reference 1,250 s": REF_1250 / "tripinfo.xml",
               "this baseline run": out / "baseline" / "tripinfo.xml",
               "this E3 trial": out / "e3" / "tripinfo.xml",
               **{f"E3 trial {p.parent.parent.name}/{p.name}": p / "tripinfo.xml" for p in E3_TRIALS}}
    options = {name: run_options(effective_config(path)) for name, path in configs.items()}
    first = options["canonical reference 3,600 s"]
    differing = {name: {k: (first.get(k), o.get(k)) for k in set(first) | set(o) if first.get(k) != o.get(k)}
                 for name, o in options.items()}
    checks["every baseline and E3 run ran the same SUMO configuration (all options but outputs and end)"] = all(
        not d for d in differing.values())
    facts["configuration differences against the canonical reference"] = differing
    versions = {name: sumo_version(path) for name, path in configs.items()}
    checks["every run was made by the same SUMO build"] = len(set(versions.values())) == 1
    facts["SUMO build"] = sorted(set(versions.values()))
    facts["compared options"] = {k: first[k] for k in sorted(first)}
    route_files = first["input/route-files"].split(",")
    files = [first["input/net-file"], *route_files]
    facts["input files (sha256)"] = {f: sha(REPO_ROOT / f if not Path(f).is_absolute() else Path(f)) for f in files}
    checks["frozen demand and incident files match what the canonical reference recorded"] = (
        ref3600["frozen_demand_sha256"] == sha(Path(ref3600["frozen_demand"]))
        and ref3600["scenario_sha256"] == sha(SCENARIO_FILE))

    # 2. the same population, vehicle for vehicle
    plan = demand()
    both = sorted(set(base["departed"]) & set(e3["departed"]))
    facts["vehicles departed (baseline, E3, both)"] = (len(base["departed"]), len(e3["departed"]), len(both))
    same = {field: [v for v in both if base["departed"][v][field] != e3["departed"][v][field]]
            for field in ("type", "class", "speed_factor", "route")}
    checks["every vehicle departing in both arms: same type, class, speed factor and route"] = all(
        not v for v in same.values()) and len(both) > 1000
    facts["vehicles differing between arms, by field"] = {k: v[:5] for k, v in same.items()}
    mismatch_plan = []
    for arm, capture in (("baseline", base), ("E3", e3)):
        for v, rec in capture["departed"].items():
            p = plan.get(v)
            if p is None or rec["type"] != p["type"] or rec["route"] != p["route"] or rec["t"] < p["depart"] - 1e-9:
                mismatch_plan.append((arm, v))
    checks["every departing vehicle matches the demand files (type, full route, not before its scheduled time)"] = (
        not mismatch_plan)
    facts["vehicles not matching the demand files"] = mismatch_plan[:5]
    frozen = json.loads(Path(REPO_ROOT / "demand/seeds/porto.speed_factors.20260721.json").read_text(encoding="utf-8"))["speed_factors"]
    sf = {"car: equals its frozen value": [], "ambient/responder: equals 1.2": []}
    for arm, capture in (("baseline", base), ("E3", e3)):
        for v, rec in capture["departed"].items():
            if kind(v) == "car":
                if rec["speed_factor"] != float(frozen[v]) or rec["speed_factor"] != plan[v]["speed_factor"]:
                    sf["car: equals its frozen value"].append((arm, v))
            elif rec["speed_factor"] != 1.2:
                sf["ambient/responder: equals 1.2"].append((arm, v, rec["speed_factor"]))
    checks["speed factors: every car its frozen value, every emergency vehicle 1.2, in both arms"] = all(not x for x in sf.values())
    facts["speed-factor exceptions"] = sf
    facts["tuner's own pairing check, E3 trial"] = e3["pairing"]
    trips_b, trips_e = tripinfo(out / "baseline" / "tripinfo.xml"), tripinfo(out / "e3" / "tripinfo.xml")
    type_diff = [v for v in set(trips_b) & set(trips_e) if trips_b[v]["vType"] != trips_e[v]["vType"]]
    sched_diff = [v for arm_trips in (trips_b, trips_e) for v, r in arm_trips.items()
                  if abs(float(r["depart"]) - float(r["departDelay"]) - plan[v]["depart"]) > 1e-6]
    checks["tripinfo agrees: same vType in both arms, scheduled departure = the demand file's"] = not type_diff and not sched_diff

    # 3. permissions and destinations
    expected = {"responder": ["emergency"], "ambient": ["authority"], "car": ["passenger"]}
    classes = {arm: {k: sorted({r["class"] for v, r in cap["departed"].items() if kind(v) == k}) for k in expected}
               for arm, cap in (("baseline", base), ("E3", e3))}
    facts["departure classes"] = classes
    checks["corrected PD-1 permissions in the baseline (ambient authority, responders emergency, cars passenger)"] = (
        classes["baseline"] == expected)
    checks["corrected PD-1 permissions in the E3 trial"] = classes["E3"] == expected
    e3_trial = json.loads((out / "e3" / "trial.json").read_text(encoding="utf-8"))
    trial_policies = [json.loads((p / "trial.json").read_text(encoding="utf-8")) for p in E3_TRIALS]
    checks["canonical reference and every E3 trial record the same road-permission rule and switched all four types"] = all(
        doc.get("road_permissions") == ROAD_PERMISSION_POLICY and sorted(doc["ambient_road_classes"].values()) == ["authority"] * 4
        for doc in (ref3600, ref1250, e3_trial, *trial_policies))
    lost = [v for arm_trips in (trips_b, trips_e) for v, r in arm_trips.items()
            if float(r["arrival"]) >= 0 and r["arrivalLane"].rsplit("_", 1)[0] != plan[v]["route"][-1]]
    checks["every finished vehicle, in both arms, arrived at its demand destination (same OD)"] = not lost
    facts["finished vehicles (baseline, E3)"] = (sum(float(r["arrival"]) >= 0 for r in trips_b.values()),
                                                sum(float(r["arrival"]) >= 0 for r in trips_e.values()))

    # 4. the canonical baseline is reproducible, and the old one is not usable
    ref_trips = tripinfo(REF_1250 / "tripinfo.xml")
    checks["this baseline run equals the canonical 1,250 s reference (every trip record, every summary step)"] = (
        trips_b == ref_trips
        and summary_steps(out / "baseline" / "summary.xml", T) == summary_steps(REF_1250 / "summary.xml", T))
    long_trips = {k: v for k, v in tripinfo(REF_3600 / "tripinfo.xml").items() if 0 <= float(v["arrival"]) < T}
    short_trips = {k: v for k, v in ref_trips.items() if 0 <= float(v["arrival"]) < T}
    checks["the canonical 3,600 s reference equals the 1,250 s one over its first 1,250 s (summary steps, finished trips)"] = (
        summary_steps(REF_3600 / "summary.xml", T) == summary_steps(REF_1250 / "summary.xml", T) and long_trips == short_trips)
    try:
        load_reference(OLD_REF / "reference.json", SEED, 3600)
        refused = False
    except ValueError:
        refused = True
    checks["the pre-correction reference (3,311 finished) is refused by the tuner"] = refused
    old = json.loads((OLD_REF / "reference.json").read_text(encoding="utf-8"))
    checks["old and new references differ only in the road-permission record (same seed, T, demand, scenario, objective)"] = (
        {k: old[k] for k in ("seed", "sim_time", "frozen_demand_sha256", "scenario_sha256", "objective_weights")}
        == {k: ref3600[k] for k in ("seed", "sim_time", "frozen_demand_sha256", "scenario_sha256", "objective_weights")}
        and "road_permissions" not in old)
    old_steps = summary_steps(OLD_REF / "summary.xml", 3600)
    new_steps = summary_steps(REF_3600 / "summary.xml", 3600)
    facts["old vs canonical 3,600 s reference: first differing step"] = next(
        (a["time"] for a, b in zip(old_steps, new_steps) if a != b), None)

    # 5. objective unchanged; the treatment is the only difference
    checks["objective weights are the finalized 13.17 values"] = OBJECTIVE_WEIGHTS == {
        "travel": 0.35, "response": 0.30, "time_loss": 0.2625, "incomplete": 0.07, "teleports": 0.0175}
    fingerprint = json.loads((REPO_ROOT / "results/campaign_persistence_smoke_2026-09-13/chunked/campaign.json")
                             .read_text(encoding="utf-8"))["code_fingerprint"]
    current = {k: sha(REPO_ROOT / k) for k in fingerprint}
    changed = sorted(k for k in fingerprint if fingerprint[k] != current[k])
    checks["no code or input file a trial runs has changed since the persistence test (objective file included)"] = not changed
    facts["files changed since the persistence test"] = changed
    facts["metrics/tuning_objective.py sha256"] = current.get("metrics/tuning_objective.py")
    checks["the treatment differs: E3 rerouted vehicles, the baseline rerouted none"] = (
        e3["coordinator"]["reroute_executed"] > 0 and base["route_stats"].get("reroutes_applied", 0) == 0)
    facts["E3 trial: J, reroutes executed, yield actions"] = (e3["objective"], e3["coordinator"]["reroute_executed"],
                                                             e3["coordinator"]["yield_actions"])
    return {"checks": checks, "facts": facts}


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("--arm", choices=("baseline", "e3"), required=True)
    p.add_argument("--out", required=True)
    p = sub.add_parser("all")
    p.add_argument("--out", required=True)
    p = sub.add_parser("verdict", help="re-judge saved runs without simulating")
    p.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    if args.cmd == "run":
        run(args.arm, out)
        return 0
    out.mkdir(parents=True, exist_ok=True)
    if args.cmd == "all":
        procs = []
        for arm in ("baseline", "e3"):
            log = open(out / f"{arm}.log", "w", encoding="utf-8")
            procs.append((subprocess.Popen([sys.executable, __file__, "run", "--arm", arm, "--out", str(out / arm)],
                                           stdout=log, stderr=subprocess.STDOUT), log))
        for proc, log in procs:
            proc.wait()
            log.close()
    tracebacks = [p.name for p in out.glob("*.log") if "Traceback" in p.read_text(encoding="utf-8", errors="replace")]
    result = compare(out)
    checks = {**result["checks"], "no traceback in either run log": not tracebacks}
    ok = all(checks.values())
    (out / "verdict.json").write_text(json.dumps({"pass": ok, "checks": checks, "facts": result["facts"]},
                                                 indent=1, default=str), encoding="utf-8")
    for key, value in checks.items():
        print(f"{'PASS' if value else 'FAIL'}  {key}")
    print("baseline fairness:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
