#!/usr/bin/env python3
"""Does production's unified yield rule reproduce the validated emulation? (2026-09-15)

Diagnostic only, and observation only.

`run` is one real tuner trial of the production code (`tune_e3_weights.evaluate_weights`): the tuning seed,
frozen demand, 3,600 s and all three incidents. Every call of `_yield_to_emergency` is recorded with what it
returned, and so is every broadcast that carried `yield_request`. Nothing is changed.

`check PRODUCTION EMULATED` compares a production run with the emulated run of the same weights
(`results/make_room_yield_2026-09-15/<w>_make_room`, made by `make_room_yield_check.py`):
- **the traffic, exactly:** every tripinfo record, every summary step (SUMO's wall-clock duration left out),
  every SUMO warning, the incident log, J with its terms, every metric, the learning and route counters, and
  the pairing;
- **the coordinator counters:** the same, except `yield_actions`. The emulation counted every response.
  Production counts only moves aside (the semantic fix of 2026-09-15), so it must equal the emulation's number
  of "move_right";
- **the yield responses, one by one:** the same (time, car, action), in the same order;
- **the flag:** every broadcast carrying `yield_request` must come within YIELD_DURATION_S after a move aside by
  the same car.

    python scripts/diagnostics/unified_yield_reproduce.py run --default-weights --out DIR/def_production
    python scripts/diagnostics/unified_yield_reproduce.py check DIR/def_production results/make_room_yield_2026-09-15/def_make_room
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from yield_signal_diagnosis import records, sumo_lines  # noqa: E402  (also chdirs to REPO_ROOT)

REFERENCE = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"
SEED = 20260721


def live_run(args) -> int:
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    import tune_e3_weights as tuner
    import p3_runtime
    import p3_hybrid_router
    from p3_metrics import P3MetricsLogger

    out = Path(args.out)
    trial_dir = out / "trial"
    if trial_dir.exists() and any(trial_dir.iterdir()):
        raise SystemExit(f"{trial_dir} already holds files; evidence is never overwritten")
    out.mkdir(parents=True, exist_ok=True)
    C = p3_runtime.P3RuntimeCoordinator
    holder = {"t": None, "coordinator": None}
    responses, flags = [], []

    real_init = C.__init__

    def init(self, *a, **k):
        real_init(self, *a, **k)
        holder["coordinator"] = self

    C.__init__ = init

    real_update = C.update_step

    def update_step(self, sim_time):
        holder["t"] = sim_time
        return real_update(self, sim_time)

    C.update_step = update_step

    real_yield = C._yield_to_emergency

    def observed_yield(self, vehicle_id, edge_id, responder_id, sim_time):
        action = real_yield(self, vehicle_id, edge_id, responder_id, sim_time)
        responses.append({"t": sim_time, "car": vehicle_id, "edge": edge_id, "responder": responder_id,
                          "action": action})
        return action

    C._yield_to_emergency = observed_yield

    real_broadcast = P3MetricsLogger.record_broadcast

    def observed_broadcast(self, vehicle_id, emergency_flag, yield_request):
        if yield_request:
            flags.append({"t": holder["t"], "car": vehicle_id})
        return real_broadcast(self, vehicle_id, emergency_flag, yield_request)

    P3MetricsLogger.record_broadcast = observed_broadcast

    if args.default_weights:
        weights = {name: float(p3_hybrid_router.DEFAULT_WEIGHTS[name]) for name in tuner.WEIGHT_NAMES}
    else:
        weights = json.loads(Path(args.weights_from).read_text(encoding="utf-8"))["weights_requested"]
    # `--unscored` is for a short tool check only: the reference is for 3,600 s.
    reference = None if args.unscored else tuner.load_reference(REFERENCE, SEED, args.sim_time)
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), args.sim_time, SEED, trial_dir=trial_dir,
                               reference=reference)
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    c = holder["coordinator"]
    actions = {}
    for r in responses:
        actions[str(r["action"])] = actions.get(str(r["action"]), 0) + 1
    summary = {"weights": weights, "rejected": rejected, "responses": len(responses), "actions": actions,
               "flagged_broadcasts": len(flags), "yield_actions_coordinator": c.yield_actions if c else None,
               "yield_requests_metric": c.metrics.yield_requests if c else None}
    (out / "responses.json").write_text(json.dumps(responses), encoding="utf-8")
    (out / "flags.json").write_text(json.dumps(flags), encoding="utf-8")
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(json.dumps(summary))
    return 0 if rejected is None else 1


def check(args) -> int:
    import p3_runtime

    p, e = Path(args.production), Path(args.emulated)
    failures = []

    def same(label, x, y):
        ok = x == y
        if not ok:
            failures.append(label)
        print(f"  {'same     ' if ok else 'DIFFERENT'} {label}" + ("" if ok else f": {str(x)[:100]} | {str(y)[:100]}"))

    tp, te = (json.loads((r / "trial" / "trial.json").read_text(encoding="utf-8")) for r in (p, e))
    for key in ("objective", "metrics", "learning", "route_stats", "exceptions_logged", "pairing"):
        same(key, tp[key], te[key])
    events = json.loads((e / "events.json").read_text(encoding="utf-8"))
    moves = sum(ev.get("action") == "move_right" for ev in events)
    same("coordinator counters other than yield_actions",
         {k: v for k, v in tp["coordinator"].items() if k != "yield_actions"},
         {k: v for k, v in te["coordinator"].items() if k != "yield_actions"})
    same(f"yield_actions: production counts moves aside only ({tp['coordinator']['yield_actions']}); the emulation's "
         f"move_right responses ({moves}; it counted all {te['coordinator']['yield_actions']})",
         tp["coordinator"]["yield_actions"], moves)
    for tag, name, ignore in (("tripinfo", "tripinfo.xml", ()), ("step", "summary.xml", ("duration",))):
        a, b = (records(r / "trial" / name, tag, ignore=ignore) for r in (p, e))
        same(f"{name} records ({len(a)} / {len(b)})", a, b)
    wa, wb = (sumo_lines(r / "trial" / "errors.log") for r in (p, e))
    same(f"SUMO warning lines ({len(wa)} / {len(wb)})", wa, wb)

    def incident_log(run):
        path = run / "run.out"
        return [l for l in path.read_text(encoding="utf-8", errors="replace").splitlines()
                if l.startswith("[EMERGENCY]")] if path.is_file() else None

    same("incident activations and closures", incident_log(p), incident_log(e))
    responses = json.loads((p / "responses.json").read_text(encoding="utf-8"))
    same(f"yield responses, one by one ({len(responses)} / {len(events)})",
         [(r["t"], r["car"], r["action"]) for r in responses], [(v["t"], v["car"], v["action"]) for v in events])
    same("responses SUMO refused (production returned None)", 0, sum(r["action"] is None for r in responses))
    flags = json.loads((p / "flags.json").read_text(encoding="utf-8"))
    moved_at = {}
    for r in responses:
        if r["action"] == "move_right":
            moved_at.setdefault(r["car"], []).append(r["t"])
    stray = [f for f in flags if not any(0.0 < f["t"] - t <= p3_runtime.YIELD_DURATION_S
                                         for t in moved_at.get(f["car"], ()))]
    same(f"flagged broadcasts not within {p3_runtime.YIELD_DURATION_S:.0f} s after a move aside by the same car "
         f"(of {len(flags)} flagged)", 0, len(stray))
    tracebacks = sum(q.read_text(encoding="utf-8", errors="replace").count("Traceback")
                     for q in (p / "run.err", p / "trial" / "errors.log") if q.is_file())
    same("tracebacks in the production run", 0, tracebacks)
    print("REPRODUCED" if not failures else "NOT REPRODUCED: " + ", ".join(failures))
    return 0 if not failures else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="one observed production trial, 3,600 s")
    source = r.add_mutually_exclusive_group(required=True)
    source.add_argument("--weights-from", help="a trial.json whose weights_requested are used")
    source.add_argument("--default-weights", action="store_true")
    r.add_argument("--sim-time", type=int, default=3600)
    r.add_argument("--unscored", action="store_true", help="tool check only: no reference, no J (for short runs)")
    r.add_argument("--out", required=True)
    c = sub.add_parser("check", help="compare a production run with the emulated run of the same weights")
    c.add_argument("production")
    c.add_argument("emulated")
    args = parser.parse_args()
    return {"run": live_run, "check": check}[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
