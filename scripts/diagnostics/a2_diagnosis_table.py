#!/usr/bin/env python3
"""Tabulate an A2 diagnosis folder written by staged_trial_check.py (2026-09-14).

    python scripts/diagnostics/a2_diagnosis_table.py results/e3_component_validation_post_audit_fixes_2026-09-14/a2_diagnosis_deterministic_pso

Each subfolder is one run, named <weights>_<variant>[_butterfly]. Printed per run:
- J with finished trips, teleports and never-inserted;
- reroutes;
- outcomes measured, closed by a new decision, and applied;
- mean outcome quality by how the window closed, and the final running mean;
- pheromone saturation;
- deposits by source, with their mean strength.

Also printed: J minus the `dropped` run of the same weights, and each butterfly run against its
base run. Writes table.json into the folder. Reads only. Nothing here is a thesis result.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def load(run: Path) -> dict | None:
    if not (run / "comparison.json").is_file():
        return None
    c = json.loads((run / "comparison.json").read_text(encoding="utf-8"))
    learning = json.loads((run / "learning.json").read_text(encoding="utf-8"))
    a = {k: v["after"] for k, v in c["table"].items()}
    probe = learning.get("probe", {})
    quality = {reason: round(total / count, 4) for reason, (count, total) in probe.get("quality_by_reason", {}).items()
               if count}
    deposits = {src: {"count": d["count"], "mean_strength": round(d["strength_sum"] / d["count"], 4) if d["count"] else None,
                      "mean_edges": round(d["edges_sum"] / d["count"], 1) if d["count"] else None}
                for src, d in probe.get("deposits", {}).items()}
    tracebacks = (run / "run.err").read_text(encoding="utf-8", errors="replace").count("Traceback") \
        if (run / "run.err").is_file() else None
    return {"run": run.name, "J": a["J"], "finished": a["finished"], "teleports": a["teleports"],
            "never_inserted": a["never_inserted"], "travel_s": a["travel_s"], "time_loss_s": a["time_loss_s"],
            "responses_s": a["responses_s"], "reroutes": a["reroute_executed"], "decisions": a["reroute_decisions"],
            "outcomes_measured": a["outcomes_reported"], "closed_by_new_decision": a["settled_by_new_decision"],
            "applied_via_record_outcome": learning["outcomes_applied_to_learning"],
            "early_channels": probe.get("early_channels"), "quality_by_reason": quality,
            "quality_mean_final": round(learning["quality_mean"], 4),
            "pheromone_at_ceiling": learning.get("pheromone_at_ceiling"),
            "pheromone_mean": round(learning["pheromone_mean"], 3) if learning.get("pheromone_mean") else None,
            "deposits": deposits, "timeline": probe.get("timeline"), "butterfly": c.get("butterfly"),
            "rejected": c["rejected"], "exceptions": len(c["exceptions_logged"]), "tracebacks": tracebacks}


def main() -> int:
    folder = Path(sys.argv[1])
    runs = [r for r in (load(p) for p in sorted(folder.iterdir()) if p.is_dir()) if r is not None]
    by_name = {r["run"]: r for r in runs}
    for r in runs:
        weights = r["run"].split("_")[0]
        base = by_name.get(f"{weights}_dropped")
        r["J_minus_dropped"] = round(r["J"] - base["J"], 4) if base and not r["butterfly"] else None
        if r["butterfly"]:
            origin = by_name.get(r["run"].replace("_butterfly", ""))
            r["J_minus_unperturbed"] = round(r["J"] - origin["J"], 4) if origin else None
    print(f"{'run':36} {'J':>7} {'dJ vs drop':>10} {'fin':>5} {'tel':>4} {'noins':>5} {'rerou':>5} {'meas':>5} "
          f"{'early':>5} {'q horizon':>9} {'q new-dec':>9} {'q-mean':>7} {'tau@max':>7} {'dep h/l':>14} {'dep new':>14}")
    for r in runs:
        q = r["quality_by_reason"]
        d = r["deposits"]
        dh, dn = d.get("horizon_or_leaving", {}), d.get("new_decision", {})
        delta = r["J_minus_dropped"] if r["J_minus_dropped"] is not None else (
            f"bfly {r.get('J_minus_unperturbed')}" if r["butterfly"] else "")
        print(f"{r['run']:36} {r['J']:7.4f} {str(delta):>10} {r['finished']:5} {r['teleports']:4} {r['never_inserted']:5} "
              f"{r['reroutes']:5} {r['outcomes_measured']:5} {r['closed_by_new_decision']:5} "
              f"{str(q.get('horizon')):>9} {str(q.get('new_decision')):>9} {r['quality_mean_final']:7.3f} "
              f"{str(r['pheromone_at_ceiling']):>7} {str((dh.get('count'), dh.get('mean_strength'))):>14} "
              f"{str((dn.get('count'), dn.get('mean_strength'))):>14}"
              + ("  REJECTED" if r["rejected"] else "") + (f"  TRACEBACKS {r['tracebacks']}" if r["tracebacks"] else ""))
    (folder / "table.json").write_text(json.dumps(runs, indent=1, default=str), encoding="utf-8")
    print(f"written: {folder / 'table.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
