#!/usr/bin/env python3
"""Are two staged_trial_check.py runs the same run? (2026-09-14)

    python scripts/diagnostics/compare_staged_runs.py RUN_A RUN_B

Written to check that the production A2 rule ("mean protected") reproduces the diagnosis run that
emulated it. Compared, exactly:
- J and every metric in trial.json;
- the coordinator and outcome-tracker counters;
- every tripinfo record, attribute by attribute (the file header, which holds a timestamp and the
  output paths, is not compared);
- every summary.xml step, except its `duration`, which is SUMO's wall-clock milliseconds for the step;
- every SUMO warning and error line;
- learning.json: the run settings, the running mean, the pheromone and PSO table sizes, and the probe
  (outcome quality by how the window closed, deposits by source, which channels early outcomes
  reached, the 300 s timeline).

Also reported, not required to match: `outcomes_applied_to_learning`. The diagnosis emulation filed
an early outcome's fitness without the counter, so there it is measured minus early; production counts
every outcome it files.

Reads only. Exit 0 when everything required matches, 1 otherwise. Nothing here is a thesis result.
"""

from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# The variant name differs by design; the two outcome counters are explained in the docstring.
NOT_REQUIRED = {"outcomes_applied_to_learning", "outcomes_in_running_mean", "a2_variant"}


def records(path: Path, tag: str, ignore=()) -> list:
    out = []
    for element in ET.parse(path).getroot().iter(tag):
        children = [(child.tag, dict(child.attrib)) for child in element]
        out.append(({k: v for k, v in element.attrib.items() if k not in ignore}, children))
    return out


def sumo_lines(path: Path) -> list:
    return [line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.startswith(("Warning", "Error"))]


def tracebacks(run: Path) -> int:
    return sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback")
               for p in (run / "run.err", run / "trial" / "errors.log", run / "trial" / "messages.log") if p.is_file())


def main() -> int:
    a, b = Path(sys.argv[1]), Path(sys.argv[2])
    failures, notes = [], []

    def same(label, x, y, required=True):
        if x == y:
            print(f"  same      {label}")
            return
        (failures if required else notes).append(label)
        print(f"  {'DIFFERENT' if required else 'differs  '} {label}: {str(x)[:120]} | {str(y)[:120]}")

    print(f"A: {a}\nB: {b}")
    for run in (a, b):
        count = tracebacks(run)
        if count:
            failures.append(f"tracebacks in {run}")
        print(f"  tracebacks in {run.name}: {count}")

    ta, tb = (json.loads((r / "trial" / "trial.json").read_text(encoding="utf-8")) for r in (a, b))
    same("J", ta["objective"]["J"], tb["objective"]["J"])
    same("objective terms", ta["objective"], tb["objective"])
    for key in sorted(set(ta["metrics"]) | set(tb["metrics"])):
        same(f"metrics.{key}", ta["metrics"].get(key), tb["metrics"].get(key))
    same("coordinator counters", ta["coordinator"], tb["coordinator"])
    same("outcome tracker counters", ta["learning"]["outcomes"], tb["learning"]["outcomes"])
    same("pheromone table entries", ta["learning"]["pheromone_table_entries"], tb["learning"]["pheromone_table_entries"])
    same("route stats", ta["route_stats"], tb["route_stats"])
    same("weights used", ta["weights_used"], tb["weights_used"])
    same("exceptions logged", ta["exceptions_logged"], tb["exceptions_logged"])

    ra, rb = (records(r / "trial" / "tripinfo.xml", "tripinfo") for r in (a, b))
    differing = sum(1 for x, y in zip(ra, rb) if x != y) + abs(len(ra) - len(rb))
    same(f"tripinfo records ({len(ra)} and {len(rb)}; {differing} differ)", ra, rb)
    sa, sb = (records(r / "trial" / "summary.xml", "step", ignore=("duration",)) for r in (a, b))
    same(f"summary steps ({len(sa)} and {len(sb)}; wall-clock duration not compared)", sa, sb)
    same("SUMO warning and error lines", sumo_lines(a / "trial" / "errors.log"), sumo_lines(b / "trial" / "errors.log"))

    la, lb = (json.loads((r / "learning.json").read_text(encoding="utf-8")) for r in (a, b))
    for key in sorted(set(la) | set(lb)):
        if key == "probe":
            continue
        same(f"learning.{key}", la.get(key), lb.get(key), required=key not in NOT_REQUIRED)
    pa, pb = la.get("probe", {}), lb.get("probe", {})
    for key in sorted(set(pa) | set(pb)):
        same(f"probe.{key}", pa.get(key), pb.get(key), required=key not in NOT_REQUIRED)

    print(f"\n{'IDENTICAL' if not failures else 'NOT IDENTICAL'}: {len(failures)} required field(s) differ"
          + (f" ({', '.join(failures)})" if failures else "")
          + (f"; expected differences: {', '.join(notes)}" if notes else ""))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
