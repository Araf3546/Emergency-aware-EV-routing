#!/usr/bin/env python3
"""Compare two ab_identity.py output directories: must the change they differ by alter nothing?

Written 2026-09-12 for the incident-distance cache, which must be a performance change only.
Every check below must pass, or the change is not adopted:

  trace               per step: vehicle count, total metres driven, hash of Python's random
                      state (E3's random stream)
  speed_factors       every vehicle's SUMO speed factor when first seen, exactly
  decisions           every routing decision: time, vehicle, candidates, choice
  set_route_calls     every route write: time, vehicle, length, hash of the edges
  distance_answers    every incident-distance answer, in order, with the incidents open at the
                      time - whether SUMO was asked or the cache answered
  find_route          every findRoute not made by the incident-distance code: identical, in
                      order. Those made by it: either identical, or - when exactly one side
                      caches - the cached side asked SUMO exactly once per key, at that key's
                      first occurrence on the other side, and never otherwise
  counters            reroute decisions and executions, emergency reroutes, vehicles rerouted,
                      candidates scored, yield requests, yield actions, route-validity counters,
                      response times and the full response summary, teleports, incident errors
  tripinfo            tripinfo.xml identical record for record, every attribute (completed
                      trips, teleports, travel time, time loss, waiting ...)
  summary             summary.xml identical step for step, apart from SUMO's wall-clock field
  sumo_warnings       errors.log (SUMO's warnings) identical line for line
  no_tracebacks       no run log in either directory contains a Python traceback or a
                      P3Runtime exception report

With `--seed S` it also checks pairing: every ordinary vehicle's recorded speed factor on both
sides equals the value frozen for seed S (contract 13.13, 13.15). Target 0 differences.

    python scripts/diagnostics/ab_compare.py DIR_A DIR_B [--seed 20260723]

Exit code 0 only if every check passes. Timing is printed, never compared.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))

# SUMO's summary.xml records each step's real computation time; it can never be identical.
SUMMARY_WALL_CLOCK_FIELDS = {"duration"}
TRACEBACK_MARKERS = ("Traceback (most recent call last)", "[P3Runtime]")


def load(directory: Path) -> dict:
    return json.loads((directory / "result.json").read_text(encoding="utf-8"))


def first_difference(a: list, b: list):
    for index, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return index, x, y
    if len(a) != len(b):
        return min(len(a), len(b)), f"length {len(a)}", f"length {len(b)}"
    return None


def xml_records(path: Path, tag: str, ignore=frozenset()) -> list:
    return [
        {k: v for k, v in element.attrib.items() if k not in ignore}
        for element in ET.parse(path).getroot().iter(tag)
    ]


def distance_find_route_expected(uncached_answers: list) -> list:
    """The distance findRoute calls a correct cache makes: the first occurrence of each key."""
    seen = set()
    expected = []
    for t, from_edge, to_edge, vehicle_type, active, _answer in uncached_answers:
        key = (from_edge, to_edge, vehicle_type, tuple(active))
        if key not in seen:
            seen.add(key)
            expected.append([t, [from_edge, to_edge, vehicle_type]])
    return expected


def compare_find_route(a: dict, b: dict) -> tuple:
    calls_a, calls_b = a["find_route_calls"], b["find_route_calls"]
    other_a = [c[:2] for c in calls_a if c[2] == "other"]
    other_b = [c[:2] for c in calls_b if c[2] == "other"]
    difference = first_difference(other_a, other_b)
    if difference:
        return False, f"non-distance findRoute differs at #{difference[0]}: {difference[1]} vs {difference[2]}"

    dist_a = [c[:2] for c in calls_a if c[2] == "distance"]
    dist_b = [c[:2] for c in calls_b if c[2] == "distance"]
    if dist_a == dist_b:
        return True, f"identical: {len(other_a)} other, {len(dist_a)} incident-distance"

    # Exactly one side caches: it must ask SUMO once per key, at the key's first occurrence.
    if len(dist_a) > len(dist_b):
        uncached, cached_calls, label = a, dist_b, "B"
    else:
        uncached, cached_calls, label = b, dist_a, "A"
    expected = distance_find_route_expected(uncached["route_distance_calls"])
    difference = first_difference(expected, cached_calls)
    if difference:
        return False, (f"side {label}'s incident-distance findRoute calls are not exactly one per "
                       f"key at first occurrence: #{difference[0]}: {difference[1]} vs {difference[2]}")
    total = len(uncached["route_distance_calls"])
    return True, (f"{len(other_a)} other calls identical; incident-distance: {max(len(dist_a), len(dist_b))} "
                  f"uncached vs {len(cached_calls)} cached = one per key ({len(expected)} keys), "
                  f"{total - len(cached_calls)} of {total} answers from memory "
                  f"({100.0 * (total - len(cached_calls)) / total:.1f}%)" if total else "no queries")


def run_logs(directory: Path) -> list:
    return [p for p in directory.iterdir() if p.suffix in (".out", ".err", ".log") and p.name != "errors.log"
            and p.name != "messages.log"]


def pairing(result: dict, seed: int) -> tuple:
    from evaluation_seeds import frozen_demand_paths

    _, manifest_path = frozen_demand_paths(seed)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if int(manifest["sumo_seed"]) != seed:
        return None, f"manifest {manifest_path} is for seed {manifest['sumo_seed']}"
    frozen = {k: float(v) for k, v in manifest["speed_factors"].items()}
    recorded = {v: sf for v, (_, sf) in result["speed_factors"].items() if v in frozen}
    differing = [v for v, sf in recorded.items() if sf != frozen[v]]
    return differing, f"{len(recorded)} ordinary vehicles recorded, {len(differing)} differ from seed {seed}'s frozen file"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("a", type=Path)
    parser.add_argument("b", type=Path)
    parser.add_argument("--seed", type=int, default=None, help="also check pairing against this seed's frozen file")
    args = parser.parse_args()

    a, b = load(args.a), load(args.b)
    checks = []

    def check(name, passed, detail):
        checks.append(passed)
        print(f"  {'PASS' if passed else 'FAIL'}  {name:18s} {detail}")

    print(f"A: {args.a}  (arm {a['arm']}, seed {a.get('evaluation_seed')}, cache {a.get('distance_cache')})")
    print(f"B: {args.b}  (arm {b['arm']}, seed {b.get('evaluation_seed')}, cache {b.get('distance_cache')})")
    check("same arm and seed", a["arm"] == b["arm"] and a.get("evaluation_seed") == b.get("evaluation_seed"),
          f"{a['arm']}/{a.get('evaluation_seed')} vs {b['arm']}/{b.get('evaluation_seed')}")

    for field, label in (("trace", "trace"), ("decisions", "decisions"), ("set_route_calls", "set_route_calls"),
                         ("route_distance_calls", "distance_answers")):
        difference = first_difference(a[field], b[field])
        check(label, difference is None,
              f"{len(a[field])} records identical" if difference is None
              else f"first difference at #{difference[0]}: {difference[1]} vs {difference[2]}")

    check("speed_factors", a["speed_factors"] == b["speed_factors"],
          f"{len(a['speed_factors'])} vehicles identical" if a["speed_factors"] == b["speed_factors"]
          else f"{sum(1 for v in a['speed_factors'] if b['speed_factors'].get(v) != a['speed_factors'][v])} differ")

    passed, detail = compare_find_route(a, b)
    check("find_route", passed, detail)

    counters_equal = a["counters"] == b["counters"]
    shown = {k: v for k, v in (a["counters"] or {}).items() if k not in ("response_summary", "route_stats", "teleports")}
    check("counters", counters_equal, json.dumps(shown) if counters_equal else
          f"A {json.dumps(a['counters'])[:300]} vs B {json.dumps(b['counters'])[:300]}")

    trips_a = xml_records(args.a / "tripinfo.xml", "tripinfo")
    trips_b = xml_records(args.b / "tripinfo.xml", "tripinfo")
    difference = first_difference(trips_a, trips_b)
    check("tripinfo", difference is None,
          f"{len(trips_a)} records identical" if difference is None
          else f"record #{difference[0]}: {difference[1]} vs {difference[2]}")

    steps_a = xml_records(args.a / "summary.xml", "step", SUMMARY_WALL_CLOCK_FIELDS)
    steps_b = xml_records(args.b / "summary.xml", "step", SUMMARY_WALL_CLOCK_FIELDS)
    difference = first_difference(steps_a, steps_b)
    check("summary", difference is None,
          f"{len(steps_a)} steps identical" if difference is None
          else f"step #{difference[0]}: {difference[1]} vs {difference[2]}")

    warnings_a = (args.a / "errors.log").read_text(encoding="utf-8", errors="replace").splitlines()
    warnings_b = (args.b / "errors.log").read_text(encoding="utf-8", errors="replace").splitlines()
    check("sumo_warnings", warnings_a == warnings_b, f"{len(warnings_a)} lines identical" if warnings_a == warnings_b
          else f"{len(warnings_a)} vs {len(warnings_b)} lines")

    dirty = [str(p) for d in (args.a, args.b) for p in run_logs(d)
             if any(m in p.read_text(encoding="utf-8", errors="replace") for m in TRACEBACK_MARKERS)]
    logs = [p.name for d in (args.a, args.b) for p in run_logs(d)]
    check("no_tracebacks", not dirty and bool(logs),
          f"{len(logs)} run logs scanned, none with a traceback" if not dirty and logs
          else (f"tracebacks in {dirty}" if dirty else "no run logs found next to result.json"))

    if args.seed is not None:
        for side, result in (("A", a), ("B", b)):
            differing, detail = pairing(result, args.seed)
            check(f"pairing {side}", differing == [], detail)

    for side, result in (("A", a), ("B", b)):
        timing = result.get("timing") or {}
        print(f"  timing {side}: {timing}")
    ok = all(checks)
    print("RESULT:", "PASS - identical in every recorded field" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
