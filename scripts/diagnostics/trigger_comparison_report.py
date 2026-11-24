#!/usr/bin/env python3
"""Side-by-side table of two trigger_comparison.py runs (condition A vs condition B).

    python scripts/diagnostics/trigger_comparison_report.py DIR_CURRENT DIR_JUNCTION

Reads DIR/summary.json from each. Refuses to compare runs of different seed, duration or
emergency-layer setting, since those would not be paired.
"""
import json
import sys
from pathlib import Path

ROWS = [
    ("network", "completed_trips", "completed trips"),
    ("network", "completion_rate", "completion rate"),
    ("network", "incomplete_trips", "incomplete (inserted, not finished)"),
    ("network", "never_inserted_vehicles", "never inserted"),
    ("network", "teleports_total", "teleports"),
    ("network", "mean_travel_time_s", "mean travel time s (completed trips)"),
    ("network", "mean_time_loss_s", "mean time loss s"),
    ("network", "mean_waiting_time_s", "mean waiting time s"),
    ("network", "distance_total_km", "total distance driven km"),
    ("routing", "reroute_requests", "reroute requests (trigger firings)"),
    ("routing", "e3_decisions", "E3 routing decisions"),
    ("routing", "pct_one_candidate", "% decisions with one candidate"),
    ("routing", "pct_real_choice", "% decisions with a real choice"),
    ("routing", "reroutes_applied", "reroutes applied"),
    ("routing", "applied_without_real_choice", "applied where there was no real choice"),
    ("routing", "reroutes_shorten_pct", "% reroutes shortening the remaining trip"),
    ("routing", "reroutes_lengthen_pct", "% reroutes lengthening it"),
    ("routing", "detour_mean_m", "mean detour m"),
    ("routing", "detour_median_m", "median detour m"),
    ("routing", "total_added_distance_m", "total added distance m"),
    ("emergency", "yield_actions", "yield actions"),
    ("emergency", "yield_requests", "yield requests"),
    ("emergency", "emergency_reroutes", "emergency reroutes"),
    ("cost", "wall_seconds", "wall clock s"),
    ("cost", "trigger_seconds", "time in trigger s"),
    ("cost", "e3_decide_seconds", "time in E3 decisions s"),
    ("pairing", "speed_factor_differences", "speed-factor differences vs frozen file"),
]


def responder_queues(run_dir: Path) -> dict:
    """Mean queue length on each responder's own route while it drives to its incident.

    From SUMO's queue.xml. "tail" is the five edges before the incident centre edge - the
    approach to the closure. Added because the junction trigger slowed responses 401 and 405
    without changing the responders' own routes; the queue they drove into is the difference.
    """
    import xml.etree.ElementTree as ET

    mine = Path(__file__).resolve().parent.parent.parent
    routes = {v.get("id").rsplit("_", 1)[1]: v.find("route").get("edges").split()
              for v in ET.parse(mine / "demand" / "porto.dispatch.rou.xml").getroot().iter("vehicle")
              if v.get("id").startswith("emergency_dispatch_")}
    responses = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))["emergency"]["responses"]
    windows = {ev: (responses[ev]["start_time"] + 60, responses[ev]["first_arrival_time"]) for ev in routes}
    totals = {ev: [0.0, 0.0, 0] for ev in routes}
    for _, element in ET.iterparse(run_dir / "queue.xml"):
        if element.tag != "data":
            continue
        t = float(element.get("timestep"))
        for ev, (start, end) in windows.items():
            if end is None or not start <= t <= end:
                continue
            route, tail = set(routes[ev]), set(routes[ev][-6:-1])
            for lane in element.iter("lane"):
                edge = lane.get("id").rsplit("_", 1)[0]
                if edge in route:
                    q = float(lane.get("queueing_length", 0.0))
                    totals[ev][0] += q
                    if edge in tail:
                        totals[ev][1] += q
            totals[ev][2] += 1
        element.clear()
    return {ev: {"mean_queue_on_route_m": round(q / max(n, 1), 1),
                 "mean_queue_last5_edges_m": round(tail / max(n, 1), 1)}
            for ev, (q, tail, n) in totals.items()}


def main() -> int:
    a, b = (json.loads((Path(d) / "summary.json").read_text(encoding="utf-8")) for d in sys.argv[1:3])
    for key in ("seed", "duration_s", "layer"):
        if a[key] != b[key]:
            print(f"not paired: {key} {a[key]} vs {b[key]}")
            return 1
    print(f"seed {a['seed']}, {a['duration_s']} s, emergency layer {a['layer']}")
    print(f"| metric | {a['trigger']} | {b['trigger']} |\n|---|---:|---:|")
    for section, key, label in ROWS:
        print(f"| {label} | {a[section].get(key)} | {b[section].get(key)} |")
    for label, run in (("A", a), ("B", b)):
        responses = run["emergency"]["responses"]
        cells = []
        for event in ("404", "401", "405"):
            r = responses.get(event, {})
            cells.append(f"{event}: {r.get('response_time_s')} s"
                         f"{' (post-window)' if r.get('arrived_after_incident_end') else ''}"
                         f"{' INVALID ' + str(r.get('invalid_reason')) if r.get('measured') and not r.get('response_valid') else ''}")
        print(f"responses {label} ({run['trigger']}): " + "; ".join(cells) +
              f"; responder teleports {run['emergency']['teleports']['responder_teleport_count']}")
    for label, directory in (("A", sys.argv[1]), ("B", sys.argv[2])):
        if (Path(directory) / "queue.xml").exists():
            print(f"queues on responder routes {label}: {responder_queues(Path(directory))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
