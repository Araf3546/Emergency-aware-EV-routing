"""Decisive experiment: keep volume (8003) and duration (3600s) fixed, remove ONLY the
origin-edge concentration, and see whether insertion recovers.

The pipeline currently snaps each observed pickup to its single nearest edge. Where a
real taxi hotspot exists, that collapses hundreds of pickups onto one short edge.
Here the same pickup coordinate is instead assigned among the edges within a small
radius, choosing the least-loaded per lane. This keeps the observed spatial demand
pattern but stops it piling onto one 45 m single-lane stub.

Nothing else changes: same seed, same sampled OD rows, same departure times.
"""

import csv
import random
import subprocess
from collections import Counter
from pathlib import Path
from xml.etree import ElementTree as ET

import sumolib

WORK = Path("results/_diagnosis")
WORK.mkdir(parents=True, exist_ok=True)

WINDOW_START, WINDOW_END = 14 * 3600, 16 * 3600
SEED, N_NORMAL, DURATION = 20260721, 7520, 3600
RADIUS = 60.0  # metres; the pipeline's own snap tolerance was up to 100 m

net = sumolib.net.readNet("network/porto.net.xml")

rows = list(csv.DictReader(open("demand/od_pairs.csv", newline="", encoding="utf-8")))
candidates = [r for r in rows
              if WINDOW_START <= int(r["departure_second_of_day_local"]) < WINDOW_END]
selected = random.Random(SEED).choices(candidates, k=N_NORMAL)
print(f"sampled {len(selected)} trips from {len(candidates)} candidate rows (identical to scripts/07)")

lane_count = {}
def lanes_of(edge_id):
    if edge_id not in lane_count:
        try:
            lane_count[edge_id] = net.getEdge(edge_id).getLaneNumber()
        except Exception:
            lane_count[edge_id] = 1
    return max(lane_count[edge_id], 1)

neighbour_cache = {}
def nearby_edges(lon, lat, fallback):
    key = (round(lon, 6), round(lat, 6))
    if key in neighbour_cache:
        return neighbour_cache[key]
    x, y = net.convertLonLat2XY(lon, lat)
    found = []
    for edge, dist in net.getNeighboringEdges(x, y, RADIUS):
        if edge.allows("passenger") and not edge.getID().startswith(":"):
            found.append((dist, edge.getID()))
    found.sort()
    result = [eid for _, eid in found] or [fallback]
    neighbour_cache[key] = result
    return result

load = Counter()
trips = []
reassigned = 0
for index, pair in enumerate(selected):
    observed = int(pair["departure_second_of_day_local"])
    depart = (observed - WINDOW_START) / (WINDOW_END - WINDOW_START) * DURATION

    options = nearby_edges(float(pair["origin_lon"]), float(pair["origin_lat"]), pair["origin_edge"])
    # least-loaded per lane wins; ties break toward the nearest edge (list order)
    best = min(options, key=lambda e: (load[e] / lanes_of(e), options.index(e)))
    if best != pair["origin_edge"]:
        reassigned += 1
    load[best] += 1
    trips.append((depart, best, pair["destination_edge"]))

trips.sort(key=lambda t: t[0])
print(f"reassigned {reassigned} of {len(trips)} origins to a nearby edge ({100*reassigned/len(trips):.1f}%)")
print(f"origin edges used: {len(load)} (was 646 across the whole demand)")
print(f"max departures on one edge: {load.most_common(1)[0]}")

root = ET.Element("routes")
ET.SubElement(root, "vType", id="normal_passenger", vClass="passenger", accel="2.6",
              decel="4.5", sigma="0.5", length="5.0", minGap="2.5", maxSpeed="13.89")
for index, (depart, origin, destination) in enumerate(trips):
    ET.SubElement(root, "trip", id=f"normal_{index:05d}", depart=f"{depart:.3f}",
                  type="normal_passenger", **{"from": origin, "to": destination})
trips_file = WORK / "E_rebalanced.trips.xml"
ET.ElementTree(root).write(trips_file, encoding="utf-8", xml_declaration=True)

routes_file = WORK / "E_rebalanced.rou.xml"
subprocess.run([
    r"C:\Program Files (x86)\Eclipse\Sumo\bin\duarouter.exe",
    "--net-file", "network/porto.net.xml", "--route-files", str(trips_file),
    "--output-file", str(routes_file), "--seed", "20260722", "--remove-loops", "true",
    "--unsorted-input", "true", "--ignore-errors", "true",
    "--error-log", str(WORK / "E_duarouter.log"),
], capture_output=True, text=True, check=False)

routed = len(ET.parse(routes_file).getroot().findall("vehicle"))
print(f"duarouter routed {routed} of {len(trips)} rebalanced trips")
