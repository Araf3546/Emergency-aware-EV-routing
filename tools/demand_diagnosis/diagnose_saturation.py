"""Controlled experiments to find the real cause of the insertion failure.

Variants, all on the identical network:
  A baseline        8003 veh / 3600s                     (current setup)
  B decompressed    8003 veh / 7200s  (depart x2)        tests: is it the 2x time compression?
  C half_volume     4002 veh / 3600s  (every 2nd veh)    tests: is it raw vehicle count?
  D deconcentrated  8003 veh / 3600s, per-edge cap       tests: is it origin concentration?

C keeps the same origin concentration; D keeps the same volume and duration.
Whichever variant recovers insertion identifies the binding constraint.
"""

import re
import subprocess
import sys
from collections import Counter
from pathlib import Path
from xml.etree import ElementTree as ET

WORK = Path("results/_diagnosis")
WORK.mkdir(parents=True, exist_ok=True)
NET = "network/porto.net.xml"
SOURCES = ["demand/porto.rou.xml", "demand/porto.emergency.rou.xml", "demand/porto.dispatch.rou.xml"]


def load_all():
    vtypes, vehicles = [], []
    for path in SOURCES:
        root = ET.parse(path).getroot()
        for vtype in root.findall("vType"):
            vtypes.append(vtype)
        for vehicle in root.findall("vehicle"):
            vehicles.append(vehicle)
    vehicles.sort(key=lambda v: float(v.attrib["depart"]))
    return vtypes, vehicles


def write_routes(path, vtypes, vehicles):
    root = ET.Element("routes")
    for vtype in vtypes:
        root.append(vtype)
    for vehicle in sorted(vehicles, key=lambda v: float(v.attrib["depart"])):
        root.append(vehicle)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def run(name, route_file, end):
    summary = WORK / f"{name}_summary.xml"
    msg = WORK / f"{name}_msg.log"
    cmd = [
        "sumo", "-n", NET, "-r", str(route_file), "--end", str(end),
        "--summary-output", str(summary), "--message-log", str(msg),
        "--tripinfo-output", str(WORK / f"{name}_tripinfo.xml"),
        "--no-step-log", "true", "--ignore-route-errors", "true", "--no-warnings", "true",
    ]
    subprocess.run(cmd, capture_output=True, text=True, check=False)

    text = summary.read_text(encoding="utf-8")
    rows = re.findall(
        r'time="([0-9.]+)" loaded="(\d+)" inserted="(\d+)" running="(\d+)" waiting="(\d+)" ended="(\d+)"', text)
    _, loaded, inserted, running, waiting, ended = rows[-1]

    log = msg.read_text(encoding="utf-8", errors="replace")
    tele = re.search(r"Teleports:\s*(\d+)", log)
    teleports = int(tele.group(1)) if tele else 0

    total = int(loaded)
    return {
        "name": name, "end": end, "loaded": total,
        "inserted": int(inserted), "never_in": total - int(inserted),
        "pct_never_in": 100.0 * (total - int(inserted)) / total if total else 0,
        "completed": int(ended), "pct_completed": 100.0 * int(ended) / total if total else 0,
        "pending_at_end": int(waiting), "teleports": teleports,
    }


vtypes, vehicles = load_all()
print(f"loaded {len(vehicles)} vehicles, {len(vtypes)} vTypes")

# A baseline
a_file = WORK / "A_baseline.rou.xml"
write_routes(a_file, vtypes, vehicles)

# B decompressed: undo the 2x compression by doubling every departure time
import copy
b_veh = []
for v in vehicles:
    c = copy.deepcopy(v)
    c.set("depart", f"{float(v.attrib['depart']) * 2:.2f}")
    b_veh.append(c)
b_file = WORK / "B_decompressed.rou.xml"
write_routes(b_file, vtypes, b_veh)

# C half volume, same concentration and duration
c_file = WORK / "C_half_volume.rou.xml"
write_routes(c_file, vtypes, vehicles[::2])

# D deconcentrated: cap departures per origin edge, drop the overflow
CAP = 40
seen = Counter()
d_veh = []
for v in vehicles:
    origin = v.find("route").attrib["edges"].split()[0]
    if seen[origin] < CAP:
        seen[origin] += 1
        d_veh.append(v)
d_file = WORK / "D_deconcentrated.rou.xml"
write_routes(d_file, vtypes, d_veh)
print(f"variant D keeps {len(d_veh)} of {len(vehicles)} vehicles (per-origin cap {CAP})")

results = [
    run("A_baseline", a_file, 3600),
    run("B_decompressed", b_file, 7200),
    run("C_half_volume", c_file, 3600),
    run("D_deconcentrated", d_file, 3600),
]

print()
hdr = f"{'variant':<18}{'end':>6}{'loaded':>8}{'inserted':>10}{'never in':>10}{'%never':>8}{'done':>7}{'%done':>7}{'telep':>7}"
print(hdr)
print("-" * len(hdr))
for r in results:
    print(f"{r['name']:<18}{r['end']:>6}{r['loaded']:>8}{r['inserted']:>10}{r['never_in']:>10}"
          f"{r['pct_never_in']:>7.1f}%{r['completed']:>7}{r['pct_completed']:>6.1f}%{r['teleports']:>7}")
