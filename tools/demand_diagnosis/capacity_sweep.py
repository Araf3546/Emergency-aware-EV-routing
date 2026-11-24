"""Find the network's carrying capacity.

Demand is always released across the same 3600 s window, only the vehicle count changes.
The simulation is allowed to run to 9000 s so the network can drain - otherwise vehicles
that departed near t=3600 are counted as failures purely because the clock stopped.

Reported per run: how much demand got in, how much finished, teleports, and mean
timeLoss (the contract's congestion definition).
"""

import copy
import re
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

WORK = Path("results/_diagnosis")
NET = "network/porto.net.xml"
SOURCES = ["demand/porto.rou.xml", "demand/porto.emergency.rou.xml", "demand/porto.dispatch.rou.xml"]
DRAIN_END = 9000


def load_all():
    vtypes, vehicles = [], []
    for path in SOURCES:
        root = ET.parse(path).getroot()
        vtypes.extend(root.findall("vType"))
        vehicles.extend(root.findall("vehicle"))
    vehicles.sort(key=lambda v: float(v.attrib["depart"]))
    return vtypes, vehicles


def write(path, vtypes, vehicles):
    root = ET.Element("routes")
    for vtype in vtypes:
        root.append(copy.deepcopy(vtype))
    for vehicle in vehicles:
        root.append(copy.deepcopy(vehicle))
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def run(name, route_file):
    summary = WORK / f"{name}_sum.xml"
    msg = WORK / f"{name}_msg.log"
    trip = WORK / f"{name}_trip.xml"
    subprocess.run([
        "sumo", "-n", NET, "-r", str(route_file), "--end", str(DRAIN_END),
        "--summary-output", str(summary), "--message-log", str(msg),
        "--tripinfo-output", str(trip), "--no-step-log", "true",
        "--ignore-route-errors", "true", "--no-warnings", "true",
    ], capture_output=True, text=True, check=False)

    text = summary.read_text(encoding="utf-8")
    rows = re.findall(r'loaded="(\d+)" inserted="(\d+)" running="(\d+)" waiting="(\d+)" ended="(\d+)"', text)
    loaded, inserted, _running, _waiting, ended = (int(x) for x in rows[-1])

    log = msg.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"Teleports:\s*(\d+)", log)
    teleports = int(match.group(1)) if match else 0

    losses, durations = [], []
    for element in ET.parse(trip).getroot().findall("tripinfo"):
        losses.append(float(element.attrib["timeLoss"]))
        durations.append(float(element.attrib["duration"]))
    mean_loss = sum(losses) / len(losses) if losses else 0.0
    mean_dur = sum(durations) / len(durations) if durations else 0.0

    return {
        "name": name, "loaded": loaded, "inserted": inserted, "ended": ended,
        "pct_in": 100.0 * inserted / loaded, "pct_done": 100.0 * ended / loaded,
        "teleports": teleports, "mean_timeloss": mean_loss, "mean_duration": mean_dur,
    }


vtypes, vehicles = load_all()
total = len(vehicles)
results = []
for target in (2000, 3000, 4000, 5000, 6000, 8003):
    step = total / target
    subset = [vehicles[int(i * step)] for i in range(target) if int(i * step) < total]
    path = WORK / f"cap_{target}.rou.xml"
    write(path, vtypes, subset)
    results.append(run(f"cap_{target}", path))
    r = results[-1]
    print(f"  {target:>5} vehicles -> in {r['pct_in']:5.1f}%  done {r['pct_done']:5.1f}%  "
          f"teleports {r['teleports']:>5}  mean timeLoss {r['mean_timeloss']:7.1f}s")

print()
hdr = (f"{'demand':>8}{'rate/s':>9}{'inserted':>10}{'completed':>11}"
       f"{'teleports':>11}{'meanTimeLoss':>14}{'meanTripTime':>14}")
print(hdr)
print("-" * len(hdr))
for r in results:
    print(f"{r['loaded']:>8}{r['loaded']/3600:>9.2f}{r['pct_in']:>9.1f}%{r['pct_done']:>10.1f}%"
          f"{r['teleports']:>11}{r['mean_timeloss']:>13.1f}s{r['mean_duration']:>13.1f}s")
