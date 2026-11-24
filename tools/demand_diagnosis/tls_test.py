import re, subprocess
from pathlib import Path
from xml.etree import ElementTree as ET
W = Path("results/_diagnosis")
NET_A = "network/porto.net.xml"
NET_B = str(W / "porto_actuated.net.xml")

def run(tag, net, routes, end=9000):
    s, m, t = W/f"{tag}_sum.xml", W/f"{tag}_msg.log", W/f"{tag}_trip.xml"
    subprocess.run(["sumo","-n",net,"-r",str(routes),"--end",str(end),
        "--summary-output",str(s),"--message-log",str(m),"--tripinfo-output",str(t),
        "--no-step-log","true","--ignore-route-errors","true","--no-warnings","true"],
        capture_output=True, text=True, check=False)
    rows = re.findall(r'loaded="(\d+)" inserted="(\d+)" running="(\d+)" waiting="(\d+)" ended="(\d+)"',
                      s.read_text(encoding="utf-8"))
    loaded, ins, _, _, ended = (int(x) for x in rows[-1])
    mm = re.search(r"Teleports:\s*(\d+)", m.read_text(encoding="utf-8", errors="replace"))
    tele = int(mm.group(1)) if mm else 0
    loss = [float(e.attrib["timeLoss"]) for e in ET.parse(t).getroot().findall("tripinfo")]
    dur = [float(e.attrib["duration"]) for e in ET.parse(t).getroot().findall("tripinfo")]
    return dict(tag=tag, loaded=loaded, pct_in=100*ins/loaded, pct_done=100*ended/loaded,
                tele=tele, loss=sum(loss)/len(loss) if loss else 0,
                dur=sum(dur)/len(dur) if dur else 0)

out=[]
for n in (5000, 6000, 8003):
    out.append(run(f"act_{n}", NET_B, W/f"cap_{n}.rou.xml"))
    r=out[-1]
    print(f"  actuated {n:>5}: in {r['pct_in']:5.1f}%  done {r['pct_done']:5.1f}%  telep {r['tele']:>5}  timeLoss {r['loss']:7.1f}s  trip {r['dur']:7.1f}s")
