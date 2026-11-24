# READ-ONLY diagnostic. Modifies no production file. Run from Mine/ with `python <path>`.
"""READ-ONLY: cross-contamination and validity checks on the proposed set."""
import xml.etree.ElementTree as ET, os
os.chdir(r"d:\p3\p3-20260818T160450Z-1-001\p3\Mine")
PICKS={"404":"routeability_03385","402p":"routeability_00386","403p":"routeability_02467"}
WIN={"404":(600,900),"401":(900,1230),"402p":(1500,1800),"403p":(2100,2400)}  # 300s windows
TRAVEL={"404":(660,1489),"401":(960,1497),"402p":(1560,2644),"403p":(2160,2871)}
CENTRE={"404":"835319585#7","401":"734630276","402p":"-1027278835#1","403p":"800460939#0"}

routes={}
for v in ET.parse("network/validation/routeability_sample.rou.xml").getroot().iter("vehicle"):
    if v.get("id") in PICKS.values():
        routes[v.get("id")]=v.find("route").get("edges").split()
for v in ET.parse("demand/porto.dispatch.rou.xml").getroot().iter("vehicle"):
    if v.get("id")=="emergency_dispatch_401":
        routes["401"]=v.find("route").get("edges").split()
byev={"401":routes["401"]}
for ev,rid in PICKS.items(): byev[ev]=routes[rid]

print("=== does a responder's route cross another incident's closed edge while it is closed? ===")
clash=False
for ev,edges in byev.items():
    t0,t1=TRAVEL[ev]
    for other,c in CENTRE.items():
        if other==ev: continue
        if c in edges:
            w0,w1=WIN[other]
            overlap = not (t1<w0 or t0>w1)
            print("  %-5s route contains %-15s (incident %s, window %d-%d) travel %d-%d -> %s"
                  %(ev,c,other,w0,w1,t0,t1,"CLASH" if overlap else "ok, not concurrent"))
            clash = clash or overlap
if not clash: print("  none - no responder route passes through another active closure")

print("\n=== do responder routes share edges (possible mutual interference)? ===")
evs=list(byev)
for i in range(len(evs)):
    for j in range(i+1,len(evs)):
        a,b=evs[i],evs[j]
        shared=set(byev[a])&set(byev[b])
        ta,tb=TRAVEL[a],TRAVEL[b]
        conc = not (ta[1]<tb[0] or ta[0]>tb[1])
        if shared:
            print("  %-5s / %-5s share %d edges, concurrent=%s %s"%(a,b,len(shared),conc,
                  "<-- check in pilot" if conc else ""))
        else:
            print("  %-5s / %-5s share no edges (concurrent=%s)"%(a,b,conc))

print("\n=== route detail for the scenario file ===")
for ev in ("404","402p","403p"):
    e=byev[ev]
    print("  %-5s %-20s origin=%-16s dest=%-16s edges=%d"%(ev,PICKS[ev],e[0],e[-1],len(e)))
