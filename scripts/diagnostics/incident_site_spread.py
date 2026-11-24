# READ-ONLY diagnostic. Modifies no production file. Run from Mine/ with `python <path>`.
"""READ-ONLY: how many mutually separated, well-used incident sites exist?"""
import xml.etree.ElementTree as ET, collections, math, os
os.chdir(r"d:\p3\p3-20260818T160450Z-1-001\p3\Mine")
net=ET.parse("network/porto.net.xml").getroot()
centroid={}
for e in net.findall("edge"):
    if e.get("function")=="internal": continue
    ls=e.findall("lane")
    if not ls: continue
    p=ls[0].get("shape","").split()
    if p:
        xs=[float(q.split(",")[0]) for q in p]; ys=[float(q.split(",")[1]) for q in p]
        centroid[e.get("id")]=(sum(xs)/len(xs),sum(ys)/len(ys))
usage=collections.Counter()
for v in ET.parse("demand/porto.rou.xml").getroot().iter("vehicle"):
    r=v.find("route")
    if r is not None:
        for e in set(r.get("edges").split()): usage[e]+=1
dests={c.find("route").get("edges").split()[-1]
       for c in ET.parse("network/validation/routeability_sample.rou.xml").getroot().iter("vehicle")}
def d(a,b): return math.hypot(centroid[a][0]-centroid[b][0],centroid[a][1]-centroid[b][1])
for minuse in (150,80,40):
    pool=[e for e in dests if usage.get(e,0)>=minuse and e in centroid]
    xs=[centroid[e][0] for e in pool]; ys=[centroid[e][1] for e in pool]
    print("\nusage>=%d: %d candidate destination edges, spread %.0fm x %.0fm"
          %(minuse,len(pool),max(xs)-min(xs),max(ys)-min(ys)))
    for sep in (800,700,600,500,400):
        # greedy farthest-point: seed at 401's edge, then repeatedly add the farthest valid
        chosen=["734630276"] if "734630276" in centroid else [pool[0]]
        for _ in range(9):
            best,bd=None,-1
            for e in pool:
                if e in chosen: continue
                m=min(d(e,c) for c in chosen)
                if m>=sep and m>bd: best,bd=e,m
            if best is None: break
            chosen.append(best)
        print("   sep>=%3dm -> max %d sites: %s"%(sep,len(chosen),
              ", ".join("%s(u=%d)"%(c,usage.get(c,0)) for c in chosen)))
