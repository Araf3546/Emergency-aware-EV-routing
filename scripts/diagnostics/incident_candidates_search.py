# READ-ONLY diagnostic. Modifies no production file. Run from Mine/ with `python <path>`.
"""READ-ONLY final selection: mutually separated incident set, calibrated on 401/402/403."""
import xml.etree.ElementTree as ET, collections, math, os
os.chdir(r"d:\p3\p3-20260818T160450Z-1-001\p3\Mine")
net = ET.parse("network/porto.net.xml").getroot()
length,speed,centroid,lanes_n={},{},{},{}
for e in net.findall("edge"):
    if e.get("function")=="internal": continue
    ls=e.findall("lane")
    if not ls: continue
    i=e.get("id")
    length[i]=max(float(l.get("length",0)) for l in ls)
    speed[i]=max(float(l.get("speed",13.9)) for l in ls)
    lanes_n[i]=len(ls)
    p=ls[0].get("shape","").split()
    if p:
        xs=[float(q.split(",")[0]) for q in p]; ys=[float(q.split(",")[1]) for q in p]
        centroid[i]=(sum(xs)/len(xs),sum(ys)/len(ys))
succ=collections.defaultdict(set)
for c in net.findall("connection"):
    f,t=c.get("from"),c.get("to")
    if f and t and not f.startswith(":") and not t.startswith(":"): succ[f].add(t)
usage=collections.Counter()
for v in ET.parse("demand/porto.rou.xml").getroot().iter("vehicle"):
    r=v.find("route")
    if r is not None:
        for e in set(r.get("edges").split()): usage[e]+=1
def prof(edges):
    edges=[e for e in edges if e in length]
    return (sum(length[e] for e in edges),
            sum(1 for e in edges[:-1] if len(succ.get(e,()))>1),
            sum(length[e]/max(1.0,speed[e]) for e in edges))
def dist(a,b):
    if a not in centroid or b not in centroid: return 1e9
    return math.hypot(centroid[a][0]-centroid[b][0],centroid[a][1]-centroid[b][1])
def mult(t): return 12.7+0.00339*(t-960)

cands=[]
for v in ET.parse("network/validation/routeability_sample.rou.xml").getroot().iter("vehicle"):
    ed=v.find("route").get("edges").split(); m,dp,ff=prof(ed)
    cands.append(dict(id=v.get("id"),edges=ed,m=m,dp=dp,ff=ff,o=ed[0],d=ed[-1]))

# Slots: (label, dispatch, min_m, max_m, min_dp). 401 occupies dispatch 960.
SLOTS=[("early",660,1200,1900,22),("mid",1560,1200,1900,22),("late",2160,700,1200,14)]
FIXED=["734630276"]                       # 401's centre edge, kept
chosen=[]; used_dest=list(FIXED)
print("Selecting; response time must be <=3400s, dest usage>=80, >=800m apart\n")
for label,t,lo,hi,mdp in SLOTS:
    ffb=(3400-t)/mult(t)
    ok=[c for c in cands if lo<=c["m"]<=hi and c["dp"]>=mdp and c["ff"]<=ffb
        and usage.get(c["d"],0)>=80
        and all(dist(c["d"],x)>=800 for x in used_dest)]
    # prefer most route choice, then a shorter estimated response
    ok.sort(key=lambda c:(-c["dp"], c["ff"]))
    print("=== %-6s dispatch %4d | %d-%dm, dp>=%d, freeflow<=%.0fs -> %d qualify"
          %(label,t,lo,hi,mdp,ffb,len(ok)))
    for c in ok[:5]:
        print("   %-20s %5.0fm dp=%2d ff=%5.1fs est_resp=%4.0fs arrive=%4.0f dest=%-14s use=%3d lanes=%d"
              %(c["id"],c["m"],c["dp"],c["ff"],c["ff"]*mult(t),t+c["ff"]*mult(t),
                c["d"],usage.get(c["d"],0),lanes_n.get(c["d"],0)))
    if ok:
        pick=ok[0]; chosen.append((label,t,pick)); used_dest.append(pick["d"])
        print("   -> PICK %s (dest %s)\n"%(pick["id"],pick["d"]))
    else:
        print("   -> NONE\n")

print("=== resulting incident set ===")
print(" 401   start  900 dispatch  960 dest 734630276    est_resp 537s (measured) arrive 1497")
for label,t,c in chosen:
    print(" %-5s start %4d dispatch %4d dest %-14s est_resp %4.0fs arrive %4.0f  route %.0fm dp=%d"
          %(label,t-60,t,c["d"],c["ff"]*mult(t),t+c["ff"]*mult(t),c["m"],c["dp"]))
print("\n=== pairwise separation between centre edges (m) ===")
alld=[("401","734630276")]+[(l,c["d"]) for l,t,c in chosen]
for i in range(len(alld)):
    for j in range(i+1,len(alld)):
        print("  %-6s <-> %-6s %7.0f m"%(alld[i][0],alld[j][0],dist(alld[i][1],alld[j][1])))
