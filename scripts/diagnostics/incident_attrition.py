# READ-ONLY diagnostic. Modifies no production file. Run from Mine/ with `python <path>`.
"""READ-ONLY: attrition diagnosis for the later slots."""
import xml.etree.ElementTree as ET, collections, math, os
os.chdir(r"d:\p3\p3-20260818T160450Z-1-001\p3\Mine")
net=ET.parse("network/porto.net.xml").getroot()
length,speed,centroid,lanes_n={},{},{},{}
for e in net.findall("edge"):
    if e.get("function")=="internal": continue
    ls=e.findall("lane")
    if not ls: continue
    i=e.get("id"); length[i]=max(float(l.get("length",0)) for l in ls)
    speed[i]=max(float(l.get("speed",13.9)) for l in ls); lanes_n[i]=len(ls)
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
def prof(ed):
    ed=[e for e in ed if e in length]
    return (sum(length[e] for e in ed), sum(1 for e in ed[:-1] if len(succ.get(e,()))>1),
            sum(length[e]/max(1.0,speed[e]) for e in ed))
def dist(a,b):
    if a not in centroid or b not in centroid: return 1e9
    return math.hypot(centroid[a][0]-centroid[b][0],centroid[a][1]-centroid[b][1])
def mult(t): return 12.7+0.00339*(t-960)
cands=[]
for v in ET.parse("network/validation/routeability_sample.rou.xml").getroot().iter("vehicle"):
    ed=v.find("route").get("edges").split(); m,dp,ff=prof(ed)
    cands.append(dict(id=v.get("id"),edges=ed,m=m,dp=dp,ff=ff,d=ed[-1],o=ed[0]))

def attrition(t,lo,hi,mdp,minuse,sep,exclude):
    ffb=(3400-t)/mult(t); s=collections.OrderedDict()
    cur=[c for c in cands if lo<=c["m"]<=hi];            s["length band"]=len(cur)
    cur=[c for c in cur if c["dp"]>=mdp];                s["dp>=%d"%mdp]=len(cur)
    cur=[c for c in cur if c["ff"]<=ffb];                s["ff<=%.0fs"%ffb]=len(cur)
    cur=[c for c in cur if usage.get(c["d"],0)>=minuse]; s["use>=%d"%minuse]=len(cur)
    cur=[c for c in cur if all(dist(c["d"],x)>=sep for x in exclude)]; s["sep>=%dm"%sep]=len(cur)
    return s,sorted(cur,key=lambda c:(-c["dp"],c["ff"]))

EX=["734630276","835319585#7"]
print("MID slot, dispatch 1560 - relaxing one constraint at a time")
for lo,hi,mdp,mu,sep in ((1200,1900,22,80,800),(1000,1900,22,80,800),(1000,1900,18,80,800),
                         (1000,1900,18,80,600),(1000,1900,18,40,600),(800,1900,15,40,600)):
    s,ok=attrition(1560,lo,hi,mdp,mu,sep,EX)
    print("  %4d-%4dm dp>=%2d use>=%3d sep>=%3d :: %s"%(lo,hi,mdp,mu,sep," -> ".join("%s:%d"%kv for kv in s.items())))
    if ok: print("        best: %s %.0fm dp=%d est_resp=%.0fs arrive=%.0f dest=%s use=%d"
                 %(ok[0]["id"],ok[0]["m"],ok[0]["dp"],ok[0]["ff"]*mult(1560),
                   1560+ok[0]["ff"]*mult(1560),ok[0]["d"],usage.get(ok[0]["d"],0)))
print("\nLATE slot, dispatch 2160 - relaxing one constraint at a time")
for lo,hi,mdp,mu,sep in ((700,1200,14,80,800),(600,1200,14,80,800),(600,1200,10,80,600),
                         (600,1200,10,40,600),(500,1200,8,40,500)):
    s,ok=attrition(2160,lo,hi,mdp,mu,sep,EX)
    print("  %4d-%4dm dp>=%2d use>=%3d sep>=%3d :: %s"%(lo,hi,mdp,mu,sep," -> ".join("%s:%d"%kv for kv in s.items())))
    if ok: print("        best: %s %.0fm dp=%d est_resp=%.0fs arrive=%.0f dest=%s use=%d"
                 %(ok[0]["id"],ok[0]["m"],ok[0]["dp"],ok[0]["ff"]*mult(2160),
                   2160+ok[0]["ff"]*mult(2160),ok[0]["d"],usage.get(ok[0]["d"],0)))
