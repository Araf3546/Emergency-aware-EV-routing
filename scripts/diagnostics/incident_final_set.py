# READ-ONLY diagnostic. Modifies no production file. Run from Mine/ with `python <path>`.
"""READ-ONLY final selection v2: adds origin separation + no shared edges when concurrent."""
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
    ed2=[e for e in ed if e in length]
    return (sum(length[e] for e in ed2), sum(1 for e in ed2[:-1] if len(succ.get(e,()))>1),
            sum(length[e]/max(1.0,speed[e]) for e in ed2))
def dist(a,b):
    if a not in centroid or b not in centroid: return 1e9
    return math.hypot(centroid[a][0]-centroid[b][0],centroid[a][1]-centroid[b][1])
def mult(t): return 12.7+0.00339*(t-960)
cands=[]
for v in ET.parse("network/validation/routeability_sample.rou.xml").getroot().iter("vehicle"):
    ed=v.find("route").get("edges").split(); m,dp,ff=prof(ed)
    cands.append(dict(id=v.get("id"),edges=ed,eset=set(ed),m=m,dp=dp,ff=ff,d=ed[-1],o=ed[0]))

r401=[v.find("route").get("edges").split() for v in
      ET.parse("demand/porto.dispatch.rou.xml").getroot().iter("vehicle")
      if v.get("id")=="emergency_dispatch_401"][0]
FIX={"id":"401","edges":r401,"eset":set(r401),"d":"734630276","o":r401[0],
     "t":960,"arrive":1497}

SEP,OSEP,ARRIVE_BY,MINUSE=700,500,3300,80
slots=[("404",660,1400,1900,25),("402p",1560,1200,1700,25),("403p",2160,700,1250,14)]
sel=[FIX]
for label,t,lo,hi,mdp in slots:
    ffb=(ARRIVE_BY-t)/mult(t)
    ok=[]
    for c in cands:
        if not (lo<=c["m"]<=hi and c["dp"]>=mdp and c["ff"]<=ffb): continue
        if usage.get(c["d"],0)<MINUSE: continue
        if any(dist(c["d"],s["d"])<SEP for s in sel): continue
        if any(dist(c["o"],s["o"])<OSEP for s in sel): continue
        arrive=t+c["ff"]*mult(t)
        bad=False
        for s in sel:
            conc = not (arrive < s["t"] or t > s["arrive"])
            if conc and (c["eset"] & s["eset"]): bad=True; break
        if bad: continue
        ok.append((c,arrive))
    ok.sort(key=lambda x:(-x[0]["dp"],x[0]["ff"]))
    print("=== %-5s dispatch %4d | %d-%dm dp>=%d ff<=%.0fs -> %d qualify"%(label,t,lo,hi,mdp,ffb,len(ok)))
    for c,a in ok[:4]:
        print("    %-20s %5.0fm dp=%2d resp=%4.0fs arrive=%4.0f o=%-15s d=%-15s use=%3d"
              %(c["id"],c["m"],c["dp"],c["ff"]*mult(t),a,c["o"],c["d"],usage.get(c["d"],0)))
    if ok:
        c,a=ok[0]; c=dict(c); c["t"]=t; c["arrive"]=a; c["label"]=label; sel.append(c)
        print("    -> PICK %s\n"%c["id"])
    else:
        print("    -> NONE\n")

print("=== FINAL VERIFIED SET ===")
rows=[]
for s in sel:
    lab=s.get("label","401")
    m,dp,ff=prof(s["edges"])
    rows.append((lab,s["t"]-60,s["t"],s["d"],s["arrive"]-s["t"],s["arrive"],m,dp,usage.get(s["d"],0),s["o"]))
rows.sort(key=lambda r:r[1])
print(" %-6s %6s %8s %-16s %8s %7s %7s %4s %5s"%("event","start","disp","centre","resp","arrive","route","dp","use"))
for r in rows: print(" %-6s %6d %8d %-16s %7.0fs %7.0f %6.0fm %4d %5d"%r[:9])
print("\n=== pairwise: separation / concurrency / shared edges ===")
for i in range(len(sel)):
    for j in range(i+1,len(sel)):
        a,b=sel[i],sel[j]
        conc = not (a["arrive"]<b["t"] or a["t"]>b["arrive"])
        print("  %-6s <-> %-6s centre %5.0fm origin %5.0fm concurrent=%-5s shared_edges=%d"
              %(a.get("label","401"),b.get("label","401"),dist(a["d"],b["d"]),
                dist(a["o"],b["o"]),conc,len(a["eset"]&b["eset"])))
