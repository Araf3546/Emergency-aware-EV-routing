# READ-ONLY diagnostic. Modifies no production file. Run from Mine/ with `python <path>`.
"""READ-ONLY: is a 4th incident achievable at all, and at what cost in route choice?"""
import xml.etree.ElementTree as ET, collections, math, os
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "incident_final_set.py")).read().split("SEP,OSEP")[0])

SEP,OSEP,ARRIVE_BY,MINUSE=700,500,3300,80
# the three-incident set already verified
chosen=[dict(FIX,label="401")]
for lab,rid,t in (("404","routeability_07399",660),("403p","routeability_01973",2160)):
    c=[x for x in cands if x["id"]==rid][0]
    c=dict(c); c["t"]=t; c["arrive"]=t+c["ff"]*mult(t); c["label"]=lab
    chosen.append(c)

print("trying to add a 4th incident, progressively relaxing route choice\n")
for t in (1500, 1560, 1800):
    for lo,hi,mdp in ((1200,1700,25),(1000,1700,18),(800,1700,15),(600,1700,10),(400,1700,5)):
        ffb=(ARRIVE_BY-t)/mult(t)
        ok=[]
        for c in cands:
            if not (lo<=c["m"]<=hi and c["dp"]>=mdp and c["ff"]<=ffb): continue
            if usage.get(c["d"],0)<MINUSE: continue
            if any(dist(c["d"],s["d"])<SEP for s in chosen): continue
            if any(dist(c["o"],s["o"])<OSEP for s in chosen): continue
            arrive=t+c["ff"]*mult(t); bad=False
            for s in chosen:
                if not (arrive<s["t"] or t>s["arrive"]):
                    if c["eset"] & s["eset"]: bad=True; break
            if bad: continue
            ok.append((c,arrive))
        ok.sort(key=lambda x:-x[0]["dp"])
        tag = "%s dp>=%2d %4d-%4dm"%(("disp %d"%t),mdp,lo,hi)
        if ok:
            c,a=ok[0]
            print("  %-28s -> %2d qualify | best %s %.0fm dp=%d resp=%.0fs arrive=%.0f d=%s"
                  %(tag,len(ok),c["id"],c["m"],c["dp"],c["ff"]*mult(t),a,c["d"]))
        else:
            print("  %-28s -> none"%tag)
    print()
