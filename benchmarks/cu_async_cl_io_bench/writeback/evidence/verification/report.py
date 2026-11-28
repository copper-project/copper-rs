"""Summarize release jitter and distinguish late starts, slow ticks and lost output."""
import collections
import csv
import gzip
import math
import pathlib
import re

here = pathlib.Path(__file__).resolve().parent

def quantile(seq, q):
    ordered = sorted(seq)
    return ordered[math.ceil(q * len(ordered))-1]

def emit(filename, rows):
    if not rows:
        return
    with (here / filename).open("w") as out:
        writer = csv.DictWriter(out, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

summaries = []
groups = []
for case, period in [("pid",1_000_000),("image",50_000_000),("age-pid",1_000_000),("age-image",50_000_000)]:
    by_group = collections.defaultdict(list)
    folder = here/(case+"-results")
    for path in sorted(folder.glob("*.txt")) or sorted(folder.glob("*.txt.gz")):
        text = gzip.decompress(path.read_bytes()).decode() if path.suffix==".gz" else path.read_text()
        if "PARTITION " not in text: continue
        rev, mode, rep = path.name.split(".")[0].split("-")
        counts = {k:int(v) for k,v in re.findall(r"(\w+)=(\d+)", text.split("RESULT ")[1])}
        ticks = [tuple(map(int,match)) for match in re.findall(r"^TICK (\d+) (\d+) (\d+)$",text,re.M)]
        assert [t[0] for t in ticks] == list(range(counts["frames"]))
        jitter = [abs(b[1]-a[1]-period) for a,b in zip(ticks,ticks[1:])]
        gap = max(b[1]-a[1] for a,b in zip(ticks,ticks[1:]))
        lateness = [max(0,start-index*period) for index,start,duration in ticks]
        duration = [t[2] for t in ticks]
        over = sum(t > period for t in duration)
        missed = sum(l+t>period for l,t in zip(lateness,duration))
        meta = dict(case=case,revision=rev,mode=mode,rep=int(rep))
        row = dict(meta,**counts,duration_over_period=over,deadline_misses=missed,max_gap_us=gap/1000)
        for key,seq in [("jitter",jitter),("release_lateness",lateness),("tick",duration)]:
            for name,q in [("p50",.5),("p95",.95),("p99",.99),("p999",.999),("max",1)]:
                row[f"{key}_{name}_us"]=quantile(seq,q)/1000
        summaries.append(row)
        by_group[rev,mode].append((row,jitter,lateness,duration))
    for (rev,mode), trials in by_group.items():
        rows = [t[0] for t in trials]
        group = dict(case=case,revision=rev,mode=mode,runs=len(rows))
        for key in ["frames","recovered","drops","keyframes","duration_over_period","deadline_misses","cl_bytes","sectors_written","writes"]:
            group[key]=sum(r[key] for r in rows)
        for i,key in enumerate(["jitter","release_lateness","tick"],1):
            seq=[v for trial in trials for v in trial[i]]
            for name,q in [("p50",.5),("p95",.95),("p99",.99),("p999",.999),("max",1)]:
                group[f"{key}_{name}_us"]=quantile(seq,q)/1000
        group["max_gap_us"]=max(r["max_gap_us"] for r in rows)
        group["max_drain_us"]=max(r["drain_us"] for r in rows)
        groups.append(group)
emit("example-runs.csv",summaries)
emit("example-groups.csv",groups)
