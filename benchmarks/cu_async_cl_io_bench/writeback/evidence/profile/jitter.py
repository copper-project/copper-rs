"""Attribute long tick spans and compute release-relative deadlines from the raw timed trials."""
import csv
import gzip
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
with (root/"deadline.csv").open("w") as output:
    fields=["run","ticks","ticks_over_50ms","overruns_without_overlapping_flush","overruns_with_50ms_outside_flush","estimated_deadline_misses","peak_release_lateness_ms","max_interstart_gap_ms","max_outside_flush_ms"]
    writer = csv.DictWriter(output, fieldnames=fields)
    writer.writeheader()
    for ok in sorted(root.glob("*.ok")):
        if ok.stem.endswith("-spatial"): continue
        with gzip.open(root/(ok.stem+".csv.gz"),"rt") as raw:
            events=list(csv.DictReader(raw))
        def spans(kind):
            return sorted((int(e["begin_ns"]),int(e["end_ns"])) for e in events if e["kind"]==kind)
        ticks=spans("tick")
        flushes=spans("flush")
        over=no_flush=outside=miss=peak=max_outside=0
        first=0
        for i,(b,e) in enumerate(ticks):
            while first<len(flushes) and flushes[first][1]<=b: first+=1
            j=first
            covered=0
            last=b
            while j<len(flushes) and flushes[j][0]<e:
                start,end=flushes[j]
                covered+=max(0,min(end,e)-max(b,start,last))
                last=max(last,end)
                j+=1
            late=max(0,b-ticks[0][0]-i*50_000_000)
            peak=max(peak,late)
            miss+=e-ticks[0][0]>(i+1)*50_000_000
            if e-b>50_000_000:
                over+=1
                no_flush+=covered==0
                outside+=(e-b-covered)>50_000_000
                max_outside=max(max_outside,e-b-covered)
        gap=max(b[0]-a[0] for a,b in zip(ticks,ticks[1:]))
        writer.writerow(dict(zip(fields,[ok.stem,len(ticks),over,no_flush,outside,miss,peak/1e6,gap/1e6,max_outside/1e6])))
