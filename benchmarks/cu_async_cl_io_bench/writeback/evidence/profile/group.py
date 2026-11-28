"""Produce review tables from the completed 20 Hz trials."""
import collections
import csv
import json
import pathlib
import statistics
import sys

root=pathlib.Path(sys.argv[1])
metrics=json.loads((root/"metrics.json").read_text())
deadlines={r["run"]:r for r in csv.DictReader((root/"deadline.csv").open())}
groups=collections.defaultdict(list)
for name,m in metrics.items():
    if not name.endswith("-spatial"):
        rev,case,slots,mode,rep=name.split("-")
        groups[(case,slots,mode,rev)].append((m,deadlines[name]))
rows=[]
for (case,slots,mode,rev),group in sorted(groups.items()):
    ms=[x for x,d in group]
    row=dict(case=case,slots=slots,mode=mode,revision=rev,runs=len(ms))
    for key in ["offered","accepted","persisted_copperlists","dropped_copperlists","persisted_keyframes","dropped_keyframes"]:
        row[key]=sum(m["counts"][key] for m in ms)
    row["ticks_over_50ms"]=sum(m["over_50ms"] for m in ms)
    for key in ["estimated_deadline_misses","overruns_without_overlapping_flush","overruns_with_50ms_outside_flush"]:
        row[key]=sum(int(d[key]) for m,d in group)
    for key in ["peak_release_lateness_ms","max_interstart_gap_ms","max_outside_flush_ms"]:
        row[key]=round(max(float(d[key]) for m,d in group),3)
    for key in ["submission","record","flush","tick","final_drain"]:
        for q in ["p99","max"]:
            v=[m["times_ns"][key][q]/1e6 for m in ms if m["times_ns"][key]]
            row[key+"_"+q+"_worst_run_ms"]=round(max(v),4) if v else ""
    row["flushes"]=sum(m["msync"]["count"] for m in ms)
    row["requested_GB"]=round(sum(m["msync"]["total_bytes"] for m in ms)/1e9,4)
    row["bytes_requested_per_recovered_cl"]=round(row["requested_GB"]*1e9/row["persisted_copperlists"])
    row["completion_inversions"]=sum(m["msync"]["completion_inversions"] for m in ms)
    rows.append(row)
with (root/"grouped.csv").open("w") as f:
    w=csv.DictWriter(f,fieldnames=rows[0])
    w.writeheader()
    w.writerows(rows)
with (root/"longest-flushes.csv").open("w") as f:
    w=csv.writer(f)
    w.writerow(["trial","rank","trigger","thread","slab","offset","bytes","ms","overlapping_ticks_over_50ms"])
    for name,m in sorted(metrics.items()):
        if name.endswith("-spatial"):continue
        for rank,e in enumerate(m["longest_flushes"],1):
            w.writerow([name,rank,e["trigger"],e["thread"],e["slab"],e["offset"],e["bytes"],round(e["ns"]/1e6,4),e["overlapping_missed_ticks"]])
