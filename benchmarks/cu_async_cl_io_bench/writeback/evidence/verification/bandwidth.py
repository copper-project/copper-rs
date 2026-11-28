"""Compare offered payloads and accepted CL bytes with direct sequential throughput."""
import csv
import gzip
import pathlib
import re
import statistics

here=pathlib.Path(__file__).resolve().parent
raw=list(csv.DictReader((here/"linear-results/summary.csv").open()))
capacity=statistics.median(float(r["MB_s"]) for r in raw)
rows=[]
for case,period in [("pid",.001),("image",.05)]:
    folder=here/(case+"-results")
    for p in sorted(folder.glob("*.txt")) or sorted(folder.glob("*.txt.gz")):
        text=gzip.decompress(p.read_bytes()).decode() if p.suffix==".gz" else p.read_text()
        if "PARTITION " not in text: continue
        trial=p.name.split(".")[0]
        values={k:int(v) for k,v in re.findall(r"(\w+)=(\d+)",text.split("RESULT ")[1])}
        frames=values["frames"]
        if case=="image":
            windows=re.findall(r"^Aligned window: left_len=(\d+) right_len=(\d+)",text,re.M)
            assert len(windows)==frames
            offered=(2*frames+sum(int(a)+int(b) for a,b in windows))*640*480
        else:
            offered=frames*2048
        sec=frames*period
        logical=offered/sec/1e6
        rows.append(dict(workload=case,trial=trial,offered_frames=frames,recovered=values["recovered"],
                         scheduled_seconds=sec,all_offered_sensor_payload_MB_s=logical,
                         accepted_encoded_CL_MB_s=values["cl_bytes"]/sec/1e6,
                         partition_written_MB_per_scheduled_s=values["sectors_written"]*512/sec/1e6,
                         offered_payload_pct_of_linear=logical/capacity*100))
for p in sorted((here.parent/"profile/sd-results").glob("*.ok")):
    if p.stem.endswith("spatial"):continue
    text=p.with_suffix(".txt").read_text()
    frames=int(re.search(r"iterations=(\d+)",text)[1])
    recovered=int(re.search(r"persisted_copperlists=(\d+)",text)[1])
    logical=750000*20/1e6
    rows.append(dict(workload="750k",trial=p.stem,offered_frames=frames,recovered=recovered,
                     scheduled_seconds=frames/20,all_offered_sensor_payload_MB_s=logical,
                     accepted_encoded_CL_MB_s="",partition_written_MB_per_scheduled_s="",
                     offered_payload_pct_of_linear=logical/capacity*100))
with (here/"bandwidth.csv").open("w") as f:
    w=csv.DictWriter(f,fieldnames=rows[0],lineterminator="\n")
    w.writeheader()
    w.writerows(rows)
