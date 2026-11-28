"""Measure sustained Linux direct sequential writes in a fresh regular file on the card."""
import csv
import math
import mmap
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import time

card=pathlib.Path(sys.argv[1]).resolve(strict=True)
dev=subprocess.check_output(["findmnt","-T",str(card),"-n","-o","MAJ:MIN"],text=True).strip()
stats=pathlib.Path("/sys/dev/block")/dev/"stat"
results=pathlib.Path(__file__).resolve().parent/"linear-results"
results.mkdir(exist_ok=True)
MiB=1024**2
chunk=MiB
total=2*1024*MiB
assert shutil.disk_usage(card).free>total+256*MiB
clock=time.perf_counter_ns
with mmap.mmap(-1,16*MiB) as buf:
    buf[:]=os.urandom(len(buf))
    view=memoryview(buf)
    summaries=[]
    for rep in range(3):
        samples=[]
        with tempfile.TemporaryDirectory(prefix="copper-linear-",dir=card) as tmp:
            fd=os.open(pathlib.Path(tmp)/"linear.dat",os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_DIRECT,0o600)
            try:
                begin_setup=clock()
                os.posix_fallocate(fd,0,total)
                os.fdatasync(fd)
                setup_end=clock()
                sectors_before=int(stats.read_text().split()[6])
                start=clock()
                for offset in range(0,total,chunk):
                    t=clock()
                    i=offset%len(view)
                    n=os.write(fd,view[i:i+chunk])
                    if n!=chunk: raise OSError(f"short direct write {n} at offset {offset}")
                    samples.append((offset,chunk,t-start,clock()-start))
                sync_start=clock()
                os.fdatasync(fd)
                end=clock()
                sectors_after=int(stats.read_text().split()[6])
                assert os.fstat(fd).st_size==total
            finally:
                os.close(fd)
        with (results/f"run-{rep}.csv").open("w") as f:
            w=csv.writer(f,lineterminator="\n")
            w.writerow(["offset","bytes","begin_ns","end_ns"])
            w.writerows(samples)
        durations=sorted(e-b for _,_,b,e in samples)
        windows=[256*MiB/(samples[i+255][3]-samples[i][2])*1000 for i in range(0,len(samples),256)]
        rows=dict(run=rep,file_bytes=total,write_bytes=chunk,setup_ms=(setup_end-begin_setup)/1e6,
                  seconds=(end-start)/1e9,MB_s=total/(end-start)*1000,
                  last_sync_ms=(end-sync_start)/1e6,p99_write_ms=durations[math.ceil(.99*len(durations))-1]/1e6,
                  max_write_ms=durations[-1]/1e6,slowest_256MiB_MB_s=min(windows),last_256MiB_MB_s=windows[-1],
                  sectors_written=sectors_after-sectors_before)
        print(rows,flush=True)
        summaries.append(rows)
    view.release()
with (results/"summary.csv").open("w") as f:
    w=csv.DictWriter(f,fieldnames=summaries[0],lineterminator="\n")
    w.writeheader()
    w.writerows(summaries)
