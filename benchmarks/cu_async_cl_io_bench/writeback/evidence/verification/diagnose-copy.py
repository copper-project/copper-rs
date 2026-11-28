"""Separate diagnostic trial for slow ticks outside a baseline msync."""
import csv
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

here = pathlib.Path(__file__).resolve().parent
base = here.parent/"profile/base"
dest = here/"copy-results"
dest.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory() as scratch:
    root = pathlib.Path(scratch)/"source"
    shutil.copytree(base/"source",root,copy_function=shutil.copy)
    manifest = root/"core/cu29_unifiedlog/Cargo.toml"
    manifest.write_text(manifest.read_text().replace("[dependencies]",'[dependencies]\nlibc = "0.2"'))
    mm = root/"core/cu29_unifiedlog/src/memmap.rs"
    s = mm.read_text()
    pos = s.index("            unsafe {",s.index("fn write(&mut self, offset: u64, bytes: &[u8])"))
    s = s[:pos] + """
            let faults = || {
                let mut usage = std::mem::MaybeUninit::uninit();
                assert_eq!(unsafe { libc::getrusage(libc::RUSAGE_THREAD, usage.as_mut_ptr()) }, 0);
                let usage = unsafe { usage.assume_init() };
                (usage.ru_minflt,usage.ru_majflt)
            };
            let probe = (count >= 65536).then(|| (faults(), cu29_traits::timing::span("payload memcpy",map.bench_index,local,count)));
""" + s[pos:]
    pos = s.index("            if TRACE_STORES",pos)
    s = s[:pos] + """
            if let Some(((minor,major),span)) = probe {
                drop(span);
                let (minor_after,major_after) = faults();
                drop(cu29_traits::timing::span("fault delta",map.bench_index,(minor_after-minor) as usize,(major_after-major) as usize));
            }
""" + s[pos:]
    mm.write_text(s)
    config = root/"benchmarks/cu_async_cl_io_bench/copperconfig.ron"
    config.write_text(re.sub(r"copperlist_count: \d+","copperlist_count: 2",config.read_text()))
    target = dest/"target"
    env = dict(os.environ,CARGO_TARGET_DIR=str(target),CARGO_BUILD_BUILD_DIR=str(target))
    subprocess.run(["cargo","build","--release","--offline","-p","cu-async-cl-io-bench"],cwd=root,env=env,check=True)
    with tempfile.TemporaryDirectory(dir=sys.argv[1]) as card, (dest/"output.txt").open("w") as out:
        subprocess.run([target/"release/cu-async-cl-io-bench","--iterations","1200","--warmup","0","--spin-iters","0","--payload-bytes","750000","--slab-mib","256","--section-mib","8"],env=dict(env,TMPDIR=card,CU_PROBE_OUTPUT=str(dest/"events.csv")),stdout=out,check=True)
with (dest/"events.csv").open() as f:
    timeline=list(csv.DictReader(f))
def spans(kind):
    return sorted((int(e["begin_ns"]),int(e["end_ns"]),int(e["offset"]),int(e["bytes"])) for e in timeline if e["kind"]==kind)
copies=spans("payload memcpy")
faults=spans("fault delta")
assert len(copies)==len(faults) and copies
ticks=spans("tick")
flush=spans("flush")
with (dest/"misses.csv").open("w") as f:
    w=csv.writer(f)
    w.writerow(["tick","duration_ms","flush_ms","payload_copy_ms","minor_faults_inside_copy","major_faults_inside_copy"])
    for i,(b,e,_,_) in enumerate(ticks):
        if e-b<=50_000_000: continue
        inside=[(c,d) for c,d in zip(copies,faults) if b<=c[0]<e]
        overlap=sum(max(0,min(e,y)-max(b,x)) for x,y,_,_ in flush)
        w.writerow([i,(e-b)/1e6,overlap/1e6,sum(c[1]-c[0] for c,d in inside)/1e6,sum(d[2] for c,d in inside),sum(d[3] for c,d in inside)])
