"""Run the writeback comparison sequentially, resuming from complete measurements."""
import argparse, gzip, os, pathlib, shutil, subprocess, tempfile
p=argparse.ArgumentParser()
p.add_argument('directory',type=pathlib.Path,help='writable directory on the device under test')
p.add_argument('results',type=pathlib.Path,help='result directory on the host')
p.add_argument('--smoke',action='store_true',help='run 6 ticks per workload and one repetition')
args=p.parse_args()
base=pathlib.Path(__file__).resolve().parent
dest=args.results.resolve()
dest.mkdir(exist_ok=True,parents=True)
storage=args.directory.resolve(strict=True)
if shutil.disk_usage(storage).free < 2*1024**3 and not args.smoke:
    raise SystemExit('2 GiB free required on the benchmark filesystem')
runs=[('normal',s,m) for s in (2,8) for m in ('inline','async')]+[('rollover',2,m) for m in ('inline','async')]
with (dest/'environment.txt').open('w') as f:
    for cmd in (['uname','-a'],['getconf','PAGESIZE'],['findmnt','-T',str(storage)],['lsblk','-o','NAME,MODEL,TRAN,FSTYPE,SIZE,MOUNTPOINTS'],['rustc','-Vv']):
        subprocess.run(cmd,stdout=f,check=True)
    f.write(f'payload_bytes=750000 period_ms=50 iterations={6 if args.smoke else 1200} warmup=0 normal_slab_mib=256 normal_section_mib=8 stress_slab_mib=4 stress_section_kib=1536 mmap-fsync=off profile=release\n')
for rep in range(1 if args.smoke else 3):
    for case,slots,mode in runs:
        for rev in ('base','pr1','pr2'):
            for spatial in ([False,True] if rep==0 and slots==2 else [False]):
                label=f'{rev}-{case}-{slots}-{mode}-{rep}'+('-spatial' if spatial else '')
                done=dest/(label+'.ok')
                if done.exists():continue
                raw=dest/(label+'.csv')
                with tempfile.TemporaryDirectory(prefix='cu-writeback-',dir=storage) as tmp:
                    env=dict(os.environ,TMPDIR=tmp,CU_PROBE_OUTPUT=str(raw))
                    if case=='rollover':env['CU_PROBE_STRESS']='1'
                    if spatial:env['CU_PROBE_STORES']='1'
                    cmd=[str(base/rev/f'{slots}-{mode}'),'--iterations',str(6 if args.smoke else 80 if spatial else 1200),'--payload-bytes','750000','--spin-iters','0','--warmup','0','--section-mib','8','--slab-mib','4' if case=='rollover' else '256']
                    print(label,flush=True)
                    with (dest/(label+'.txt')).open('w') as f:
                        subprocess.run(cmd,env=env,stdout=f,check=True)
                with raw.open('rb') as src,gzip.open(str(raw)+'.gz','wb') as output:shutil.copyfileobj(src,output)
                raw.unlink()
                done.write_text((base/rev/'revision').read_text())
