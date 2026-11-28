"""Build the same isolated measurement overlay against a named Git revision."""
import pathlib, subprocess, sys, os, shutil, re
root=pathlib.Path(sys.argv[1]).resolve()
output=pathlib.Path(sys.argv[2]).resolve()
output.mkdir(parents=True,exist_ok=True)
repo=output/'source'
repo.mkdir(exist_ok=True)
rev=subprocess.check_output(['git','-C',root,'rev-parse','HEAD'],text=True).strip()
(output/'revision').write_text(rev+'\n')
arc=subprocess.Popen(['git','-C',root,'archive',rev],stdout=subprocess.PIPE)
subprocess.run(['tar','-x','-C',repo],stdin=arc.stdout,check=True)
assert arc.wait()==0
def edit(path,old,new):
    p=repo/path
    text=p.read_text()
    if old not in text: raise RuntimeError((path,old))
    p.write_text(text.replace(old,new))
traits='core/cu29_traits/src/lib.rs'
p=repo/traits
p.write_text(p.read_text()+'\n#[cfg(feature="std")]\npub mod timing;\n')
shutil.copy(pathlib.Path(__file__).with_name('timing.rs'),p.with_name('timing.rs'))
mm='core/cu29_unifiedlog/src/memmap.rs'
edit(mm,'struct MappedFile {','struct MappedFile {\n    bench_index: usize,')
edit(mm,'        Ok(Self {\n            mapping,','''        Ok(Self {
            bench_index: {
                use std::os::fd::AsRawFd;
                let path=std::fs::read_link(format!("/proc/self/fd/{}",file.as_raw_fd()))?;
                path.file_stem().unwrap().to_str().unwrap().rsplit('_').next().unwrap().parse().unwrap()
            },
            mapping,''')
if 'fn flush_range' in (repo/mm).read_text():
    edit(mm,'fn flush_range(&self, offset: usize, len: usize) -> CuResult<()> {','fn flush_range(&self, offset: usize, len: usize) -> CuResult<()> {\n        let _measure=cu29_traits::timing::span("flush",self.bench_index,offset,len);')
else:
    edit(mm,'Mapping::Write(mapping) => {','Mapping::Write(mapping) => {\n                    let _measure=cu29_traits::timing::span("flush",map.bench_index,0,map.length);')
# Page dirty volume for peak estimates comes from the spatial pass, separate from latency.
edit(mm,'            Ok(())\n        })\n    }\n    fn commit','''            if TRACE_STORES.get_or_init(|| std::env::var_os("CU_PROBE_STORES").is_some()).to_owned() {
                drop(cu29_traits::timing::span("store",map.bench_index,local,count));
            }
            Ok(())
        })
    }
    fn commit''')
edit(mm,'const MAX_SPANS:', 'static TRACE_STORES: std::sync::OnceLock<bool> = std::sync::OnceLock::new();\nconst MAX_SPANS:')
bl='core/cu29_unifiedlog/src/byte_log.rs'
def trigger(path,signature,label):
    edit(path,signature,signature+'\n        #[cfg(feature="std")] let _cause=cu29_traits::timing::trigger("'+label+'");')
for sig,label in [('fn publish(&mut self) -> CuResult<()> {','publish'),('pub fn close(&mut self) -> CuResult<()> {','close'),('fn flush(&mut self) -> CuResult<usize> {','section flush')]:
    trigger(bl,sig,label)
if (repo/'core/cu29_unifiedlog/src/memmap_writeback.rs').exists() and 'self.submit(all)?;' in (repo/'core/cu29_unifiedlog/src/memmap_writeback.rs').read_text():
    edit('core/cu29_unifiedlog/src/memmap_writeback.rs','let all = pending.urgent || left.is_zero();','let all = pending.urgent || left.is_zero();')
    edit('core/cu29_unifiedlog/src/memmap_writeback.rs','            all\n        };\n        self.submit(all)?;', '''            (all, if pending.urgent { "section close" } else if left.is_zero() { "age" } else { "bytes" })
        };
        let _cause=cu29_traits::timing::trigger(all.1);
        self.submit(all.0)?;''')
runtime='core/cu29_runtime/src/curuntime.rs'
text=(repo/runtime).read_text()
for name,kind in [('write_completed_output','record'),('finish_serial_output','submission')]:
    text=re.sub(r'(fn '+name+r'\b[^{]+\{)', lambda m:m[0]+'\n        #[cfg(feature="std")] let _measure=cu29_traits::timing::span("'+kind+'",0,0,0);',text)
(repo/runtime).write_text(text)
macro='core/cu29_derive/src/runtime_macro.rs'
edit(macro,'let local_copperlist_sink = Local::open(&unified_logger, default_section_size, log_context)?;', '''if std::env::var_os("CU_PROBE_STRESS").is_some() { default_section_size = 1536 * 1024; }
                let local_copperlist_sink = Local::open(&unified_logger, default_section_size, log_context)?;''')
bench='benchmarks/cu_async_cl_io_bench/'
edit(bench+'Cargo.toml','[dependencies]','[dependencies]\ncu29-traits = { workspace = true }')
main=bench+'src/main.rs'
edit(main,'    for _ in 0..args.iterations {','''    let stress = std::env::var_os("CU_PROBE_STRESS").is_some();
    for tick in 0..args.iterations {
        std::thread::sleep((benchmark_start + std::time::Duration::from_millis(u64::from(tick)*50)).saturating_duration_since(Instant::now()));
        let _measure=cu29_traits::timing::span("tick",0,0,0);
        if stress { for sample in 0u32..8 { info!("Rollover record {} {}",tick,sample); } }''')
edit(main,'    let mut stopped = app.stop()?.into_inner();','''    let drain=cu29_traits::timing::span("final_drain",0,0,0);
    let mut stopped = app.stop()?.into_inner();''')
edit(main,'    drop(stopped);','''    drop(stopped);
    drop(drain);
    cu29_traits::timing::dump(std::path::Path::new(&std::env::var_os("CU_PROBE_OUTPUT").unwrap()));''')
target=pathlib.Path(os.environ['CARGO_TARGET_DIR'])
for slots in [2,8]:
    ron=repo/(bench+'copperconfig.ron')
    ron.write_text(re.sub(r'copperlist_count: \d+',f'copperlist_count: {slots}',ron.read_text()))
    for mode in ['inline','async']:
        cmd=['cargo','build','--release','--offline','-p','cu-async-cl-io-bench']
        if mode=='async':cmd+=['--features','async-cl-io']
        subprocess.run(cmd,cwd=repo,check=True)
        shutil.copy2(target/'release/cu-async-cl-io-bench',output/f'{slots}-{mode}')
