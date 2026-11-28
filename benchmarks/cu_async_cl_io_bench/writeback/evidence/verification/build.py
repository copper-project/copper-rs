"""Build isolated, pinned copies of the image example and worker acceptance test."""
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

here = pathlib.Path(__file__).resolve().parent
repo = pathlib.Path(os.environ.get("COPPER_REPO", here.parent.parent / "copper-rs")).resolve()
profile = here.parent / "profile"
labels = ("age1s",) if "--age-one-second" in sys.argv else ("pr2", "pr1", "base")
for label in labels:
    rev = (profile / ("pr2" if label=="age1s" else label) / "revision").read_text().strip()
    dest = here / label
    dest.mkdir(exist_ok=True)
    artifacts = dest / "target"
    env = dict(os.environ, CARGO_TARGET_DIR=str(artifacts), CARGO_BUILD_BUILD_DIR=str(artifacts), CARGO_BUILD_JOBS="8")
    with tempfile.TemporaryDirectory() as scratch:
        root = pathlib.Path(scratch)
        arc = subprocess.Popen(["git", "archive", rev], cwd=repo, stdout=subprocess.PIPE)
        subprocess.run(["tar", "-xm", "-C", scratch], stdin=arc.stdout, check=True)
        assert arc.wait() == 0
        if label == "age1s":
            tracker = root / "core/cu29_unifiedlog/src/memmap_writeback.rs"
            old = "const MAX_AGE: Duration = Duration::from_millis(100);"
            assert tracker.read_text().count(old) == 1
            tracker.write_text(tracker.read_text().replace(old, "const MAX_AGE: Duration = Duration::from_secs(1);"))
        ex = root / "examples/cu_image_aligner"
        shutil.copy(here / "image-main.rs", ex / "src/main.rs")
        config = ex / "copperconfig.ron"
        config.write_text(config.read_text().replace("    tasks:", "    logging: (copperlist_count: 2, slab_size_mib: 64, section_size_mib: 8, keyframe_interval: 1000000),\n    tasks:").replace('"width": 64','"width": 640').replace('"height": 48','"height": 480'))
        for mode in ("inline", "async"):
            cmd = ["cargo", "build", "--release", "--offline", "-p", "cu-image-aligner"]
            if mode == "async":
                cmd.extend(["--features", "cu29/async-cl-io"])
            subprocess.run(cmd, cwd=root, env=env, check=True)
            shutil.copy2(artifacts / "release/cu-image-aligner", dest / mode)
        pid = root / "examples/cu_reflect_demo"
        config = pid / "copperconfig.ron"
        config.write_text(config.read_text().replace("slab_size_mib: 16, keyframe_interval: 2", "copperlist_count: 2, slab_size_mib: 16, section_size_mib: 1, keyframe_interval: 1000"))
        manifest = pid / "Cargo.toml"
        manifest.write_text(manifest.read_text().replace("[dependencies]", "[dependencies]\ntempfile = { workspace = true }"))
        main = pid / "src/main.rs"
        tasks = main.read_text().split("pub mod tasks {",1)[1].split("#[copper_runtime",1)[0]
        tasks = tasks.replace("pub value: f32,", "pub value: f32,\n        pub samples: [[[u8; 32]; 8]; 8],")
        tasks = tasks.replace("self.next_measurement += 0.05;", "self.next_measurement += 0.05;\n            if self.next_measurement > 1.25 { self.next_measurement = 0.75; }")
        tasks = tasks.replace("value: self.next_measurement,", "value: self.next_measurement,\n                samples: [[[(self.next_measurement * 64.0) as u8; 32]; 8]; 8],")
        instrumented = (here / "image-main.rs").read_text()
        instrumented = instrumented.replace('pub mod tasks;', 'use cu_pid::{GenericPIDTask, PIDControlOutputPayload};\nuse cu29::bincode::{Decode, Encode};\npub mod tasks {' + tasks)
        instrumented = instrumented.replace("ImageAlignerApp","PidApp").replace("image-aligner.copper","pid.copper").replace("Some(64 << 20)","Some(16 << 20)").replace("Duration::from_millis(i as u64 * 50)", "Duration::from_micros(i as u64 * 1000)")
        main.write_text(instrumented)
        for mode in ("inline", "async"):
            cmd = ["cargo","build","--release","--offline","-p","cu-reflect-demo"]
            if mode == "async":
                cmd.extend(["--features","cu29/async-cl-io"])
            subprocess.run(cmd,cwd=root,env=env,check=True)
            shutil.copy2(artifacts / "release/cu-reflect-demo",dest / ("pid-" + mode))
        if label == "pr2":
            ex = root / "benchmarks/cu_async_cl_io_bench"
            main = ex / "src/main.rs"
            main.write_text(main.read_text().split("fn main() -> CuResult<()> {")[0] + (here / "barrier-main.rs").read_text())
            config = ex / "copperconfig.ron"
            config.write_text(re.sub(r"keyframe_interval: \d+", "keyframe_interval: 1",config.read_text()))
            mm = root / "core/cu29_unifiedlog/src/memmap.rs"
            signature = "fn flush_range(&self, offset: usize, len: usize) -> CuResult<()> {"
            assert mm.read_text().count(signature) == 1
            mm.write_text(mm.read_text().replace(signature, signature + """
        if std::thread::current().name() == Some("cu-output")
            && std::path::Path::new("arm").exists() && !std::path::Path::new("release").exists()
        {
            std::fs::write("held", b"").unwrap();
            let wait = std::time::Instant::now();
            while !std::path::Path::new("release").exists() {
                assert!(wait.elapsed() < std::time::Duration::from_secs(8), "producer stalled behind mmap flush");
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        }"""))
            subprocess.run(["cargo","build","--release","--offline","-p","cu-async-cl-io-bench","--features","async-cl-io"],cwd=root,env=env,check=True)
            shutil.copy2(artifacts / "release/cu-async-cl-io-bench", dest / "barrier")
    (dest / "revision").write_text(rev + "\n")
