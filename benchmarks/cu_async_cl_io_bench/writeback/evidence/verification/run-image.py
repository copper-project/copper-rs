"""Run two-camera image alignment, rotating revision order between repetitions."""
import os
import pathlib
import subprocess
import sys
import tempfile

here = pathlib.Path(__file__).resolve().parent
card = pathlib.Path(sys.argv[1]).resolve(strict=True)
dev = subprocess.check_output(["findmnt","-T",str(card),"-n","-o","MAJ:MIN"],text=True).strip()
stat_path = pathlib.Path("/sys/dev/block")/dev/"stat"
pid = len(sys.argv) == 3 and sys.argv[2] == "pid"
results = here / ("pid-results" if pid else "image-results")
results.mkdir(exist_ok=True)
revisions = ["base", "pr1", "pr2"]
for rep in range(3):
    for mode in ["inline", "async"]:
        for rev in revisions[rep:]+revisions[:rep]:
            path = results / f"{rev}-{mode}-{rep}.txt"
            if path.exists() and "PARTITION " in path.read_text():
                continue
            print(path.stem, flush=True)
            before = list(map(int,stat_path.read_text().split()))
            with tempfile.TemporaryDirectory(prefix="copper-camera-",dir=card) as tmp, path.open("w") as out:
                binary = "pid-" + mode if pid else mode
                subprocess.run([here/rev/binary], cwd=tmp, env=dict(os.environ,TMPDIR=tmp,CU_FRAMES="30000" if pid else "300"),stdout=out,check=True,timeout=180)
            after = list(map(int,stat_path.read_text().split()))
            with path.open("a") as out:
                print(f"PARTITION writes={after[4]-before[4]} sectors_written={after[6]-before[6]} sectors_read={after[2]-before[2]}",file=out)
