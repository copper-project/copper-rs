"""Interleave two PR 2 policies on the card with the same 1 MiB size trigger."""
import os
import pathlib
import subprocess
import sys
import tempfile

here=pathlib.Path(__file__).resolve().parent
card=pathlib.Path(sys.argv[1]).resolve(strict=True)
dev=subprocess.check_output(["findmnt","-T",str(card),"-n","-o","MAJ:MIN"],text=True).strip()
stat=pathlib.Path("/sys/dev/block")/dev/"stat"
with (here/"age-build.log").open("w") as log:
    subprocess.run(["python3",here/"build.py","--age-one-second"],stdout=log,stderr=subprocess.STDOUT,check=True)
for rep in range(3):
    for case,n in [("pid",30000),("image",300)]:
        results=here/("age-"+case+"-results")
        results.mkdir(exist_ok=True)
        for idx,mode in enumerate(["inline","async"]):
            labels=["pr2","age1s"] if (rep+idx)%2==0 else ["age1s","pr2"]
            for label in labels:
                path=results/f"{label}-{mode}-{rep}.txt"
                if path.exists() and "PARTITION " in path.read_text():continue
                print(case,path.stem,flush=True)
                exe=here/label/(("pid-" if case=="pid" else "")+mode)
                before=list(map(int,stat.read_text().split()))
                with tempfile.TemporaryDirectory(dir=card) as tmp,path.open("w") as out:
                    subprocess.run([exe],cwd=tmp,env=dict(os.environ,TMPDIR=tmp,CU_FRAMES=str(n)),stdout=out,check=True,timeout=180)
                after=list(map(int,stat.read_text().split()))
                with path.open("a") as out:
                    print(f"PARTITION writes={after[4]-before[4]} sectors_written={after[6]-before[6]} sectors_read={after[2]-before[2]}",file=out)
