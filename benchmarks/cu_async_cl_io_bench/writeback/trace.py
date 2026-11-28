"""Trace a pinned checkout's host mmap requests and logical writes.

Usage: python3 trace.py CHECKOUT OUTPUT_DIRECTORY
The probe runs against a temporary copy with a single diagnostic after memcpy.
"""
import csv
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile


root, out = (pathlib.Path(arg).resolve() for arg in sys.argv[1:])
out.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory() as tmp:
    repo = pathlib.Path(tmp)
    # Keep all workspace dependency paths and versions from the selected revision.
    rev = subprocess.check_output(['git', '-C', root, 'rev-parse', 'HEAD'], text=True).strip()
    archive = subprocess.Popen(['git', '-C', root, 'archive', rev], stdout=subprocess.PIPE)
    subprocess.run(['tar', '-x', '-C', repo], stdin=archive.stdout, check=True)
    assert archive.wait() == 0
    p = repo / 'core/cu29_unifiedlog/src/memmap.rs'
    src = p.read_text()
    pos = src.index('            Ok(())', src.index('bytes[done..].as_ptr()'))
    p.write_text(src[:pos] + '            std::io::Write::write_all(&mut std::io::stderr(), format!("STORE {:p} {}\\n", map.pointer.wrapping_add(local), count).as_bytes()).unwrap();\n' + src[pos:])
    probe = repo / 'core/cu29_unifiedlog/examples/writeback_probe.rs'
    probe.parent.mkdir(exist_ok=True)
    shutil.copyfile(pathlib.Path(__file__).with_name('probe.rs'), probe)
    subprocess.run(['cargo', 'build', '--offline', '-p', 'cu29-unifiedlog', '--example', 'writeback_probe',
                    '--target-dir', str(repo / 'target')], cwd=repo, check=True)
    cmd = ['strace', '-f', '-r', '-T', '-y', '-s', '160', '-e', 'trace=mmap,msync,fsync,ftruncate,write',
           '-o', str(out/'syscalls.txt'), str(repo/'target/debug/examples/writeback_probe')]
    with (out/'events.txt').open('w') as log:
        subprocess.run(cmd, cwd=out, stderr=log, check=True)

maps = {}
events = []
when = 0.0
for line in (out/'syscalls.txt').read_text().splitlines():
    thread, delta, call = line.split(maxsplit=2)
    when += float(delta)
    m = re.search(r'mmap\(NULL, (\d+), .*?MAP_SHARED.*?probe_(\d+)\.copper>, 0\) = (0x\w+)', call)
    if m:
        size, slab, addr = m.groups()
        maps[int(addr,16)] = (int(slab),int(size))
    m = re.search(r'STORE (0x\w+) (\d+)', call)
    f = re.search(r'msync\((0x\w+), (\d+), MS_SYNC\)\s+= 0 <([\d.]+)>', call)
    if not (m or f):
        continue
    addr, length = (m or f).groups()[:2]
    addr = int(addr,16)
    base = next(b for b, (_, sz) in maps.items() if b <= addr < b+sz)
    slab, size = maps[base]
    assert addr+int(length) <= base+size
    event = [when,thread,'write' if m else 'request',slab,addr-base,int(length)]
    events.append(event)
    if f:
        events.append([when+float(f[3]),thread,'complete',*event[3:]])
assert any(event[2] == 'write' for event in events)
assert any(event[2] == 'request' for event in events)
with (out/'ranges.csv').open('w') as dest:
    writer=csv.writer(dest)
    writer.writerow(['seconds','tid','event','slab','offset','length'])
    writer.writerows(events)
svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1030" height="%d">' % (90+18*len(events)),
       '<rect width="100%" height="100%" fill="black"/>',
       '<g font-family="monospace" font-size="12" fill="white">',
       f'<text x="8" y="18">{rev[:12]}   512 B/cell   slab 0 | slab 1   D descriptor   C commit   P payload</text>',
       '<text x="8" y="39">purple pending writes   red syscall range   green completed pages</text>']
dirty=set()
for row,(_,_,kind,slab,offset,length) in enumerate(events):
    y=52+18*row
    blocks=set(range(slab*16+offset//512,slab*16+(offset+length-1)//512+1))
    if kind=='write':
        dirty.update(blocks)
    elif kind=='complete':
        dirty.difference_update(blocks)
    svg.append(f'<text x="8" y="{y+12}">{row+1:02} {kind:8} {length:4} B</text>')
    for i in range(32):
        x=170+26*i+(10 if i>=16 else 0)
        fill='#7840af' if i in dirty else '#262626'
        stroke='#ff5757' if i in blocks and kind=='request' else '#62d985' if i in blocks and kind=='complete' else '#333'
        label='H' if i==0 else 'D' if i in (1,9,12,20) else 'C' if i in (2,10,13,21) else 'P' if i<28 else ''
        svg.append(f'<rect x="{x}" y="{y}" width="24" height="15" fill="{fill}" stroke="{stroke}"/><text x="{x+8}" y="{y+12}">{label}</text>')
svg.append('</g></svg>')
(out/'ranges.svg').write_text('\n'.join(svg))
