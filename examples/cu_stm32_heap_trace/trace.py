"""Run inside GDB from the example directory. See README.md."""

import collections
import pathlib
import shlex

import gdb


def stack():
    frame = gdb.newest_frame().older()
    frames = []
    while frame:
        location = frame.find_sal()
        source = ""
        if location.symtab:
            source = f" at {location.symtab.filename}:{location.line}"
        frames.append(f"{frame.name() or hex(frame.pc())}{source}")
        frame = frame.older()
    return "\n    ".join(frames)


elf = pathlib.Path(gdb.current_progspace().filename)
gdb.execute("set pagination off")
gdb.execute("set language c")
gdb.execute("set remotetimeout 15")
gdb.execute("target remote | exec qemu-system-arm -M netduinoplus2 -nographic "
            "-monitor none -serial none -S -gdb stdio -kernel " + shlex.quote(str(elf)))
event = gdb.Breakpoint("*heap_event", internal=True)
event.silent = True
event_pc = int(gdb.parse_and_eval("(unsigned)&heap_event")) & ~1

phases = ["before builder", "built", "started", "first iteration complete"]
live = {}
at_peak = {}
peak = 0
startup = True
released = {}
trace_path = elf.with_name(elf.stem + "-heap-trace.txt")
try:
    with trace_path.open("w") as trace:
        while True:
            gdb.execute("continue", to_string=True)
            if int(gdb.parse_and_eval("$pc")) != event_pc:
                raise gdb.GdbError("Firmware stopped outside heap_event; inspect bt")
            kind, ptr, size, used = (
                int(gdb.parse_and_eval(f"$r{i}").cast(gdb.lookup_type("unsigned int")))
                for i in range(4)
            )
            if kind == 0:
                if not ptr:
                    raise gdb.GdbError(f"OOM requesting {size} bytes with {used} bytes used")
                site = stack()
                # The sequence number distinguishes allocations that reuse an address.
                live[ptr] = (trace.tell(), size, site)
                print(f"alloc {ptr:#x} size={size} used={used}\n    {site}", file=trace)
                if startup and used > peak:
                    peak = used
                    at_peak = live.copy()
                    released.clear()
            elif kind == 1:
                ident, original_size, site = live.pop(ptr)
                if size != original_size:
                    raise gdb.GdbError("Layout mismatch on free")
                drop = stack()
                print(f"free {ptr:#x} size={size} used={used} alloc_record={ident}\n    {drop}", file=trace)
                if startup and ptr in at_peak and at_peak[ptr][0] == ident:
                    released[ident] = drop
            elif kind == 2:
                message = f"{phases[ptr]}: {used} allocator bytes"
                print(message, file=trace)
                print(message)
                if ptr == 2:
                    startup = False
                if ptr == 3:
                    break
            else:
                raise gdb.GdbError("Firmware panic:\n" + stack())

        groups = collections.Counter()
        for ident, size, site in at_peak.values():
            if ident in released:
                groups[site, released[ident]] += size
        print(f"Startup peak: {peak} allocator bytes")
        print("Blocks live at peak and freed by started (grouped by both call stacks):")
        for (site, drop), size in groups.most_common():
            print(f"{size} requested bytes\n  allocated in:\n    {site}\n  freed in:\n    {drop}", file=trace)
        print(f"{sum(groups.values())} requested bytes released across {len(groups)} stack pairs")
        print(f"Full allocation and release stacks: {trace_path}")
finally:
    try:
        gdb.execute("set confirm off")
        gdb.execute("kill", to_string=True)
    except gdb.error:
        # QEMU can close the remote connection before acknowledging kill.
        pass
