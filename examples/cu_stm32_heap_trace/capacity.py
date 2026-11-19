"""Run with gdb -nx -batch FIRMWARE -x capacity.py."""
import pathlib
import shlex

import gdb

elf = pathlib.Path(gdb.current_progspace().filename)
gdb.execute("set pagination off")
gdb.execute("set confirm off")
gdb.execute("set language c")
gdb.execute("set remotetimeout 15")


def boot(size):
    gdb.execute("target remote | exec qemu-system-arm -M netduinoplus2 -nographic "
                "-monitor none -serial none -S -gdb stdio -kernel " + shlex.quote(str(elf)),
                to_string=True)
    points = [gdb.Breakpoint("*" + name, internal=True)
              for name in ("heap_capacity", "heap_complete", "heap_out_of_memory")]
    try:
        for point in points:
            point.silent = True
        gdb.execute("continue", to_string=True)
        if not points[0].hit_count:
            raise gdb.GdbError("Firmware stopped before heap initialization")
        gdb.execute(f"set {{unsigned int}}&heap_limit = {size}")
        gdb.execute("continue", to_string=True)
        if points[1].hit_count:
            return tuple(int(gdb.parse_and_eval(reg)) for reg in ("$r0", "$r1", "$r2"))
        if points[2].hit_count:
            return None
        raise gdb.GdbError("Firmware stopped before completing the first iteration")
    finally:
        for point in points:
            point.delete()
        try:
            gdb.execute("kill", to_string=True)
        except gdb.error:
            # QEMU can close stdio before acknowledging the kill packet.
            if gdb.selected_inferior().pid:
                gdb.execute("disconnect", to_string=True)


maximum = 96 * 1024
free, largest, peak = boot(maximum)
print(f"96 KiB arena: {free} bytes free, largest 4-byte-aligned request {largest} bytes")
# For this deterministic graph the live aligned allocations at peak give a
# lower bound on the required arena. Try every word size above that bound.
for size in range((peak + 3) & ~3, maximum + 1, 4):
    if boot(size) is not None:
        print(f"Minimum arena through first iteration: {size} bytes (peak {peak})")
        break
