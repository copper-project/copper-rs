# Commands arrive over the parent process's private pipe. EOF restores flash execution.
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
set failed [catch {
    reset halt
    load_image reader.elf 0 elf 0x24000000 0x80000
    mww $ready_address 0
    mww 0xe000ed08 0x24000000
    reg msp [mrw 0x24000000]
    resume [expr {[mrw 0x24000004] & ~1}]
    adapter speed 16000
    puts stdout READY
    flush stdout
    while {[gets stdin command] >= 0} {
        if {[catch {eval $command} result]} {
            puts stdout "ERROR $result"
        } else {
            puts stdout "OK $result"
        }
        flush stdout
    }
} message]
reset run
if {$failed} { error $message }
shutdown
