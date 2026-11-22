/* STM32H743VIT6 memory layout (FLASH + AXI SRAM) */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   (xrw) : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM_D2 (xrw) : ORIGIN = 0x30000000, LENGTH = 256K
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

SECTIONS {
  .init_array : ALIGN(4)
  {
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
    KEEP(*(.init_array))
  } > FLASH
} INSERT AFTER .text;

/* SRAM1 and SRAM2 are contiguous. Startup enables their clocks before use. */
SECTIONS {
  .sd_heap (NOLOAD) : {
    *(.sd_heap)
  } > SRAM_D2
} INSERT BEFORE .bss;
