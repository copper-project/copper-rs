/* Memory layout for STM32H743VIT6 */
MEMORY {
    /* Reader code and data initializers live in the first 64 KiB of AXI SRAM */
    FLASH (rx)  : ORIGIN = 0x24000000, LENGTH = 64K

    /* AXI SRAM - main RAM region, 512KB */
    RAM   (xrw) : ORIGIN = 0x24010000, LENGTH = 448K
}

/* Map standard regions to physical memory */
REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
