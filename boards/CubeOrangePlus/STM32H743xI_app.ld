/*
 * CubeOrangePlus application linker script.
 *
 * The ArduPilot/CubePilot BL5 bootloader occupies the first 128 KB
 * (0x08000000–0x0801FFFF) of STM32H743 flash.  Application code must be
 * compiled to run from 0x08020000 onward so that vector table entries
 * contain the correct absolute addresses.
 *
 * Memory layout:
 *   Bootloader  : 0x08000000  128 K   (reserved, not ours)
 *   Application : 0x08020000  1792 K  (this firmware)
 *   Calibration : 0x081E0000  128 K   (Bank 2 sector 7 — persistent cal data)
 */

MEMORY
{
    flash0 (rx) : org = 0x08020000, len = 1792k      /* App flash (after 128 KB BL) */
    cal    (r)  : org = 0x081E0000, len = 128k        /* Calibration sector (Bank2 S7) */
    flash1 (rx) : org = 0x08000000, len = 1M         /* Bank 1 (not directly used)  */
    flash2 (rx) : org = 0x08100000, len = 1M         /* Bank 2 (not directly used)  */
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x24000000, len = 512k       /* AXI SRAM  */
    ram1   (wx) : org = 0x30000000, len = 256k       /* AHB SRAM1+SRAM2 */
    ram2   (wx) : org = 0x30000000, len = 288k       /* AHB SRAM1+SRAM2+SRAM3 */
    ram3   (wx) : org = 0x30040000, len = 32k        /* AHB SRAM3 */
    ram4   (wx) : org = 0x38000000, len = 64k        /* AHB SRAM4 */
    ram5   (wx) : org = 0x20000000, len = 128k       /* DTCM-RAM  */
    ram6   (wx) : org = 0x00000000, len = 64k        /* ITCM-RAM  */
    ram7   (wx) : org = 0x38800000, len = 4k         /* BCKP SRAM */
}

REGION_ALIAS("VECTORS_FLASH",     flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);
REGION_ALIAS("XTORS_FLASH",       flash0);
REGION_ALIAS("XTORS_FLASH_LMA",   flash0);
REGION_ALIAS("TEXT_FLASH",        flash0);
REGION_ALIAS("TEXT_FLASH_LMA",    flash0);
REGION_ALIAS("RODATA_FLASH",      flash0);
REGION_ALIAS("RODATA_FLASH_LMA",  flash0);
REGION_ALIAS("VARIOUS_FLASH",     flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);
REGION_ALIAS("RAM_INIT_FLASH_LMA",flash0);

REGION_ALIAS("MAIN_STACK_RAM",    ram5);
REGION_ALIAS("PROCESS_STACK_RAM", ram5);
REGION_ALIAS("DATA_RAM",          ram0);
REGION_ALIAS("DATA_RAM_LMA",      flash0);
REGION_ALIAS("BSS_RAM",           ram0);
REGION_ALIAS("HEAP_RAM",          ram0);
REGION_ALIAS("NOCACHE_RAM",       ram3);
REGION_ALIAS("ETH_RAM",           ram3);

INCLUDE rules_stacks.ld

SECTIONS
{
    .nocache (NOLOAD) : ALIGN(4)
    {
        __nocache_base__ = .;
        *(.nocache)
        *(.nocache.*)
        *(.bss.__nocache_*)
        . = ALIGN(4);
        __nocache_end__ = .;
    } > NOCACHE_RAM

    .eth (NOLOAD) : ALIGN(4)
    {
        __eth_base__ = .;
        *(.eth)
        *(.eth.*)
        *(.bss.__eth_*)
        . = ALIGN(4);
        __eth_end__ = .;
    } > ETH_RAM
}

INCLUDE rules_code.ld
INCLUDE rules_data.ld
INCLUDE rules_memory.ld
