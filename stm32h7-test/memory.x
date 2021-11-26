MEMORY
{
  FLASH  : ORIGIN = 0x08000000, LENGTH = 1M

  /* DTCM  */
  RAM    : ORIGIN = 0x20000000, LENGTH = 128K

  /* AXISRAM */
  AXISRAM : ORIGIN = 0x24000000, LENGTH = 128K

  /* SRAM */
  SRAM1 : ORIGIN = 0x30000000, LENGTH = 16K
  SRAM2 : ORIGIN = 0x30004000, LENGTH = 16K
  SRAM4 : ORIGIN = 0x38000000, LENGTH = 16K

  /* Backup SRAM */
  BSRAM : ORIGIN = 0x38800000, LENGTH = 4K

  /* Instruction TCM */
  ITCM  : ORIGIN = 0x00000000, LENGTH = 16K
}

/* The location of the stack can be overridden using the
   `_stack_start` symbol.  Place the stack at the end of RAM */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the
   `_stext` symbol.  By default it will place after .vector_table */
/* _stext = ORIGIN(FLASH) + 0x40c; */

/* These sections are used for some of the examples */
SECTIONS {
  .axisram (NOLOAD) : ALIGN(8) {
    *(.axisram .axisram.*);
    . = ALIGN(8);
    } > AXISRAM
  /* The SRAM1 and SRAM2 section are commonly used as the stack and heap for the
     CM4 core in dualcore versions and should thus not be used in examples*/
  .sram4 (NOLOAD) : ALIGN(4) {
    *(.sram4 .sram4.*);
    . = ALIGN(4);
    } > SRAM4
} 
