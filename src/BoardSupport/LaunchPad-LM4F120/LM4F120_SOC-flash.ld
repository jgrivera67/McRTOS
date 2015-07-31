/*
 * Linker script for LM4F120 SoC
 */

/*
 * MPU region alignment in bytes
 */
MPU_REGION_ALIGNMENT = 32;

/* Entry Point */
ENTRY(cortex_m_reset_handler)

/* Specify the memory areas */
MEMORY
{
  flash_interrupts  (rx) : ORIGIN = 0x00000000, LENGTH = 0x26C
  flash_text        (rx) : ORIGIN = 0x00000800, LENGTH = 256K - 0x800
  sram_data         (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}


/* Define output sections */
SECTIONS
{
  /* The startup code goes first into Flash */
  .interrupts :
  {
    . = ALIGN(4);
    KEEP(*(.vectortable)) /* Interrupt Vector Table */
    . = ALIGN(4);
  } >flash_interrupts

    /* The program code and other data goes into Flash */
  .text :
  {
    . = ALIGN(MPU_REGION_ALIGNMENT);
    __flash_text_start = .;
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(128 * 1024); /* MPU_REGION_ALIGNMENT);??? */
    __flash_text_end = .;
    __flash_initialized_data_start = .;
  } >flash_text

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > flash_text
  .ARM : {
    __exidx_start = .;
      *(.ARM.exidx*)
      __exidx_end = .;
  } >flash_text

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    __ram_initialized_data_start = .;
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    __ram_initialized_data_end = .;
  } >sram_data AT >flash_text

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    __uninitialized_data_start = .;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    __uninitialized_data_end = .;
  } >sram_data

  .reset_stack :
  {
        . = ALIGN(8);
	KEEP(*(.resetstack))	/* reset stack */
  } >sram_data

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

