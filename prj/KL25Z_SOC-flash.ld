/*
 * Linker script for KL25Z SoC
 *  Default linker command file for Flash targets
 */

/*
 * Hardware micro trace buffer size in bytes
 */
MICRO_TRACE_BUFFER_SIZE_IN_BYTES = 1024;

/* Entry Point */
ENTRY(cortex_m_reset_handler)

/* Specify the memory areas */
MEMORY
{
  flash_interrupts  (rx) : ORIGIN = 0x00000000, LENGTH = 0xC0
  flash_cfmprotrom  (rx) : ORIGIN = 0x00000400, LENGTH = 0x10
  flash_text        (rx) : ORIGIN = 0x00000800, LENGTH = 128K - 0x800
  sram_data         (rwx) : ORIGIN = 0x1FFFF000, LENGTH = 16K
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
  
  .cfmprotect :
  {
    . = ALIGN(4);
	KEEP(*(.cfmconfig))	/* Flash Configuration Field (FCF) */
	. = ALIGN(4);
  } >flash_cfmprotrom

  /* The program code and other data goes into Flash */
  .text :
  {
    . = ALIGN(4);
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

    . = ALIGN(4);
    __flash_text_end = .; 
    __flash_initialized_data_start = .; 
  } >flash_text

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > flash_text
  .ARM : {
    __exidx_start = .;
      *(.ARM.exidx*)
      __exidx_end = .;
  } >flash_text
 
  /* Reserve MTB memory at the beginning of sram_data */
  .mtb :
  {
    . = ALIGN(8);
    __micro_trace_buffer = .;
    . += MICRO_TRACE_BUFFER_SIZE_IN_BYTES;
    . = ALIGN(8);
    __micro_trace_buffer_end = .;
  } >sram_data

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
