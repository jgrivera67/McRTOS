/*
*****************************************************************************
**
**  File        : KL25Z128M4_flash.ld
**
**  Default linker command file for Flash targets
**
*****************************************************************************
*/
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
    KEEP(*(.vectortable)) /* Startup code */
    . = ALIGN(4);
  } > flash_interrupts
  
  .cfmprotect :
  {
    . = ALIGN(4);
	KEEP(*(.cfmconfig))	/* Flash Configuration Field (FCF) */
	. = ALIGN(4);
  } > flash_cfmprotrom

  /* The program code and other data goes into Flash */
  .text :
  {
    . = ALIGN(4);
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
    _etext = .;        /* define a global symbols at end of code */
  } > flash_text

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > flash_text
  .ARM : {
    __exidx_start = .;
      *(.ARM.exidx*)
      __exidx_end = .;
  } > flash_text
  
  .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE (*crtend.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > flash_text  
  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > flash_text  

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > flash_text
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > flash_text
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
	
	___ROM_AT = .;
  } > flash_text

  /* reserve MTB memory at the beginning of sram_data */
  .mtb : /* MTB buffer address as defined by the hardware */
  {
    . = ALIGN(8);
    _mtb_start = .;
    KEEP(*(.mtb_buf)) /* need to KEEP Micro Trace Buffer as not referenced by application */
    . = ALIGN(8);
    _mtb_end = .;
  } > sram_data  

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : AT(___ROM_AT)
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } > sram_data
  
  ___data_size = _edata - _sdata;
  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    __start_bss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    __end_bss = .;
  } > sram_data

  _romp_at = ___ROM_AT + SIZEOF(.data);
  .romp : AT(_romp_at)
  {
	__S_romp = _romp_at;
    LONG(___ROM_AT);
    LONG(_sdata);
    LONG(___data_size);
    LONG(0);
    LONG(0);
    LONG(0);
  } > sram_data
 
  .reset_stack :
  {
        . = ALIGN(8);
	KEEP(*(.resetstack))	/* reset stack */
  } > sram_data
  
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
