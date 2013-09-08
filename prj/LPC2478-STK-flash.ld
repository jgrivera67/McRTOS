/*ENTRY(ResetHandler)*/
ENTRY(_startup)
SEARCH_DIR(.)

/*
 * Define stack size here
 */
FIQ_STACK_SIZE = 0x0040;
IRQ_STACK_SIZE = 0x0100;
ABT_STACK_SIZE = 0x0040;
UND_STACK_SIZE = 0x0040;
SVC_STACK_SIZE = 0x0100;
SYS_STACK_SIZE = 0x0040;

/*
 * This file, lpc2478_flash.ld, locates the program in the internal
 * flash of the LPC2478. For more information about the memory of the LPC2478
 * take a look in the LPC2478 user manual
 * Reference is made to the user manual from 26 August 2009 Rev. 04
 *
 * From Chapter 2, Section 1. Memory maps
 */
 
MEMORY
{
	flash     		: ORIGIN = 0,          LENGTH = 0x7E000	/* 516096 FLASH ROM */	
	flash_isp  		: ORIGIN = 0x0007E000, LENGTH = 0x2000	/* 8192 boot block */	
	
	ram_isp_lo(A)   : ORIGIN = 0x40000040, LENGTH = 0x1C0   /* 448 bytes for 
															   variables used
															   by ISP bootloader
															   and RealMonitor 
															   DMI need to verifiy origin */
	ram   			: ORIGIN = 0x40000200, LENGTH = 0xFCE0  /* 64736 free RAM area */
	ram_isp_stack   : ORIGIN = 0x4000FEE0, LENGTH = 0x100   /* 256 bytes used by ISP */
	ram_isp_hi(A)   : ORIGIN = 0x4000FFE0, LENGTH = 0x20    /* 32 bytes used by ISP */
}

/* define the bootloader entry address.  This is the first free address in RAM */

boot_entry = 0x40000200;

/* define a global symbol _stack_end a.k.a. first stack entry. stack grows down  */

/*_stack_end = 0x40007EDC;	*/			/* 0x40007FFF-0x40007EDC=0x123=291d */
PROVIDE (__ram_end = 0x40007FFF);   


/*
 * Do not change the next code
 */
SECTIONS
{
	. = 0;								/* set location counter to address zero  */

	startup : {
	*(.startup)
    . = ALIGN(4);
    *(.vectors);
    . = ALIGN(4);
    *(.init);
    . = ALIGN(4);
	} >flash		/* the startup code goes into FLASH */

  .text :
  {
    *(.text);
    . = ALIGN(4);
    *(.rodata);
    . = ALIGN(4);
    *(.rodata*);
    . = ALIGN(4);
    *(.glue_7t);
    . = ALIGN(4);
    *(.glue_7);
    . = ALIGN(4);
    etext = .;
    _etext = .;
  } >flash

  .data :
  {
    PROVIDE (__data_start = .);
    *(.data)
    . = ALIGN(4);
    edata = .;
    _edata = .;
    PROVIDE (__data_end = .);
  } >ram AT >flash

  .bss :
  {
    PROVIDE (__bss_start = .);
    *(.bss)
    *(COMMON)
    . = ALIGN(4);
    PROVIDE (__bss_end = .);
    
    . = ALIGN(256);
    
    PROVIDE (__stack_start = .);
    
    PROVIDE (__stack_fiq_start = .);
    . += FIQ_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_fiq_end = .);

    PROVIDE (__stack_irq_start = .);
    . += IRQ_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_irq_end = .);

    PROVIDE (__stack_abt_start = .);
    . += ABT_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_abt_end = .);

    PROVIDE (__stack_und_start = .);
    . += UND_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_und_end = .);

    PROVIDE (__stack_svc_start = .);
    . += SVC_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_svc_end = .);

    PROVIDE (__stack_sys_start = .);
    . += SYS_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_sys_end = .);

    PROVIDE (__stack_end = .);
    PROVIDE (__heap_start = .);   

    . = ALIGN(16);
    PROVIDE (__free_ram_start = .);   
  } > ram
}
	_end = .;							/* define a global symbol marking the end of application RAM */
  PROVIDE ( end = . );
/*** EOF ***/

