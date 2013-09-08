#ifndef __GPIO_H
#define __GPIO_H

#include "utils.h"
#include "compile_time_checks.h"

/*******************************************************************************
 * GPIO Registers (legacy APB accessible registers)
 */

#define GPIO_BASE_ADDR  (0xE0028000)
#define IO0PIN          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x00)) /* RW */
#define IO0SET          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x04)) /* RW */
#define IO0DIR          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x08)) /* RW */
#define IO0CLR          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x0C)) /* WO */

#define IO1PIN          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x10)) /* RW */
#define IO1SET          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x14)) /* RW */
#define IO1DIR          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x18)) /* RW */
#define IO1CLR          (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x1C)) /* WO */


/*******************************************************************************
 * GPIO Interrupt Registers
 */

/* Wierd note: these are the only registers with upper and lower case */
/* IOxIntClr registers are write 1 to clear*/

#define IOIntStatus     (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x80)) /* RO */
#define IOIntStatusMask (0x00000005) /* 0b0...0101 */

#define IO0IntStatR     (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x84)) /* RO */
#define IO0IntStatF     (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x88)) /* RO */
#define IO0IntClr       (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x8C)) /* WO */
#define IO0IntEnR       (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x90)) /* RW */
#define IO0IntEnF       (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0x94)) /* RW */

#define IO2IntStatR     (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0xA4)) /* RO */
#define IO2IntStatF     (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0xA8)) /* RO */
#define IO2IntClr       (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0xAC)) /* WO */
#define IO2IntEnR       (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0xB0)) /* RW */
#define IO2IntEnF       (*(volatile uint32_t *)(GPIO_BASE_ADDR + 0xB4)) /* RW */


/*******************************************************************************
 * Fast GPIO Registers (local bus accessible registers - enhanced GPIO features)
 */

/* FIOxMASK Registers: NOTE 0 enables bit, 1 disables bit access */
/* FIO Clear registers are write 1 to clear which is probably why they are WO */

#define FIO_BASE_ADDR       UINT32_C(0x3FFFC000)

/**
 * grivera: Register space size for a single GPIO port
 */
#define FAST_GPIO_PORT_SIZE UINT32_C(0x20)

/**
 * grivera: Enumerated type for GPIO port names
 */
enum gpio_port_names
{
    GPIO_PORT_P0 = 0,
    GPIO_PORT_P1,
    GPIO_PORT_P2,
    GPIO_PORT_P3,
    GPIO_PORT_P4,

    GPIO_NUM_PORTS
};

/**
 * Memory-mapped I/O registers of an individual fast GPIO port in the LPC2478.
 * Each GPIO port has 32 pins.
 */
struct fast_gpio_port {
    /**
     * Port Direction control register. This register
     * individually controls the direction of each port pin.
     */ 
    union {
        /**  
         * For all i: 0..31, the value of bit i of reg_FIODIR represents the
         * following:
         * - 0: pin i configured as an input pin.
         * - 1: pin i configured as an output pin.
         */  
        uint32_t reg_FIODIR;

        /**  
         * For all i: 0..7, j: 0..3:
         *      Bit i of reg_byte_FIODIR[j] corresponds to pin "i + 8*j":
         */  
        uint8_t reg_byte_FIODIR[4];

        /**  
         * For all i: 0..15, j: 0..1:
         *      Bit i of reg_half_word_FIODIR[j] corresponds to pin "i + 16*j":
         */  
        uint16_t reg_half_word_FIODIR[2];
    };

    /**
     * 12 bytes of Padding
     */
    uint8_t reg_padding1[0xC];

    /**
     * Mask register for port. Writes, sets, clears, and reads to
     * port (done via writes to FIOPIN, FIOSET, and FIOCLR, and
     * reads of FIOPIN) alter or return only the bits enabled by zeros
     * in this register.
     */
    union {
        /**
         * For all i: 0..31, the value of bit i of reg_FIOMASK represents the
         * following:
         * - 0: pin i enabled
         * - 1: pin i disabled
         */   
        uint32_t reg_FIOMASK;

        /**  
         * For all i: 0..7, j: 0..3:
         *      Bit i of reg_byte_FIOMASK[j] corresponds to pin "i + 8*j":
         */  
        uint8_t reg_byte_FIOMASK[4];

        /**  
         * For all i: 0..15, j: 0..1:
         *      Bit i of reg_half_word_FIOMASK[j] corresponds to pin "i + 16*j":
         */  
        uint16_t reg_half_word_FIOMASK[2];
    };

    /**
     * Port Pin value register using FIOMASK. The current state
     * of digital port pins can be read from this register, regardless of
     * pin direction or alternate function selection (as long as pins are
     * not configured as an input to ADC). The value read is masked
     * by ANDing with inverted FIOMASK. Writing to this register
     * places corresponding values in all bits enabled by zeros in
     * FIOMASK.
     * Important: if a FIOPIN register is read, its bit(s) masked with 1
     * in the FIOMASK register will be set to 0 regardless of the
     * physical pin state
     */ 
    union {
        /**
         * For all i: 0..31, the value of bit i of reg_FIOPIN represents the
         * following:
         * - 0: pin i is low
         * - 1: pin i is high
         */
        uint32_t reg_FIOPIN;

        /**  
         * For all i: 0..7, j: 0..3:
         *      Bit i of reg_byte_FIOPIN[j] corresponds to pin "i + 8*j":
         */  
        uint8_t reg_byte_FIOPIN[4];

        /**  
         * For all i: 0..15, j: 0..1:
         *      Bit i of reg_half_word_FIOPIN[j] corresponds to pin "i + 16*j":
         */  
        uint16_t reg_half_word_FIOPIN[2];
    };

    /**
     * Port Output Set register using FIOMASK. This register
     * controls the state of output pins. Writing 1s produces highs at
     * the corresponding port pins. Writing 0s has no effect. Reading
     * this register returns the current contents of the port output
     * register. Only bits enabled by 0 in FIOMASK can be altered.
     */ 
    union {
        /** 
         * For all i: 0..31, the value of bit i of reg_FIOSET represents the
         * following:
         * - 0: nop
         * - 1: set pin i to high
         */   
        uint32_t reg_FIOSET;

        /**  
         * For all i: 0..7, j: 0..3:
         *      Bit i of reg_byte_FIOSET[j] corresponds to pin "i + 8*j":
         */  
        uint8_t reg_byte_FIOSET[4];

        /**  
         * For all i: 0..15, j: 0..1:
         *      Bit i of reg_half_word_FIOSET[j] corresponds to pin "i + 16*j":
         */  
        uint16_t reg_half_word_FIOSET[2];
    };

    /**
     * Port Output Clear register using FIOMASK0. This register
     * controls the state of output pins. Writing 1s produces lows at
     * the corresponding port pins. Writing 0s has no effect. Only bits
     * enabled by 0 in FIOMASK0 can be altered. This is a write-only register.
     */ 
    union {
        /**
         * For all i: 0..31, the value of bit i of reg_FIOCLR represents the
         * following:
         * - 0: nop
         * - 1: set pin i to low
         */   
        uint32_t reg_FIOCLR;

        /**  
         * For all i: 0..7, j: 0..3:
         *      Bit i of reg_byte_FIOCLR[j] corresponds to pin "i + 8*j":
         */  
        uint8_t reg_byte_FIOCLR[4];

        /**  
         * For all i: 0..15, j: 0..1:
         *      Bit i of reg_half_word_FIOCLR[j] corresponds to pin "i + 16*j":
         */  
        uint16_t reg_half_word_FIOCLR[2];
    };
};

/*
 * Compile-time asserts to verify that the compiler generates the expected offsets
 * for structs that represent groups of memory-mapped I/O registers
 */

C_ASSERT(sizeof(struct fast_gpio_port) == FAST_GPIO_PORT_SIZE);

C_ASSERT(offsetof(struct fast_gpio_port, reg_FIODIR) == 0x00);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIODIR[0]) == 0x00);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIODIR[1]) == 0x01);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIODIR[2]) == 0x02);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIODIR[3]) == 0x03);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIODIR[0]) == 0x00);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIODIR[1]) == 0x02);

C_ASSERT(offsetof(struct fast_gpio_port, reg_FIOMASK) == 0x10);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOMASK[0]) == 0x10);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOMASK[1]) == 0x11);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOMASK[2]) == 0x12);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOMASK[3]) == 0x13);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOMASK[0]) == 0x10);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOMASK[1]) == 0x12);

C_ASSERT(offsetof(struct fast_gpio_port, reg_FIOPIN) == 0x14);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOPIN[0]) == 0x14);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOPIN[1]) == 0x15);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOPIN[2]) == 0x16);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOPIN[3]) == 0x17);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOPIN[0]) == 0x14);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOPIN[1]) == 0x16);

C_ASSERT(offsetof(struct fast_gpio_port, reg_FIOSET) == 0x18);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOSET[0]) == 0x18);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOSET[1]) == 0x19);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOSET[2]) == 0x1A);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOSET[3]) == 0x1B);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOSET[0]) == 0x18);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOSET[1]) == 0x1A);

C_ASSERT(offsetof(struct fast_gpio_port, reg_FIOCLR) == 0x1C);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOCLR[0]) == 0x1C);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOCLR[1]) == 0x1D);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOCLR[2]) == 0x1E);
C_ASSERT(offsetof(struct fast_gpio_port, reg_byte_FIOCLR[3]) == 0x1F);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOCLR[0]) == 0x1C);
C_ASSERT(offsetof(struct fast_gpio_port, reg_half_word_FIOCLR[1]) == 0x1E);

/**
 * Base address for the GPIO interrupt registers
 */
#define GPIO_INTERRUPT_REGISTERS_BASE_ADDR  UINT32_C(0xE0028080)

/**
 * Register space size for the GPIO interrupt registers
 */
#define GPIO_INTERRUPT_REGISTERS_SIZE       UINT32_C(0x44)

/*
 * Per GPIO port interrupt registers for ports GPIO_PORT_P0 and
 * GPIO_PORT_P2
 */
struct gpio_port_interrupt_registers 
{
    /**
     * GPIO Interrupt Status for Rising edge (read-only register).
     * If bit i is 1, there is a pending interrupt for detected
     * rising edge on pin i of the GPIO port.
     * (Bit 0 to corresponds to pin 0, bit 31 corresponds to pin 31)
     */
    uint32_t reg_IntStatR;

    /**
     * GPIO Interrupt Status for Falling edge (read-only register).
     * If bit i is 1, there is a pending interrupt for detected
     * falling edge on pin i of the GPIO port.
     * (Bit 0 to corresponds to pin 0, bit 31 corresponds to pin 31)
     */
    uint32_t reg_IntStatF;

    /**
     * GPIO Interrupt Clear (write-only register).
     * Writing a 1 into a bit in this register clears any
     * interrupts for the corresponding GPIO port pin.
     * (Bit 0 to corresponds to pin 0, bit 31 corresponds to pin 31)
     */
    uint32_t reg_IntClr;

    /**
     * GPIO Interrupt Enable for Rising edge.
     * Writing a 1 into a bit in this register enables the rising edge
     * interrupt for the corresponding GPIO port pin. Writing a 0
     * disables the interrupt for the pin.
     * (Bit 0 to corresponds to pin 0, bit 31 corresponds to pin 31)
     */
    uint32_t reg_IntEnR;

    /**
     * GPIO Interrupt Enable for Falling edge.
     * Writing a 1 into a bit in this register enables the falling edge
     * interrupt for the corresponding GPIO port pin. Writing a 0
     * disables the interrupt for the pin.
     * (Bit 0 to corresponds to pin 0, bit 31 corresponds to pin 31)
     */
    uint32_t reg_IntEnF;

    uint32_t padding[3];

};

/**
 * Memory-mapped I/O registers for interrupts for a GPIO ports that 
 * support interrupts in the LPC2478 (P0 and P2).
 */
struct gpio_interrupt_registers
{
    /**
     * GPIO overall Interrupt Status (read-only register).
     */
    uint32_t reg_IntStatus;

    /**
     * If this bit is 1, there is at least one pending interrupt for
     * port GPIO port P0
     */ 
#   define  GPIO_INT_STATUS_P0_INT_MASK	    BIT(0)
#   define  GPIO_INT_STATUS_RESERVED1_MASK  BIT(1)    

    /**
     * If this bit is 1, there is at least one pending interrupt for
     * port GPIO port P2
     */ 
#   define  GPIO_INT_STATUS_P2_INT_MASK	    BIT(2)

    /*
     * Reserved
     */
#   define  GPIO_INT_STATUS_RESERVED2_MASK  MULTI_BIT_MASK(31, 3)
#   define  GPIO_INT_STATUS_RESERVED2_SHIFT 3

    /*
     * Per GPIO port interrupt registers for port GPIO_PORT_P0
     */
    struct gpio_port_interrupt_registers gpio_port_p0;

    /*
     * Per GPIO port interrupt registers for port GPIO_PORT_P2
     */
    struct gpio_port_interrupt_registers gpio_port_p2;

};

/*
 * Compile-time asserts to verify that the compiler generates the expected offsets
 * for structs that represent groups of memory-mapped I/O registers
 */

C_ASSERT(sizeof(struct gpio_interrupt_registers) == GPIO_INTERRUPT_REGISTERS_SIZE);

C_ASSERT(offsetof(struct gpio_interrupt_registers, reg_IntStatus) == 0x00);
C_ASSERT(offsetof(struct gpio_interrupt_registers, gpio_port_p0.reg_IntStatR) == 0x04);
C_ASSERT(offsetof(struct gpio_interrupt_registers, gpio_port_p0.reg_IntStatF) == 0x08);
C_ASSERT(offsetof(struct gpio_interrupt_registers, gpio_port_p0.reg_IntClr) == 0x0C);
C_ASSERT(offsetof(struct gpio_interrupt_registers, gpio_port_p2.reg_IntStatR) == 0x24);
C_ASSERT(offsetof(struct gpio_interrupt_registers, gpio_port_p2.reg_IntStatF) == 0x28);
C_ASSERT(offsetof(struct gpio_interrupt_registers, gpio_port_p2.reg_IntClr) == 0x2C);

#endif /* __GPIO_H */
