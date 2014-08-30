/**
 * @file k64f_soc_enet.c
 *
 * Ethernet MAC (ENET) device driver for the K64F SoC
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "k64f_soc.h"
#include "frdm_board.h"
#include "McRTOS_arm_cortex_m.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"

static struct enet_device_var g_enet_var = {
    .initialized = false,
};

const struct enet_device g_enet_device = {
    .signature = ENET_DEVICE_SIGNATURE,
    .var_p = &g_enet_var,
    .mmio_registers_p = (volatile ENET_Type *)ENET_BASE,
    .rmii_mdio_pin = PIN_INITIALIZER(PIN_PORT_B, 0, PIN_FUNCTION_ALT4),
    .rmii_mdc_pin = PIN_INITIALIZER(PIN_PORT_B, 1, PIN_FUNCTION_ALT4),
    .rmii_rxd0_pin = PIN_INITIALIZER(PIN_PORT_A, 13, PIN_FUNCTION_ALT4),
    .rmii_rxd1_pin = PIN_INITIALIZER(PIN_PORT_A, 12, PIN_FUNCTION_ALT4),
    .rmii_crs_dv_pin = PIN_INITIALIZER(PIN_PORT_A, 14, PIN_FUNCTION_ALT4),
    .rmii_rxer_pin = PIN_INITIALIZER(PIN_PORT_A, 5, PIN_FUNCTION_ALT4),
    .rmii_txen_pin = PIN_INITIALIZER(PIN_PORT_A, 15, PIN_FUNCTION_ALT4),
    .rmii_txd0_pin = PIN_INITIALIZER(PIN_PORT_A, 16, PIN_FUNCTION_ALT4),
    .rmii_txd1_pin = PIN_INITIALIZER(PIN_PORT_A, 17, PIN_FUNCTION_ALT4),
    .mii_txer_pin = PIN_INITIALIZER(PIN_PORT_A, 28, PIN_FUNCTION_ALT4),
    .enet_1588_tmr_pins = {
	[0] = PIN_INITIALIZER(PIN_PORT_C, 16, PIN_FUNCTION_ALT4),
	[1] = PIN_INITIALIZER(PIN_PORT_C, 17, PIN_FUNCTION_ALT4),
	[2] = PIN_INITIALIZER(PIN_PORT_C, 18, PIN_FUNCTION_ALT4),
	[3] = PIN_INITIALIZER(PIN_PORT_C, 19, PIN_FUNCTION_ALT4),
    },

    .clock_gate_mask = SIM_SCGC2_ENET_MASK,
};

/**
 * MAC address (in big-endian) for the Ethernet interface
 */
static const uint8_t g_mac_address[] = {
    [0] = 0x00,
    [1] = 0xCF,
    [2] = 0x52,
    [3] = 0x35,
    [4] = 0x00,
    [5] = 0x01
};

static void
ethernet_mac_init(const struct enet_device *enet_device_p)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    uint32_t reg_value;
    uint16_t polling_count = UINT16_MAX;

    /*
     * Reset Ethernet MAC module:
     */
    write_32bit_mmio_register(&enet_regs_p->ECR, ENET_ECR_RESET_MASK);

    /*
     * Wait for reset to complete:
     */
    do {
	reg_value = read_32bit_mmio_register(&enet_regs_p->ECR);
	polling_count --;
    } while ((reg_value & ENET_ECR_RESET_MASK) != 0 && polling_count != 0);

    if (reg_value & ENET_ECR_RESET_MASK) {
	fdc_error_t fdc_error =
            CAPTURE_FDC_ERROR("Enet reset failed", enet_device_p, reg_value);

        fatal_error_handler(fdc_error);
    }

    /*
     * Disable generation of interrupts:
     */
    write_32bit_mmio_register(&enet_regs_p->EIMR, 0x0);

    /*
     * Clear pending interrupts:
     */
    write_32bit_mmio_register(&enet_regs_p->EIR, MULTI_BIT_MASK(30, 0));

    /*
     * Clear multicast group and individual hash registers
     */
    write_32bit_mmio_register(&enet_regs_p->GALR, 0x0);
    write_32bit_mmio_register(&enet_regs_p->GAUR, 0x0);
    write_32bit_mmio_register(&enet_regs_p->IALR, 0x0);
    write_32bit_mmio_register(&enet_regs_p->IAUR, 0x0);
}

/**
 * Initializes the Ethernet MAC module of the K64F SoC
 */
void
enet_init(const struct enet_device *enet_device_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    FDC_ASSERT(!enet_var_p->initialized, enet_device_p, enet_var_p);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Enable the Clock to the ENET Module
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC2);
    reg_value |= enet_device_p->clock_gate_mask;
    write_32bit_mmio_register(&SIM_SCGC2, reg_value);

    /*
     * Configure GPIO pins for Ethernet PHY functions:
     * - Set "open drain enabled", "pull-up resistor enabled" and
     *   "internal pull resistor enabled" for rmii_mdio pin
     *
     * NOTE: No external pullup is available on MDIO signal when the K64F SoC
     * requests status of the Ethernet link connection. Internal pullup
     * is required when port configuration for MDIO signal is enabled.
     */
    set_pin_function(&enet_device_p->rmii_mdio_pin,
		     PORT_PCR_ODE_MASK |
		     PORT_PCR_PE_MASK |
		     PORT_PCR_PS_MASK);

    set_pin_function(&enet_device_p->rmii_mdc_pin, 0);
    set_pin_function(&enet_device_p->rmii_rxd0_pin, 0);
    set_pin_function(&enet_device_p->rmii_rxd1_pin, 0);
    set_pin_function(&enet_device_p->rmii_rxer_pin, 0);
    set_pin_function(&enet_device_p->rmii_txen_pin, 0);
    set_pin_function(&enet_device_p->rmii_txd0_pin, 0);
    set_pin_function(&enet_device_p->rmii_txd1_pin, 0);
    set_pin_function(&enet_device_p->mii_txer_pin, 0);

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(enet_device_p->enet_1588_tmr_pins);
	 ++ i) {
	set_pin_function(&enet_device_p->enet_1588_tmr_pins[i], 0);
    }

    ethernet_mac_init(enet_device_p);

    //??? Init ENET device

    //??? ethernet_phy_init(enet_device_p);

    enet_var_p->initialized = true;
}

