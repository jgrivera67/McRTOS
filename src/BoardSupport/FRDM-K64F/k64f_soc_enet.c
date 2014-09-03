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

/**
 * Maximum Ethernet frame size (in bytes) including CRC
 */
#define ENET_MAX_FRAME_SIZE	    1518

/**
 * Maximum Ethernet VLAN frame size (in bytes)
 */
#define ENET_MAX_FRAME_VLAN_SIZE    1522

/**
 * Maximum Ethernet frame size rounded-up to the required alignment
 */
#define ENET_ALIGNED_MAX_FRAME_SIZE \
	ROUND_UP(ENET_MAX_FRAME_SIZE, ENET_FRAME_BUFFER_ALIGNMENT)

C_ASSERT(ENET_ALIGNED_MAX_FRAME_SIZE >= 256 &&
	 (ENET_ALIGNED_MAX_FRAME_SIZE & ~ENET_MRBR_R_BUF_SIZE_MASK) == 0);

/**
 * Maximum Ethernet frame data size
 */
#define ENET_MAX_FRAME_DATA_SIZE    1500

static struct enet_device_var g_enet_var = {
    .initialized = false,
};

static uint8_t g_enet_rx_data_buffers[ENET_MAX_RX_FRAME_BUFFERS][ENET_ALIGNED_MAX_FRAME_SIZE]
__attribute__ ((aligned(ENET_FRAME_BUFFER_ALIGNMENT)));

static uint8_t g_enet_tx_data_buffers[ENET_MAX_TX_FRAME_BUFFERS][ENET_ALIGNED_MAX_FRAME_SIZE]
__attribute__ ((aligned(ENET_FRAME_BUFFER_ALIGNMENT)));

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
    .mac_address = {
	/* MAC address in big-endian byte order */
	[0] = 0x01,
	[1] = 0x00,
	[2] = 0x35,
	[3] = 0x52,
	[4] = 0xCF,
	[5] = 0x00
    }
};

static void
enet_buffer_descriptor_rings_init(const struct enet_device *enet_device_p)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    struct enet_device_var *enet_var_p = enet_device_p->var_p;

    /*
     * Configure Rx buffer descriptor ring:
     * - Set Rx descriptor ring start address
     * - Set max receive buffer size in bytes
     * - Initialize Rx buffer descriptors
     */
    FDC_ASSERT(((uintptr_t)enet_var_p->rx_buffer_descriptors &
		~ENET_RDSR_R_DES_START_MASK) == 0,
	           enet_var_p->rx_buffer_descriptors, enet_device_p);

    write_32bit_mmio_register(&enet_regs_p->RDSR,
			      (uintptr_t)enet_var_p->rx_buffer_descriptors);
    write_32bit_mmio_register(&enet_regs_p->MRBR, ENET_ALIGNED_MAX_FRAME_SIZE);

    for (unsigned int i = 0; i < ENET_MAX_RX_FRAME_BUFFERS; i ++) {
	struct enet_rx_buffer_descriptor *buffer_desc_p =
	    &enet_var_p->rx_buffer_descriptors[i];

	buffer_desc_p->data_buffer = g_enet_rx_data_buffers[i];
	buffer_desc_p->data_length = 0;
	/*
	 * Set the wrap flag for the last buffer of the ring:
	 */
	if (i != ENET_MAX_RX_FRAME_BUFFERS - 1) {
	    buffer_desc_p->control = 0;
	} else {
	    buffer_desc_p->control = ENET_RX_BD_WRAP_MASK;
	}

	buffer_desc_p->control |= ENET_RX_BD_EMPTY_MASK;

        /*
	 * Enable generation of receive interrupts
	 */
        buffer_desc_p->control_extend1 = ENET_RX_BD_GENERATE_INTERRUPT_MASK;
    }

    /*
     * Configure Tx buffer descriptor ring:
     * - Set Tx descriptor ring start address
     * - Initialize Tx buffer descriptors
     */
    FDC_ASSERT(((uintptr_t)enet_var_p->tx_buffer_descriptors &
		~ENET_TDSR_X_DES_START_MASK) == 0,
	           enet_var_p->tx_buffer_descriptors, enet_device_p);

    write_32bit_mmio_register(&enet_regs_p->TDSR,
			      (uintptr_t)enet_var_p->tx_buffer_descriptors);

    for (unsigned int i = 0; i < ENET_MAX_TX_FRAME_BUFFERS; i ++) {
	struct enet_tx_buffer_descriptor *buffer_desc_p =
	    &enet_var_p->tx_buffer_descriptors[i];

	buffer_desc_p->data_buffer = g_enet_tx_data_buffers[i];
	buffer_desc_p->data_length = 0;
	buffer_desc_p->control = 0;

	/*
	 * Set the wrap flag for the last buffer of the ring:
	 */
	if (i == ENET_MAX_TX_FRAME_BUFFERS - 1) {
	    buffer_desc_p->control |= ENET_TX_BD_WRAP_MASK;
	}
    }
}

/**
 * Initializes the Ethernet MAC (SoC ENET peripheral)
 */
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

    /*
     * Configure MAC registers to default reset values:
     *
     * TODO: This is redundant, as we reset the MAC above.
     * This could be replaced to FDC_ASSERTs
     */

    /*
     * - Enable normal operating mode (disable sleep mode)
     * - Enable enhanced frame time-stamping functions
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->ECR);
    reg_value &= ~ENET_ECR_SLEEP_MASK;
    reg_value |= ENET_ECR_EN1588_MASK;
    write_32bit_mmio_register(&enet_regs_p->ECR, reg_value);

    /*
     * - Enable stripping of CRC field for incoming frames
     * - Configure RMII interface to the Ethernet PHY
     * - Enable 100Mbps operation
     * - Disable internal loopback
     * - Set max incoming frame length (including CRC)
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->RCR);
    reg_value |= ENET_RCR_CRCFWD_MASK;
    reg_value |= ENET_RCR_MII_MODE_MASK | ENET_RCR_RMII_MODE_MASK;
    reg_value &= ~ENET_RCR_RMII_10T_MASK;
    reg_value &= ~ENET_RCR_LOOP_MASK;
    SET_BIT_FIELD(reg_value, ENET_RCR_MAX_FL_MASK, ENET_RCR_MAX_FL_SHIFT,
		  1518);
    write_32bit_mmio_register(&enet_regs_p->RCR, reg_value);

    /*
     * Enable duplex mode:
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->TCR);
    reg_value |= ENET_TCR_FDEN_MASK;
    write_32bit_mmio_register(&enet_regs_p->TCR, reg_value);

    /*
     * Set receive frame truncate length:
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->FTRL);
    SET_BIT_FIELD(reg_value, ENET_FTRL_TRUNC_FL_MASK, ENET_FTRL_TRUNC_FL_SHIFT,
		  2047);
    write_32bit_mmio_register(&enet_regs_p->FTRL, reg_value);

    /*
     * Set the transmit inter-packet gap
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->TIPG);
    SET_BIT_FIELD(reg_value, ENET_TIPG_IPG_MASK, ENET_TIPG_IPG_SHIFT,
		  12);
    write_32bit_mmio_register(&enet_regs_p->TIPG, reg_value);

    /*
     * Set pause duration to 0:
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->OPD);
    SET_BIT_FIELD(reg_value, ENET_OPD_PAUSE_DUR_MASK, ENET_OPD_PAUSE_DUR_SHIFT,
		  0);
    write_32bit_mmio_register(&enet_regs_p->OPD, reg_value);

#if 0  /* TODO: Enable this after initial testing */
    /*
     * Set Tx accelerators:
     */
    reg_value =	ENET_TACC_PROCHK_MASK |
		ENET_TACC_IPCHK_MASK |
		ENET_TACC_SHIFT16_MASK;
    write_32bit_mmio_register(&enet_regs_p->TACC, reg_value);

    /*
     * Set Rx accelerators:
     */
    reg_value =	ENET_RACC_PADREM_MASK |
		ENET_RACC_IPDIS_MASK |
		ENET_RACC_PRODIS_MASK |
		ENET_RACC_LINEDIS_MASK |
		ENET_RACC_SHIFT16_MASK;
    write_32bit_mmio_register(&enet_regs_p->RACC, reg_value);
#endif

    /**
     * Initialize the Serial Management Interface (SMI) between the Ethernet MAC
     * and the Ethernet PHY chip. The SMI is also known as the MII Management
     * interface (MIIM) and consists of two pins: MDC and MDIO.
     *
     * Settings:
     * - HOLDTIME:0 (Hold time on MDIO output: one internal module clock cycle)
     * - DIS_PRE: 0 (Preamble enabled)
     * - MII_SPEED: 24 (MDC clock freq: (1 / ((MII_SPEED + 1) * 2) = 1/50 of the
     *                  internal module clock frequency, which is 50MHz)
     */
    reg_value = 0;
    SET_BIT_FIELD(reg_value, ENET_MSCR_MII_SPEED_MASK, ENET_MSCR_MII_SPEED_SHIFT,
		  SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ/ 5);
    write_32bit_mmio_register(&enet_regs_p->MSCR, reg_value);

    /*
     * Program the MAC address:
     */
    reg_value = enet_device_p->mac_address[5] << 24 |
		enet_device_p->mac_address[4] << 16 |
		enet_device_p->mac_address[3] << 8 |
		enet_device_p->mac_address[2];
    write_32bit_mmio_register(&enet_regs_p->PALR, reg_value);
    reg_value = enet_device_p->mac_address[1] << 24 |
		enet_device_p->mac_address[0] << 16;
    write_32bit_mmio_register(&enet_regs_p->PAUR, reg_value);

    /*
     * Configure Tx FIFO:
     * - Set store and forward mode
     * - Set Tx FIFO section empty threshold to 0 (reset default)
     * - Set Tx FIFO almost empty threshold to 4 (reset default)
     * - Set Tx FIFO almost full threshold to 8 (reset default)
     */
    write_32bit_mmio_register(&enet_regs_p->TFWR, ENET_TFWR_STRFWD_MASK);
    write_32bit_mmio_register(&enet_regs_p->TSEM, 0);
    write_32bit_mmio_register(&enet_regs_p->TAEM, 4);
    write_32bit_mmio_register(&enet_regs_p->TAFL, 8);

    /*
     * Configure Rx FIFO:
     * - Set Rx FIFO section full threshold to 0 (reset default)
     * - Set Rx FIFO section empty threshold to 0 (reset default)
     * - Set Rx FIFO almost empty threshold to 4 (reset default)
     * - Set Rx FIFO almost full threshold to 4 (reset default)
     */
    write_32bit_mmio_register(&enet_regs_p->RSFL, 0);
    write_32bit_mmio_register(&enet_regs_p->RSEM, 0);
    write_32bit_mmio_register(&enet_regs_p->RAEM, 4);
    write_32bit_mmio_register(&enet_regs_p->RAFL, 4);

    enet_buffer_descriptor_rings_init(enet_device_p);

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

    ethernet_phy_init(enet_device_p);

    enet_var_p->initialized = true;
}

