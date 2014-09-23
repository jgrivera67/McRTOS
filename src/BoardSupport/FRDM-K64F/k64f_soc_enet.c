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
#include <Networking/networking.h>

C_ASSERT(NETWORK_MTU == 1500);

/**
 * Maximum Ethernet frame size (in bytes) including CRC
 */
#define ENET_MAX_FRAME_SIZE	    (NETWORK_MTU + 18)

/**
 * Maximum Ethernet VLAN frame size (in bytes)
 */
#define ENET_MAX_FRAME_VLAN_SIZE    (NETWORK_MTU + 22)

/**
 * Maximum Ethernet frame size + 4-byte signature prefix, rounded-up to
 * the required alignment
 */
#define ENET_ALIGNED_MAX_FRAME_SIZE \
	ROUND_UP(ENET_MAX_FRAME_SIZE + sizeof(struct enet_buf_control_block), \
		 ENET_FRAME_BUFFER_ALIGNMENT)

struct enet_buf_control_block {
    uint32_t signature;
#   define ENET_TX_BUFFER_SIGNATURE  GEN_SIGNATURE('T', 'X', 'B', 'U')
#   define ENET_RX_BUFFER_SIGNATURE  GEN_SIGNATURE('R', 'X', 'B', 'U')

    bool in_transit;
    union {
	struct enet_tx_buffer_descriptor *tx_buf_desc_p;
	struct enet_rx_buffer_descriptor *rx_buf_desc_p;
    };
};

C_ASSERT(ENET_ALIGNED_MAX_FRAME_SIZE >= 256 &&
	 (ENET_ALIGNED_MAX_FRAME_SIZE & ~ENET_MRBR_R_BUF_SIZE_MASK) == 0);


/*
 * ENET PHY Registers
 */
enum enet_phy_registers
{
    ENET_PHY_CONTROL_REG =  0x0, /* basic control register */
    ENET_PHY_STATUS_REG =   0x1, /* basic status register */
    ENET_PHY_ID1_REG =	    0x2, /* identification register 1*/
    ENET_PHY_ID2_REG =	    0x3, /* identification register 2*/
    ENET_PHY_CONTROL1_REG = 0x1e, /* control register 1 */
    ENET_PHY_CONTROL2_REG = 0x1f, /* control register 2*/
};

/*
 * Bit masks for ENET_PHY_CONTROL_REG register flags
 */
#define ENET_PHY_RESET_MASK		BIT(15)
#define ENET_PHY_LOOP_MASK		BIT(14)
#define ENET_PHY_100_MBPS_SPEED_MASK    BIT(13)
#define ENET_PHY_AUTO_NEGOTIATION_MASK  BIT(12)
#define ENET_PHY_POWER_DOWN_MASK	BIT(11)
#define ENET_PHY_ISOLATE_MASK		BIT(10)
#define ENET_PHY_RESTART_AUTO_NEG_MASK  BIT(9)
#define ENET_PHY_FULL_DUPLEX_MODE_MASK  BIT(8)
#define ENET_PHY_COLLISION_TEST_MASK    BIT(7)

/*
 * Bit masks for ENET_PHY_STATUS_REG register flags
 */
#define ENET_PHY_AUTO_NEG_COMPLETE_MASK	BIT(5)
#define ENET_PHY_AUTO_NEG_CAPABLE_MASK	BIT(3)
#define ENET_PHY_LINK_UP_MASK		BIT(2)

/**
 * Maximum number of iterations for a polling loop
 */
#define MAX_POLLING_COUNT   UINT16_MAX

/**
 * Ethernet Tx data buffers
 */
static uint8_t g_enet_tx_data_buffers[ENET_MAX_TX_FRAME_BUFFERS][ENET_ALIGNED_MAX_FRAME_SIZE]
__attribute__ ((aligned(ENET_FRAME_BUFFER_ALIGNMENT)));

/**
 * Ethernet Rx data buffers
 */
static uint8_t g_enet_rx_data_buffers[ENET_MAX_RX_FRAME_BUFFERS][ENET_ALIGNED_MAX_FRAME_SIZE]
__attribute__ ((aligned(ENET_FRAME_BUFFER_ALIGNMENT)));

/**
 * McRTOS interrupt objects for ENET Tx/Rx interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_enet_tx_p = NULL;
struct rtos_interrupt *g_rtos_interrupt_enet_rx_p = NULL;

/**
 * Global non-const structure for ENET MAC device
 * (allocated in SRAM space)
 */
static struct enet_device_var g_enet_var = {
    .initialized = false,
};

/**
 * Global const structure for the ENET MAC devices
 * (allocated in flash space)
 */
const struct enet_device g_enet_device0 = {
    .signature = ENET_DEVICE_SIGNATURE,
    .name = "enet0",
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
    .tx_rtos_interrupt_params = {
            .irp_name_p = "ENET Transmit Interrupt",
            .irp_isr_function_p = k64f_enet_transmit_isr,
            .irp_arg_p =  (void *)&g_enet_device,
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ENET_Transmit),
            .irp_priority = ENET_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

    .tx_rtos_interrupt_pp = &g_rtos_interrupt_enet_tx_p,

    .rx_rtos_interrupt_params = {
            .irp_name_p = "ENET Receive Interrupt",
            .irp_isr_function_p = k64f_enet_receive_isr,
            .irp_arg_p =  (void *)&g_enet_device,
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ENET_Receive),
            .irp_priority = ENET_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

    .rx_rtos_interrupt_pp = &g_rtos_interrupt_enet_rx_p,

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
enet_tx_buffer_descriptor_ring_init(const struct enet_device *enet_device_p)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    struct enet_device_var *enet_var_p = enet_device_p->var_p;

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
	struct enet_buf_control_block *buf_control_block_p =
	    (struct enet_buf_control_block *)g_enet_tx_data_buffers[i];

	buf_control_block_p->signature = ENET_TX_BUFFER_SIGNATURE;
	buf_control_block_p->in_transit = false;
	buf_control_block_p->tx_buf_desc_p = buffer_desc_p;
	buffer_desc_p->data_buffer = buf_control_block_p + 1;
	buffer_desc_p->data_length = 0;

	/*
	 * Tx buffers are large enough to always hold entire frames, so a
	 * frame is never fragmented into multiple buffers.
	 * Frames smaller than 60 bytes are automatically padded.
	 * The minimum Ethernet frame length transmitted on the wire
	 * is 64 bytes, including the CRC.
	 */
	buffer_desc_p->control = ENET_TX_BD_LAST_IN_FRAME_MASK |
				 ENET_TX_BD_CRC_MASK;

	/*
	 * Set the wrap flag for the last buffer of the ring:
	 */
	if (i == ENET_MAX_TX_FRAME_BUFFERS - 1) {
	    buffer_desc_p->control |= ENET_TX_BD_WRAP_MASK;
	}
    }
}


static void
enet_rx_buffer_descriptor_ring_init(const struct enet_device *enet_device_p)
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
	struct enet_buf_control_block *buf_control_block_p =
	    (struct enet_buf_control_block *)g_enet_rx_data_buffers[i];

	buf_control_block_p->signature = ENET_RX_BUFFER_SIGNATURE;
	buf_control_block_p->in_transit = true;
	buf_control_block_p->rx_buf_desc_p = buffer_desc_p;
	buffer_desc_p->data_buffer = buf_control_block_p + 1;
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
}


/**
 * Initializes the Ethernet MAC (SoC ENET peripheral)
 */
static void
ethernet_mac_init(const struct enet_device *enet_device_p)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    uint32_t reg_value;
    uint16_t polling_count = MAX_POLLING_COUNT;

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
     * - Enable normal operating mode (disable sleep mode)
     * - Enable enhanced frame time-stamping functions
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->ECR);
    reg_value &= ~ENET_ECR_SLEEP_MASK;
    reg_value |= ENET_ECR_EN1588_MASK;
    write_32bit_mmio_register(&enet_regs_p->ECR, reg_value);

    /*
     * Configure receive control register:
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
		  ENET_MAX_FRAME_SIZE);
    write_32bit_mmio_register(&enet_regs_p->RCR, reg_value);

    /*
     * Configure transmit control register:
     * - Automatically write the source MAC address (SA) to Ethernet frame
     *   header in the Tx buffer, using the address programmed in the PALR/PAUR
     *   registers
     * - Enable full duplex mode
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->TCR);
    reg_value |= ENET_TCR_ADDINS_MASK | ENET_TCR_FDEN_MASK;
    write_32bit_mmio_register(&enet_regs_p->TCR, reg_value);

    /*
     * Set receive frame truncate length (use reset value 0x7ff):
     *
     * TODO: Should this be changed to ENET_MAX_FRAME_SIZE?
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

    enet_tx_buffer_descriptor_ring_init(enet_device_p);
    enet_rx_buffer_descriptor_ring_init(enet_device_p);

    /*
     * Enable generation of Tx/Rx interrupts:
     * - Generate Tx interrupt when a frame has been transmitted (the
     *   last and only Tx buffer descriptor of the frame has been updated)
     * - Generate Rx interrupt when a frame has been received (the
     *   (last and only Rx buffer descriptor of the frame has been updated)
     */
    write_32bit_mmio_register(&enet_regs_p->EIMR,
			      ENET_EIMR_TXF_MASK |
			      ENET_EIMR_RXF_MASK);

    /*
     * Register McRTOS interrupt handlers
     */
    rtos_k_register_interrupt(
        &enet_device_p->tx_rtos_interrupt_params,
        enet_device_p->tx_rtos_interrupt_pp);

    DBG_ASSERT(
        *enet_device_p->tx_rtos_interrupt_pp != NULL,
        enet_device_p->tx_rtos_interrupt_pp, enet_device_p);

    rtos_k_register_interrupt(
        &enet_device_p->rx_rtos_interrupt_params,
        enet_device_p->rx_rtos_interrupt_pp);

    DBG_ASSERT(
        *enet_device_p->rx_rtos_interrupt_pp != NULL,
        enet_device_p->rx_rtos_interrupt_pp, enet_device_p);

    /*
     * Enable ENET module and enable buffer descriptor byte swapping
     * (since ARM Cortex-M is little-endian):
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->ECR);
    reg_value |= ENET_ECR_ETHEREN_MASK |
		 ENET_ECR_DBSWP_MASK;
    write_32bit_mmio_register(&enet_regs_p->ECR, reg_value);

    /*
     * Activate Rx buffer descriptor ring:
     * (the Rx descriptor ring must have at least one descriptor with the "empty"
     *  bit set in its control field)
     */
    write_32bit_mmio_register(&enet_regs_p->RDAR, ENET_RDAR_RDAR_MASK);
}


void
enet_phy_write(const struct enet_device *enet_device_p, uint32_t phy_reg, uint32_t data)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    uint32_t reg_value;
    uint16_t polling_count = MAX_POLLING_COUNT;

    reg_value = read_32bit_mmio_register(&enet_regs_p->MSCR);
    FDC_ASSERT((reg_value & ENET_MSCR_MII_SPEED_MASK) != 0,
	       reg_value, enet_device_p);

    reg_value = read_32bit_mmio_register(&enet_regs_p->EIR);
    FDC_ASSERT((reg_value & ENET_EIR_MII_MASK) == 0,
	       reg_value, enet_device_p);

    /*
     * Set write command
     */
    reg_value = 0;
    SET_BIT_FIELD(reg_value, ENET_MMFR_ST_MASK, ENET_MMFR_ST_SHIFT,
		  0x1);
    SET_BIT_FIELD(reg_value, ENET_MMFR_OP_MASK, ENET_MMFR_OP_SHIFT,
		  ENET_MMFR_OP_WRITE_VALID_MII_MANAGEMENT_FRAME);
    SET_BIT_FIELD(reg_value, ENET_MMFR_PA_MASK, ENET_MMFR_PA_SHIFT,
		  ENET_PHY_ADDRESS);
    SET_BIT_FIELD(reg_value, ENET_MMFR_RA_MASK, ENET_MMFR_RA_SHIFT,
		  phy_reg);
    SET_BIT_FIELD(reg_value, ENET_MMFR_TA_MASK, ENET_MMFR_TA_SHIFT,
		  0x2);
    SET_BIT_FIELD(reg_value, ENET_MMFR_DATA_MASK, ENET_MMFR_DATA_SHIFT,
		  data);
    write_32bit_mmio_register(&enet_regs_p->MMFR, reg_value);

    /*
     * Wait for SMI write to complete
     * (the MMI interrupt event bit is set in the EIR, when
     *  an SMI data transfer is completed)
     */
    do {
	reg_value = read_32bit_mmio_register(&enet_regs_p->EIR);
	polling_count --;
    } while ((reg_value & ENET_EIR_MII_MASK) == 0 && polling_count != 0);

    if ((reg_value & ENET_EIR_MII_MASK) == 0) {
	fdc_error_t fdc_error =
            CAPTURE_FDC_ERROR("SMI write failed", enet_device_p, reg_value);

        fatal_error_handler(fdc_error);
    }

    /*
     * Clear the MII interrupt event in the EIR register
     * (EIR is a w1c register)
     */
    write_32bit_mmio_register(&enet_regs_p->EIR, ENET_EIR_MII_MASK);
}


uint32_t
enet_phy_read(const struct enet_device *enet_device_p, uint32_t phy_reg)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    uint32_t reg_value;
    uint16_t polling_count = MAX_POLLING_COUNT;

    reg_value = read_32bit_mmio_register(&enet_regs_p->MSCR);
    FDC_ASSERT((reg_value & ENET_MSCR_MII_SPEED_MASK) != 0,
	       reg_value, enet_device_p);

    reg_value = read_32bit_mmio_register(&enet_regs_p->EIR);
    FDC_ASSERT((reg_value & ENET_EIR_MII_MASK) == 0,
	       reg_value, enet_device_p);

    /*
     * Set write command
     */
    reg_value = 0;
    SET_BIT_FIELD(reg_value, ENET_MMFR_ST_MASK, ENET_MMFR_ST_SHIFT,
		  0x1);
    SET_BIT_FIELD(reg_value, ENET_MMFR_OP_MASK, ENET_MMFR_OP_SHIFT,
		  ENET_MMFR_OP_READ_VALID_MII_MANAGEMENT_FRAME);
    SET_BIT_FIELD(reg_value, ENET_MMFR_PA_MASK, ENET_MMFR_PA_SHIFT,
		  ENET_PHY_ADDRESS);
    SET_BIT_FIELD(reg_value, ENET_MMFR_RA_MASK, ENET_MMFR_RA_SHIFT,
		  phy_reg);
    SET_BIT_FIELD(reg_value, ENET_MMFR_TA_MASK, ENET_MMFR_TA_SHIFT,
		  0x2);
    write_32bit_mmio_register(&enet_regs_p->MMFR, reg_value);

    /*
     * Wait for SMI read to complete
     * (the MMI interrupt event bit is set in the EIR, when
     *  an SMI data transfer is completed)
     */
    do {
	reg_value = read_32bit_mmio_register(&enet_regs_p->EIR);
	polling_count --;
    } while ((reg_value & ENET_EIR_MII_MASK) == 0 && polling_count != 0);

    if ((reg_value & ENET_EIR_MII_MASK) == 0) {
	fdc_error_t fdc_error =
            CAPTURE_FDC_ERROR("SMI read failed", enet_device_p, reg_value);

        fatal_error_handler(fdc_error);
    }

    reg_value = read_32bit_mmio_register(&enet_regs_p->MMFR);

    /*
     * Clear the MII interrupt event in the EIR register
     * (EIR is a w1c register)
     */
    write_32bit_mmio_register(&enet_regs_p->EIR, ENET_EIR_MII_MASK);

    return GET_BIT_FIELD(reg_value, ENET_MMFR_DATA_MASK, ENET_MMFR_DATA_SHIFT);
}


/**
 * Initializes the Ethernet PHY chip (Micrel KSZ8081RNA)
 */
static void
ethernet_phy_init(const struct enet_device *enet_device_p)
{
    uint32_t reg_value;
    uint16_t polling_count;
    fdc_error_t fdc_error;

    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    /*
     * Reset Phy
     */
    enet_phy_write(enet_device_p, ENET_PHY_CONTROL_REG, ENET_PHY_RESET_MASK);

    /*
     * Wait for reset to complete:
     */
    polling_count = MAX_POLLING_COUNT;
    do {
	reg_value = enet_phy_read(enet_device_p, ENET_PHY_CONTROL_REG);
	polling_count --;
    } while ((reg_value & ENET_PHY_RESET_MASK) != 0 && polling_count != 0);

    if ((reg_value & ENET_PHY_RESET_MASK) != 0) {
	fdc_error =
            CAPTURE_FDC_ERROR("Enet PHY reset failed", enet_device_p, reg_value);

        fatal_error_handler(fdc_error);
    }

    reg_value = enet_phy_read(enet_device_p, ENET_PHY_STATUS_REG);
    if ((reg_value & ENET_PHY_AUTO_NEG_CAPABLE_MASK) != 0 &&
	(reg_value & ENET_PHY_AUTO_NEG_COMPLETE_MASK) == 0) {
	/*
	 * Set auto-negotiation:
	 */
	reg_value = enet_phy_read(enet_device_p, ENET_PHY_CONTROL_REG);
	reg_value |= ENET_PHY_AUTO_NEGOTIATION_MASK;
	enet_phy_write(enet_device_p, ENET_PHY_CONTROL_REG, reg_value);

	/*
	 * Wait for auto-negotiation completion:
	 */
	polling_count = MAX_POLLING_COUNT;
	do {
	    reg_value = enet_phy_read(enet_device_p, ENET_PHY_STATUS_REG);
	    polling_count --;
	} while ((reg_value & ENET_PHY_AUTO_NEG_COMPLETE_MASK) == 0 &&
		 polling_count != 0);

	if ((reg_value & ENET_PHY_RESET_MASK) != 0) {
	    fdc_error =
		CAPTURE_FDC_ERROR("Enet PHY auto-negotiation failed",
				  enet_device_p, reg_value);

	    fatal_error_handler(fdc_error);
	}
    }
}

static void
enet_tx_payload_buffer_pool_init(const struct enet_device *enet_device_p)
{
    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    rtos_k_pointer_circular_buffer_init(
	"Ethernet Tx buffer pool",
        ENET_MAX_TX_FRAME_BUFFERS,
        (void **)enet_var_p->tx_buffer_pool_entries,
        NULL,
        SOC_GET_CURRENT_CPU_ID(),
        &enet_var_p->tx_buffer_pool);

    for (unsigned int i = 0; i < ENET_MAX_TX_FRAME_BUFFERS; i ++) {
        bool write_ok = rtos_k_pointer_circular_buffer_write(
                            &enet_var_p->tx_buffer_pool,
			    enet_var_p->tx_buffer_descriptors[i].data_buffer,
                            false);

        DBG_ASSERT(write_ok, 0, 0);
    }
}


static void
enet_rx_payload_buffer_queue_init(const struct enet_device *enet_device_p)
{
    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    rtos_k_pointer_circular_buffer_init(
	"Ethernet Rx buffer queue",
        ENET_MAX_RX_FRAME_BUFFERS,
        (void **)enet_var_p->rx_buffer_queue_entries,
        NULL,
        SOC_GET_CURRENT_CPU_ID(),
        &enet_var_p->rx_buffer_queue);
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
    enet_tx_payload_buffer_pool_init(enet_device_p);
    enet_rx_payload_buffer_queue_init(enet_device_p);
    enet_var_p->initialized = true;
    DEBUG_PRINTF("Initialized device %s\n", enet_device_p->name);
}


void
k64f_enet_transmit_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct enet_device *enet_device_p =
        (struct enet_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;

    for ( ; ; ) {
	uint32_t reg_value = read_32bit_mmio_register(&enet_regs_p->EIR);

	if ((reg_value & ENET_EIR_TXF_MASK) == 0) {
	    break;
	}

	reg_value &= ~ENET_EIR_TXF_MASK;
	write_32bit_mmio_register(&enet_regs_p->EIR, reg_value);

	for (unsigned int i = 0; i < ENET_MAX_TX_FRAME_BUFFERS; i ++) {
	    struct enet_tx_buffer_descriptor *buffer_desc_p =
		&enet_var_p->tx_buffer_descriptors[i];

	    FDC_ASSERT((buffer_desc_p->control &
			(ENET_TX_BD_LAST_IN_FRAME_MASK | ENET_TX_BD_CRC_MASK)) ==
		       (ENET_TX_BD_LAST_IN_FRAME_MASK | ENET_TX_BD_CRC_MASK),
		       buffer_desc_p->control, buffer_desc_p);

	    if (i == ENET_MAX_TX_FRAME_BUFFERS - 1) {
		FDC_ASSERT(buffer_desc_p->control & ENET_TX_BD_WRAP_MASK,
		           buffer_desc_p->control, buffer_desc_p);
	    }

	    if (buffer_desc_p->control & ENET_TX_BD_READY_MASK) {
		continue;
	    }

	    void *tx_payload_buf = buffer_desc_p->data_buffer;
	    struct enet_buf_control_block *buf_control_block_p =
		((struct enet_buf_control_block *)tx_payload_buf) - 1;

	    DBG_ASSERT(buf_control_block_p->signature == ENET_TX_BUFFER_SIGNATURE,
		       buf_control_block_p->signature, buf_control_block_p);

	    if (!buf_control_block_p->in_transit) {
		continue;
	    }

	    buffer_desc_p->data_length = 0;
	    buffer_desc_p->control_extend1 &= ~ENET_TX_BD_INTERRUPT_MASK;
	    enet_free_tx_buffer(enet_device_p, tx_payload_buf);
	}
    }
}


void
k64f_enet_receive_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct enet_device *enet_device_p =
        (struct enet_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;

    for ( ; ; ) {
	uint32_t reg_value = read_32bit_mmio_register(&enet_regs_p->EIR);

	if ((reg_value & ENET_EIR_RXF_MASK) == 0) {
	    break;
	}

	reg_value &= ~ENET_EIR_RXF_MASK;
	write_32bit_mmio_register(&enet_regs_p->EIR, reg_value);

	for (unsigned int i = 0; i < ENET_MAX_RX_FRAME_BUFFERS; i ++) {
	    struct enet_rx_buffer_descriptor *buffer_desc_p =
		&enet_var_p->rx_buffer_descriptors[i];

	    FDC_ASSERT(buffer_desc_p->control & ENET_RX_BD_LAST_IN_FRAME_MASK,
		       buffer_desc_p->control, buffer_desc_p);

	    if (i == ENET_MAX_RX_FRAME_BUFFERS - 1) {
		FDC_ASSERT(buffer_desc_p->control & ENET_RX_BD_WRAP_MASK,
		           buffer_desc_p->control, buffer_desc_p);
	    }

	    if (buffer_desc_p->control & ENET_RX_BD_EMPTY_MASK) {
		continue;
	    }

	    void *rx_payload_buf = buffer_desc_p->data_buffer;
	    struct enet_buf_control_block *buf_control_block_p =
		((struct enet_buf_control_block *)rx_payload_buf) - 1;

	    DBG_ASSERT(buf_control_block_p->signature == ENET_RX_BUFFER_SIGNATURE,
		       buf_control_block_p->signature, buf_control_block_p);

	    if (!buf_control_block_p->in_transit) {
		continue;
	    }

	    buffer_desc_p->control_extend1 &= ~ENET_RX_BD_GENERATE_INTERRUPT_MASK;
	    enet_enqueue_rx_buffer(enet_device_p, rx_payload_buf);
	}
    }
}


/**
 * Allocates a Tx buffer from the Tx buffer pool of the given ENET device.
 * It returns a pointer to the beginning of the data payload section of
 * the allocated Tx buffer.
 */
void *
enet_allocate_tx_buffer(const struct enet_device *enet_device_p)
{
    void *tx_payload_buf = NULL;

    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    (void)rtos_k_pointer_circular_buffer_read(
		&enet_var_p->tx_buffer_pool,
        	&tx_payload_buf,
		true);

    DBG_ASSERT(tx_payload_buf != NULL, enet_device_p, 0);

    struct enet_buf_control_block *buf_control_block_p = ((struct enet_buf_control_block *)tx_payload_buf) - 1;

    FDC_ASSERT(buf_control_block_p->signature == ENET_TX_BUFFER_SIGNATURE,
	       buf_control_block_p->signature, buf_control_block_p);
    FDC_ASSERT(!buf_control_block_p->in_transit,
	       buf_control_block_p, enet_device_p);

    buf_control_block_p->in_transit = true;
    struct enet_tx_buffer_descriptor *tx_buf_desc_p = buf_control_block_p->tx_buf_desc_p;

    FDC_ASSERT(tx_buf_desc_p->data_buffer == tx_payload_buf,
	       tx_buf_desc_p->data_buffer, tx_payload_buf);
    FDC_ASSERT((tx_buf_desc_p->control & ENET_TX_BD_READY_MASK) == 0,
	       tx_buf_desc_p->control, tx_buf_desc_p);

    return tx_payload_buf;
}


/**
 * Frees a Tx buffer back to the Tx buffer pool of the corresponding
 * ENET device
 */
void
enet_free_tx_buffer(const struct enet_device *enet_device_p, void *tx_payload_buf)
{
    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;
    struct enet_buf_control_block *buf_control_block_p = ((struct enet_buf_control_block *)tx_payload_buf) - 1;

    FDC_ASSERT(buf_control_block_p->signature == ENET_TX_BUFFER_SIGNATURE,
	       buf_control_block_p->signature, buf_control_block_p);
    FDC_ASSERT(buf_control_block_p->in_transit,
	       buf_control_block_p, enet_device_p);

    buf_control_block_p->in_transit = false;
    struct enet_tx_buffer_descriptor *tx_buf_desc_p = buf_control_block_p->tx_buf_desc_p;

    FDC_ASSERT(tx_buf_desc_p->data_buffer == tx_payload_buf,
               tx_buf_desc_p->data_buffer, tx_payload_buf);
    FDC_ASSERT((tx_buf_desc_p->control & ENET_TX_BD_READY_MASK) == 0,
	       tx_buf_desc_p->control, tx_buf_desc_p);

    bool write_ok = rtos_k_pointer_circular_buffer_write(
			&enet_var_p->tx_buffer_pool,
			tx_payload_buf,
			false);

    FDC_ASSERT(write_ok, enet_device_p, tx_payload_buf);
}


void
enet_start_xmit(const struct enet_device *enet_device_p,
	        void *tx_payload_buf)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    struct enet_buf_control_block *buf_control_block_p =
	((struct enet_buf_control_block *)tx_payload_buf) - 1;

    DBG_ASSERT(buf_control_block_p->signature == ENET_TX_BUFFER_SIGNATURE,
	       buf_control_block_p->signature, buf_control_block_p);

    struct enet_tx_buffer_descriptor *tx_buf_desc_p = buf_control_block_p->tx_buf_desc_p;

    DBG_ASSERT(tx_buf_desc_p->data_buffer == tx_payload_buf,
               tx_buf_desc_p->data_buffer, tx_payload_buf);
    DBG_ASSERT((tx_buf_desc_p->control & ENET_TX_BD_READY_MASK) == 0,
	       tx_buf_desc_p->control, tx_buf_desc_p);

    tx_buf_desc_p->control |= ENET_TX_BD_READY_MASK;
    tx_buf_desc_p->control_extend1 |= ENET_TX_BD_INTERRUPT_MASK;

    /*
     * Activate Rx buffer descriptor ring:
     * (the Tx descriptor ring has at least one descriptor with the "ready"
     *  bit set in its control field)
     */
    write_32bit_mmio_register(&enet_regs_p->TDAR, ENET_TDAR_TDAR_MASK);
}


/**
 * Dequeues an Rx buffer from the Rx buffer queue of the given ENET device.
 * It returns a pointer to the beginning of the data payload section of
 * the dequeued Rx buffer.
 */
void *
enet_dequeue_rx_buffer(const struct enet_device *enet_device_p)
{
    void *rx_payload_buf = NULL;

    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    (void)rtos_k_pointer_circular_buffer_read(
		&enet_var_p->rx_buffer_queue,
        	&rx_payload_buf,
		true);

    DBG_ASSERT(rx_payload_buf != NULL, enet_device_p, 0);

    struct enet_buf_control_block *buf_control_block_p = ((struct enet_buf_control_block *)rx_payload_buf) - 1;

    FDC_ASSERT(buf_control_block_p->signature == ENET_RX_BUFFER_SIGNATURE,
	       buf_control_block_p->signature, buf_control_block_p);
    FDC_ASSERT(!buf_control_block_p->in_transit,
	       buf_control_block_p, enet_device_p);

    buf_control_block_p->in_transit = true;

    struct enet_rx_buffer_descriptor *rx_buf_desc_p = buf_control_block_p->rx_buf_desc_p;

    FDC_ASSERT(rx_buf_desc_p->data_buffer == rx_payload_buf,
               rx_buf_desc_p->data_buffer, rx_payload_buf);
    FDC_ASSERT((rx_buf_desc_p->control & ENET_RX_BD_EMPTY_MASK) == 0,
	       rx_buf_desc_p->control, rx_buf_desc_p);

    return rx_payload_buf;
}


/**
 * Enqueue a Rx buffer into the Rx buffer queue of the corresponding
 * ENET device
 */
void
enet_enqueue_rx_buffer(const struct enet_device *enet_device_p, void *rx_payload_buf)
{
    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;
    struct enet_buf_control_block *buf_control_block_p = ((struct enet_buf_control_block *)rx_payload_buf) - 1;

    FDC_ASSERT(buf_control_block_p->signature == ENET_RX_BUFFER_SIGNATURE,
	       buf_control_block_p->signature, buf_control_block_p);
    FDC_ASSERT(buf_control_block_p->in_transit,
	       buf_control_block_p, enet_device_p);

    buf_control_block_p->in_transit = false;

    struct enet_rx_buffer_descriptor *rx_buf_desc_p = buf_control_block_p->rx_buf_desc_p;

    FDC_ASSERT(rx_buf_desc_p->data_buffer == rx_payload_buf,
               rx_buf_desc_p->data_buffer, rx_payload_buf);
    FDC_ASSERT((rx_buf_desc_p->control & ENET_RX_BD_EMPTY_MASK) == 0,
	       rx_buf_desc_p->control, rx_buf_desc_p);

    bool write_ok = rtos_k_pointer_circular_buffer_write(
			&enet_var_p->rx_buffer_queue,
			rx_payload_buf,
			false);

    FDC_ASSERT(write_ok, enet_device_p, rx_payload_buf);
}


/**
 * Recycle the given Rx buffer, by marking the corresponding Rx descriptor
 * as empty and reactivating the Rx descriptor ring.
 */
void
enet_recycle_rx_buffer(const struct enet_device *enet_device_p, void *rx_payload_buf)
{
    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    struct enet_buf_control_block *buf_control_block_p =
	((struct enet_buf_control_block *)rx_payload_buf) - 1;

    FDC_ASSERT(buf_control_block_p->signature == ENET_RX_BUFFER_SIGNATURE,
	       buf_control_block_p->signature, buf_control_block_p);
    FDC_ASSERT(buf_control_block_p->in_transit,
	       buf_control_block_p, enet_device_p);

    struct enet_rx_buffer_descriptor *rx_buf_desc_p = buf_control_block_p->rx_buf_desc_p;

    FDC_ASSERT(rx_buf_desc_p->data_buffer == rx_payload_buf,
               rx_buf_desc_p->data_buffer, rx_payload_buf);
    FDC_ASSERT((rx_buf_desc_p->control & ENET_RX_BD_EMPTY_MASK) == 0,
	       rx_buf_desc_p->control, rx_buf_desc_p);

    rx_buf_desc_p->control |= ENET_RX_BD_EMPTY_MASK;
    rx_buf_desc_p->control_extend1 |= ENET_RX_BD_GENERATE_INTERRUPT_MASK;

    /*
     * Re-activate Rx buffer descriptor ring:
     * (the Rx descriptor ring has at least one descriptor with the "empty"
     *  bit set in its control field)
     */
    write_32bit_mmio_register(&enet_regs_p->RDAR, ENET_RDAR_RDAR_MASK);
}


