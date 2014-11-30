/**
 * @file k64f_soc_enet.c
 *
 * Ethernet MAC (ENET) device driver for the K64F SoC
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "k64f_soc.h"
#include "k64f_soc_enet.h"
#include "frdm_board.h"
#include "McRTOS_arm_cortex_m.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"

C_ASSERT(NET_PACKET_DATA_BUFFER_SIZE >= 256 &&
	 (NET_PACKET_DATA_BUFFER_SIZE & ~ENET_MRBR_R_BUF_SIZE_MASK) == 0);

/*
 * ENET PHY Registers
 */
enum enet_phy_registers
{
    ENET_PHY_CONTROL_REG =  0x0, /* basic control register */
    ENET_PHY_STATUS_REG =   0x1, /* basic status register */
    ENET_PHY_ID1_REG =	    0x2, /* identification register 1 */
    ENET_PHY_ID2_REG =	    0x3, /* identification register 2 */
    ENET_PHY_INTR_CONTROL_STATUS_REG = 0x1b, /* interrupt control/status register */
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

/*
 * Bit masks for ENET_PHY_INTR_CONTROL_STATUS_REG register flags
 */
#define ENET_PHY_RECEIVE_ERROR_INTR_ENABLE_MASK	BIT(14)
#define ENET_PHY_LINK_DOWN_INTR_ENABLE_MASK	BIT(10)
#define ENET_PHY_LINK_UP_INTR_ENABLE_MASK	BIT(8)
#define ENET_PHY_RECEIVE_ERROR_INTR_MASK	BIT(6)
#define ENET_PHY_LINK_DOWN_INTR_MASK		BIT(2)
#define ENET_PHY_LINK_UP_INTR_MASK		BIT(0)

/*
 * Bit masks for ENET_PHY_CONTROL2_REG register flags
 */
#define ENET_PHY_POWER_SAVING_MASK	    BIT(10) /* 1 = enabled, 0 = disabled, default 0 */
#define ENET_PHY_INTR_LEVEL_MASK	    BIT(9)  /* 1 = active high, 0 = active low, default 0 */
#define ENET_PHY_DISABLE_TRANSMITTER_MASK   BIT(3)  /* 1 = disabled, 0 = enabled, default 0 */

/**
 * Maximum number of iterations for a polling loop
 */
#define MAX_POLLING_COUNT   UINT16_MAX

/**
 * McRTOS interrupt objects for ENET Tx/Rx interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_enet_tx_p = NULL;
struct rtos_interrupt *g_rtos_interrupt_enet_rx_p = NULL;
struct rtos_interrupt *g_rtos_interrupt_enet_error_p = NULL;

/**
 * Global non-const structure for ENET MAC device
 * (allocated in SRAM space)
 */
struct enet_device_var g_enet_var = {
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
    .rmii_rxer_pin = PIN_INITIALIZER(PIN_PORT_A, 5, PIN_FUNCTION_ALT4),
    .rmii_rxd1_pin = PIN_INITIALIZER(PIN_PORT_A, 12, PIN_FUNCTION_ALT4),
    .rmii_rxd0_pin = PIN_INITIALIZER(PIN_PORT_A, 13, PIN_FUNCTION_ALT4),
    .rmii_crs_dv_pin = PIN_INITIALIZER(PIN_PORT_A, 14, PIN_FUNCTION_ALT4),
    .rmii_txen_pin = PIN_INITIALIZER(PIN_PORT_A, 15, PIN_FUNCTION_ALT4),
    .rmii_txd0_pin = PIN_INITIALIZER(PIN_PORT_A, 16, PIN_FUNCTION_ALT4),
    .rmii_txd1_pin = PIN_INITIALIZER(PIN_PORT_A, 17, PIN_FUNCTION_ALT4),
    .mii_txer_pin = PIN_INITIALIZER(PIN_PORT_A, 28, PIN_FUNCTION_ALT4),
    .rmii_mdio_pin = PIN_INITIALIZER(PIN_PORT_B, 0, PIN_FUNCTION_ALT4),
    .rmii_mdc_pin = PIN_INITIALIZER(PIN_PORT_B, 1, PIN_FUNCTION_ALT4),
    .enet_1588_tmr_pins = {
	[0] = PIN_INITIALIZER(PIN_PORT_C, 16, PIN_FUNCTION_ALT4),
	[1] = PIN_INITIALIZER(PIN_PORT_C, 17, PIN_FUNCTION_ALT4),
	[2] = PIN_INITIALIZER(PIN_PORT_C, 18, PIN_FUNCTION_ALT4),
	[3] = PIN_INITIALIZER(PIN_PORT_C, 19, PIN_FUNCTION_ALT4),
    },

    .tx_rtos_interrupt_params = {
            .irp_name_p = "ENET Transmit Interrupt",
            .irp_isr_function_p = k64f_enet_transmit_isr,
            .irp_arg_p =  (void *)&g_enet_device0,
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ENET_Transmit),
            .irp_priority = ENET_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
    },

    .tx_rtos_interrupt_pp = &g_rtos_interrupt_enet_tx_p,

    .rx_rtos_interrupt_params = {
            .irp_name_p = "ENET Receive Interrupt",
            .irp_isr_function_p = k64f_enet_receive_isr,
            .irp_arg_p =  (void *)&g_enet_device0,
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ENET_Receive),
            .irp_priority = ENET_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

    .rx_rtos_interrupt_pp = &g_rtos_interrupt_enet_rx_p,

    .error_rtos_interrupt_params = {
            .irp_name_p = "ENET Error Interrupt",
            .irp_isr_function_p = k64f_enet_error_isr,
            .irp_arg_p =  (void *)&g_enet_device0,
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ENET_Error),
            .irp_priority = ENET_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

    .error_rtos_interrupt_pp = &g_rtos_interrupt_enet_error_p,
    .clock_gate_mask = SIM_SCGC2_ENET_MASK,
    .mac_address = {
	 /*
	  * NOTE: Bit 1 of the first byte of the MAC address must 1
	  * for private MAC addresses (locally administered)
	  */
	.bytes = { 0x82, 0x88, 0x88, 0x88, 0x88, 0x80 + BOARD_INSTANCE }
    },

    .mpu_region_index = RTOS_NUM_GLOBAL_MPU_REGIONS,
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

    for (unsigned int i = 0; i < NET_MAX_TX_PACKETS; i ++) {
	volatile struct enet_tx_buffer_descriptor *buffer_desc_p =
	    &enet_var_p->tx_buffer_descriptors[i];

	buffer_desc_p->data_buffer = NULL;
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
	if (i == NET_MAX_TX_PACKETS - 1) {
	    buffer_desc_p->control |= ENET_TX_BD_WRAP_MASK;
	}
    }

    /*
     * The Tx descriptor ring is empty:
     */
    enet_var_p->tx_ring_entries_filled = 0;
    enet_var_p->tx_ring_write_cursor = &enet_var_p->tx_buffer_descriptors[0];
    enet_var_p->tx_ring_read_cursor = &enet_var_p->tx_buffer_descriptors[0];
}


static void
enet_rx_buffer_descriptor_ring_init(const struct enet_device *enet_device_p)
{
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    struct enet_device_var *enet_var_p = enet_device_p->var_p;
    struct local_l3_end_point *local_l3_end_point_p = enet_var_p->local_l3_end_point_p;

    FDC_ASSERT(local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
	       local_l3_end_point_p->signature, local_l3_end_point_p);

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
    write_32bit_mmio_register(&enet_regs_p->MRBR, NET_PACKET_DATA_BUFFER_SIZE);

    for (unsigned int i = 0; i < NET_MAX_RX_PACKETS; i ++) {
	volatile struct enet_rx_buffer_descriptor *buffer_desc_p =
	    &enet_var_p->rx_buffer_descriptors[i];

	struct network_packet *rx_packet_p = &local_l3_end_point_p->rx_packets[i];

	FDC_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE,
		   rx_packet_p->signature, rx_packet_p);

	rx_packet_p->state_flags = NET_PACKET_IN_RX_TRANSIT;
	rx_packet_p->rx_buf_desc_p = buffer_desc_p;
	buffer_desc_p->data_buffer = rx_packet_p->data_buffer;

	DBG_ASSERT((uintptr_t)buffer_desc_p->data_buffer %
		   NET_PACKET_DATA_BUFFER_ALIGNMENT == 0,
		   buffer_desc_p->data_buffer, enet_device_p);

	buffer_desc_p->data_length = NET_PACKET_DATA_BUFFER_SIZE;

	/*
	 * Set the wrap flag for the last buffer of the ring:
	 */
	if (i != NET_MAX_RX_PACKETS - 1) {
	    buffer_desc_p->control = 0;
	} else {
	    buffer_desc_p->control = ENET_RX_BD_WRAP_MASK;
	}

	/*
         * Mark buffer descriptor as "available for reception":
         */
	buffer_desc_p->control |= ENET_RX_BD_EMPTY_MASK;

        /*
	 * Enable generation of receive interrupts
	 */
        buffer_desc_p->control_extend1 = ENET_RX_BD_GENERATE_INTERRUPT_MASK;
    }

    /*
     * The Rx descriptor ring is full:
     */
    enet_var_p->rx_ring_entries_filled = NET_MAX_RX_PACKETS;
    enet_var_p->rx_ring_write_cursor = &enet_var_p->rx_buffer_descriptors[0];
    enet_var_p->rx_ring_read_cursor = &enet_var_p->rx_buffer_descriptors[0];
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

    FDC_ASSERT((reg_value & ENET_ECR_ETHEREN_MASK) == 0, reg_value,
	       enet_device_p);

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
    reg_value = enet_device_p->mac_address.bytes[0] << 24 |
		enet_device_p->mac_address.bytes[1] << 16 |
		enet_device_p->mac_address.bytes[2] << 8 |
		enet_device_p->mac_address.bytes[3];
    write_32bit_mmio_register(&enet_regs_p->PALR, reg_value);
    reg_value = enet_device_p->mac_address.bytes[4] << 24 |
		enet_device_p->mac_address.bytes[5] << 16;
    write_32bit_mmio_register(&enet_regs_p->PAUR, reg_value);

    /*
     * - Enable normal operating mode (Disable sleep mode)
     * - Enable buffer descriptor byte swapping
     *   (since ARM Cortex-M is little-endian):
     * - Enable enhanced frame time-stamping functions
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->ECR);
    reg_value &= ~ENET_ECR_SLEEP_MASK;
    reg_value |= ENET_ECR_DBSWP_MASK | ENET_ECR_EN1588_MASK;
    write_32bit_mmio_register(&enet_regs_p->ECR, reg_value);

    /*
     * Configure receive control register:
     * - Enable stripping of CRC field for incoming frames
     * - ???Enable frame padding remove for incoming frames
     * - Enable flow control
     * - Configure RMII interface to the Ethernet PHY
     * - Enable 100Mbps operation
     * - Disable internal loopback
     * - Set max incoming frame length (including CRC)
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->RCR);
    reg_value |= ENET_RCR_CRCFWD_MASK;
    //???reg_value |= ENET_RCR_PADEN_MASK;
    reg_value |= ENET_RCR_FCE_MASK;
    reg_value |= ENET_RCR_MII_MODE_MASK | ENET_RCR_RMII_MODE_MASK;

#if 0  /* Promiscuous mode is useful for debugging */
    reg_value |= ENET_RCR_PROM_MASK;
#endif

    reg_value &= ~ENET_RCR_RMII_10T_MASK;
    reg_value &= ~ENET_RCR_LOOP_MASK;
    SET_BIT_FIELD(reg_value, ENET_RCR_MAX_FL_MASK, ENET_RCR_MAX_FL_SHIFT,
		  ETHERNET_MAX_FRAME_SIZE);
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
     * - Enable IP header checksum offload
     *   (automatically insert IP header checksum)
     * - Enable layer-4 checksum offload (for TCP, UDP, ICMP)
     *   (automatically insert layer-4 checksum)
     * - Enable Tx FIFO shift 16, so that the data payload of
     *   an outgoing Ethernet frame can be 32-bit aligned in memory.
     *   (Like if 2 dummy bytes were added at the beginning of the
     *   14-byte long Ethernet header. However, with the SHIFT16 flag,
     *   the 2 dummy bytes are not transmitted on the wire)
     */

    reg_value = 0;

#   ifdef ENET_CHECKSUM_OFFLOAD
    reg_value =	ENET_TACC_PROCHK_MASK |
		ENET_TACC_IPCHK_MASK;
#   endif

#   ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
    reg_value |= ENET_TACC_SHIFT16_MASK;
#   endif

    write_32bit_mmio_register(&enet_regs_p->TACC, reg_value);

    /*
     * Set Rx accelerators:
     * - Enable padding removal for short IP frames
     * - Enable discard of frames with MAC layer errors
     * - Enable IP header checksum offload
     *   (automatically discard frames with wrong IP header checksum)
     * - Enable layer-4 checksum offload for TCP, UDP, ICMP
     *   (automatically discard frames with wrong layer-4 checksum)
     * - Enable Rx FIFO shift 16, so that the data payload of
     *   an incoming Ethernet frame can be 32-bit aligned in memory.
     *   (Like if 2 dummy bytes were added at the beginning of the
     *    14-byte long Ethernet header, right after the frame is
     *    received.)
     */

    reg_value =	ENET_RACC_PADREM_MASK |
		ENET_RACC_LINEDIS_MASK;

#   ifdef ENET_CHECKSUM_OFFLOAD
    reg_value |= ENET_RACC_IPDIS_MASK |
		 ENET_RACC_PRODIS_MASK;
#   endif

#   ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
    reg_value |= ENET_RACC_SHIFT16_MASK;
#   endif

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
		  SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ / 5);
    write_32bit_mmio_register(&enet_regs_p->MSCR, reg_value);

    /*
     * Setup MIB counters:
     */
    write_32bit_mmio_register(&enet_regs_p->MIBC,
			      ENET_MIBC_MIB_DIS_MASK | ENET_MIBC_MIB_CLEAR_MASK);
    write_32bit_mmio_register(&enet_regs_p->MIBC, 0x0);

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

    rtos_k_register_interrupt(
        &enet_device_p->error_rtos_interrupt_params,
        enet_device_p->error_rtos_interrupt_pp);

    DBG_ASSERT(
        *enet_device_p->error_rtos_interrupt_pp != NULL,
        enet_device_p->error_rtos_interrupt_pp, enet_device_p);
}

static void
enet_phy_write_nolock(const struct enet_device *enet_device_p, uint32_t phy_reg,
		      uint32_t data)
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


void
enet_phy_write(const struct enet_device *enet_device_p, uint32_t phy_reg, uint32_t data)
{
    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    FDC_ASSERT(enet_var_p->initialized, enet_device_p, enet_var_p);
    rtos_k_mutex_acquire(&enet_var_p->phy_mutex);
    enet_phy_write_nolock(enet_device_p, phy_reg, data);
    rtos_k_mutex_release(&enet_var_p->phy_mutex);
}


static uint32_t
enet_phy_read_nolock(const struct enet_device *enet_device_p, uint32_t phy_reg)
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
     * Set read command
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


uint32_t
enet_phy_read(const struct enet_device *enet_device_p, uint32_t phy_reg)
{
    uint32_t reg_value;
    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    FDC_ASSERT(enet_var_p->initialized, enet_device_p, enet_var_p);
    rtos_k_mutex_acquire(&enet_var_p->phy_mutex);
    reg_value =	enet_phy_read_nolock(enet_device_p, phy_reg);
    rtos_k_mutex_release(&enet_var_p->phy_mutex);
    return reg_value;
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

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    FDC_ASSERT(!enet_var_p->initialized, enet_device_p, enet_var_p);
    rtos_k_mutex_init("Ethernet PHY mutex", &enet_var_p->phy_mutex);

    /*
     * Reset Phy
     */
    enet_phy_write_nolock(enet_device_p, ENET_PHY_CONTROL_REG, ENET_PHY_RESET_MASK);

    /*
     * Wait for reset to complete:
     */
    polling_count = MAX_POLLING_COUNT;
    do {
	reg_value = enet_phy_read_nolock(enet_device_p, ENET_PHY_CONTROL_REG);
	polling_count --;
    } while ((reg_value & ENET_PHY_RESET_MASK) != 0 && polling_count != 0);

    if ((reg_value & ENET_PHY_RESET_MASK) != 0) {
	fdc_error =
            CAPTURE_FDC_ERROR("Enet PHY reset failed", enet_device_p, reg_value);

        fatal_error_handler(fdc_error);
    }

    reg_value = enet_phy_read_nolock(enet_device_p, ENET_PHY_STATUS_REG);
    if ((reg_value & ENET_PHY_AUTO_NEG_CAPABLE_MASK) != 0 &&
	(reg_value & ENET_PHY_AUTO_NEG_COMPLETE_MASK) == 0) {
	/*
	 * Set auto-negotiation:
	 */
	reg_value = enet_phy_read_nolock(enet_device_p, ENET_PHY_CONTROL_REG);
	reg_value |= ENET_PHY_AUTO_NEGOTIATION_MASK;
	enet_phy_write_nolock(enet_device_p, ENET_PHY_CONTROL_REG, reg_value);

	/*
	 * Wait for auto-negotiation completion:
	 */
	polling_count = MAX_POLLING_COUNT;
	do {
	    reg_value = enet_phy_read_nolock(enet_device_p, ENET_PHY_STATUS_REG);
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
     */

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(enet_device_p->enet_1588_tmr_pins);
	 ++ i) {
	set_pin_function(&enet_device_p->enet_1588_tmr_pins[i], 0);
    }

    set_pin_function(&enet_device_p->rmii_mdc_pin, 0);

    /*
     * Set "open drain enabled", "pull-up resistor enabled" and
     * "internal pull resistor enabled" for rmii_mdio pin
     *
     * NOTE: No external pullup is available on MDIO signal when the K64F SoC
     * requests status of the Ethernet link connection. Internal pullup
     * is required when port configuration for MDIO signal is enabled.
     */
    set_pin_function(&enet_device_p->rmii_mdio_pin,
		     PORT_PCR_ODE_MASK |
		     PORT_PCR_PE_MASK |
		     PORT_PCR_PS_MASK);

    set_pin_function(&enet_device_p->rmii_rxd0_pin, 0);
    set_pin_function(&enet_device_p->rmii_rxd1_pin, 0);
    set_pin_function(&enet_device_p->rmii_crs_dv_pin, 0);
    set_pin_function(&enet_device_p->rmii_rxer_pin, 0);
    set_pin_function(&enet_device_p->rmii_txd0_pin, 0);
    set_pin_function(&enet_device_p->rmii_txd1_pin, 0);
    set_pin_function(&enet_device_p->rmii_txen_pin, 0);
    set_pin_function(&enet_device_p->mii_txer_pin, 0);

    /*
     * NOTE: The K64F ENET MAC DMA engine does not work with the K64F MPU
     * so, unfortunately we have to disable the MPU
     */
#if 0
    /*
     * Configure MPU access for CPU and ENET DMA engine:
     */
    mpu_set_privileged_global_data_region(enet_device_p->mpu_region_index,
					  enet_var_p, enet_var_p + 1);
    mpu_set_mpu_region_for_dma(enet_device_p->mpu_region_index,
			       enet_var_p, enet_var_p + 1, 0);
#else
    mpu_disable();
    capture_fdc_msg_printf("MPU had to be disabled for ENET MAC DMA to work\n");
#endif

    ethernet_mac_init(enet_device_p);
    ethernet_phy_init(enet_device_p);
    enet_var_p->initialized = true;
    DEBUG_PRINTF("Initialized device %s\n", enet_device_p->name);
}


/**
 * Activates the Ethernet MAC module of the K64F SoC
 */
void
enet_start(const struct enet_device *enet_device_p,
	   struct local_l3_end_point *local_l3_end_point_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;
    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    FDC_ASSERT(enet_var_p->initialized, enet_device_p, enet_var_p);
    FDC_ASSERT(enet_var_p->local_l3_end_point_p == NULL,
               enet_var_p->local_l3_end_point_p, enet_var_p);

    enet_var_p->local_l3_end_point_p = local_l3_end_point_p;

    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /*
     * Initialize Tx buffer descriptor ring:
     */
    enet_tx_buffer_descriptor_ring_init(enet_device_p);

    /*
     * Initialize Rx buffer descriptor ring:
     */
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
			      ENET_EIMR_RXF_MASK |
			      ENET_EIMR_BABR_MASK |
			      ENET_EIMR_BABT_MASK |
			      ENET_EIMR_EBERR_MASK |
			      ENET_EIMR_UN_MASK |
			      ENET_EIMR_PLR_MASK);

    /*
     * Enable ENET module:
     */
    reg_value = read_32bit_mmio_register(&enet_regs_p->ECR);
    reg_value |= ENET_ECR_ETHEREN_MASK;
    write_32bit_mmio_register(&enet_regs_p->ECR, reg_value);

    /*
     * Activate Rx buffer descriptor ring:
     * (the Rx descriptor ring must have at least one descriptor with the "empty"
     *  bit set in its control field)
     */
    __DSB();
    write_32bit_mmio_register(&enet_regs_p->RDAR, ENET_RDAR_RDAR_MASK);

    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Transmit completion interrupt handler
 */
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

	/*
	 * Clear interrupt source (w1c):
	 */
	write_32bit_mmio_register(&enet_regs_p->EIR, ENET_EIR_TXF_MASK);

	cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

	FDC_ASSERT(enet_var_p->tx_ring_entries_filled > 0 &&
		   enet_var_p->tx_ring_entries_filled <= NET_MAX_TX_PACKETS,
		   enet_var_p->tx_ring_entries_filled, enet_device_p);

	FDC_ASSERT(enet_var_p->tx_ring_read_cursor != NULL,
		   enet_device_p, 0);

	volatile struct enet_tx_buffer_descriptor *buffer_desc_p =
	    enet_var_p->tx_ring_read_cursor;

	do {
	    FDC_ASSERT(buffer_desc_p >= &enet_var_p->tx_buffer_descriptors[0] &&
		       buffer_desc_p <= &enet_var_p->tx_buffer_descriptors[NET_MAX_TX_PACKETS - 1],
		       buffer_desc_p, enet_device_p);

	    FDC_ASSERT(buffer_desc_p != enet_var_p->tx_ring_write_cursor ||
		       enet_var_p->tx_ring_entries_filled == NET_MAX_TX_PACKETS,
		       buffer_desc_p, enet_var_p->tx_ring_write_cursor);

	    FDC_ASSERT((buffer_desc_p->control &
			(ENET_TX_BD_LAST_IN_FRAME_MASK | ENET_TX_BD_CRC_MASK)) ==
		       (ENET_TX_BD_LAST_IN_FRAME_MASK | ENET_TX_BD_CRC_MASK),
		       buffer_desc_p->control, buffer_desc_p);

	    if (buffer_desc_p->control & ENET_TX_BD_READY_MASK) {
		break;
	    }

	    struct network_packet *tx_packet_p =
		    BUFFER_TO_NETWORK_PACKET(buffer_desc_p->data_buffer);

	    DBG_ASSERT(tx_packet_p->signature == NET_TX_PACKET_SIGNATURE,
		       tx_packet_p->signature, tx_packet_p);

	    FDC_ASSERT(tx_packet_p->tx_buf_desc_p == buffer_desc_p,
		       tx_packet_p->tx_buf_desc_p, buffer_desc_p);

	    FDC_ASSERT(tx_packet_p->state_flags & NET_PACKET_IN_TX_TRANSIT,
		       tx_packet_p->state_flags, tx_packet_p);

	    FDC_ASSERT(tx_packet_p->state_flags & NET_PACKET_IN_TX_USE_BY_APP,
		       tx_packet_p->state_flags, tx_packet_p);

#if 0 /* To enable this, need to increase RTOS_MAX_TIME_INTERRUPTS_DISABLED */
	    DEBUG_PRINTF("Transmitted packet %p\n", tx_packet_p);
#endif
	    tx_packet_p->state_flags &= ~NET_PACKET_IN_TX_TRANSIT;
	    tx_packet_p->tx_buf_desc_p = NULL;
	    buffer_desc_p->data_buffer = NULL;
	    buffer_desc_p->control_extend1 &= ~ENET_TX_BD_INTERRUPT_MASK;
	    if (buffer_desc_p->control_extend0 &
	        (ENET_TX_BD_ERROR_MASK |
	         ENET_TX_BD_FIFO_OVERFLOW_ERROR_MASK |
	         ENET_TX_BD_TMESTAMP_ERROR_MASK |
		 ENET_TX_BD_FRAME_ERROR_MASK)) {
		(void)CAPTURE_FDC_ERROR("Ethernet transmission failed (Tx packet dropped)",
					buffer_desc_p->control_extend0, buffer_desc_p);
	    }

	    if (tx_packet_p->state_flags & NET_PACKET_FREE_AFTER_TX_COMPLETE) {
		tx_packet_p->state_flags &= ~NET_PACKET_FREE_AFTER_TX_COMPLETE;
		net_free_tx_packet(tx_packet_p);
	    }

	    if (buffer_desc_p->control & ENET_TX_BD_WRAP_MASK) {
		buffer_desc_p = &enet_var_p->tx_buffer_descriptors[0];
	    } else {
		buffer_desc_p ++;
	    }

	    enet_var_p->tx_ring_entries_filled --;
	} while (enet_var_p->tx_ring_entries_filled != 0);

	enet_var_p->tx_ring_read_cursor = buffer_desc_p;
	rtos_k_restore_cpu_interrupts(cpu_status_register);
    }
}


/**
 * Receive completion interrupt handler
 */
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

	/*
	 * Clear interrupt source (w1c):
	 */
	write_32bit_mmio_register(&enet_regs_p->EIR, ENET_EIR_RXF_MASK);

	cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

	FDC_ASSERT(enet_var_p->rx_ring_entries_filled > 0 &&
		   enet_var_p->rx_ring_entries_filled <= NET_MAX_RX_PACKETS,
		   enet_var_p->rx_ring_entries_filled, enet_device_p);

	FDC_ASSERT(enet_var_p->rx_ring_read_cursor != NULL,
		   enet_device_p, 0);

	volatile struct enet_rx_buffer_descriptor *buffer_desc_p =
	    enet_var_p->rx_ring_read_cursor;

	do {
	    FDC_ASSERT(buffer_desc_p >= &enet_var_p->rx_buffer_descriptors[0] &&
		       buffer_desc_p <= &enet_var_p->rx_buffer_descriptors[NET_MAX_RX_PACKETS - 1],
		       buffer_desc_p, enet_device_p);

	    FDC_ASSERT(buffer_desc_p != enet_var_p->rx_ring_write_cursor ||
		       enet_var_p->rx_ring_entries_filled == NET_MAX_RX_PACKETS,
		       buffer_desc_p, enet_var_p->rx_ring_write_cursor);

	    if (buffer_desc_p->control & ENET_RX_BD_EMPTY_MASK) {
		break;
	    }

	    FDC_ASSERT(buffer_desc_p->control & ENET_RX_BD_LAST_IN_FRAME_MASK,
	       buffer_desc_p->control, buffer_desc_p);

	    struct network_packet *rx_packet_p =
		BUFFER_TO_NETWORK_PACKET(buffer_desc_p->data_buffer);

	    DBG_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE,
		       rx_packet_p->signature, rx_packet_p);

	    FDC_ASSERT(rx_packet_p->state_flags & NET_PACKET_IN_RX_TRANSIT,
	               rx_packet_p->state_flags, rx_packet_p);

	    FDC_ASSERT(!(rx_packet_p->state_flags & NET_PACKET_IN_RX_USE_BY_APP),
		        rx_packet_p->state_flags, rx_packet_p);

	    rx_packet_p->state_flags &= ~NET_PACKET_IN_RX_TRANSIT;
	    rx_packet_p->rx_buf_desc_p = NULL;
	    buffer_desc_p->data_buffer = NULL;
	    buffer_desc_p->control_extend1 &= ~ENET_RX_BD_GENERATE_INTERRUPT_MASK;
	    if (buffer_desc_p->control &
	        (ENET_RX_BD_LENGTH_VIOLATION_MASK |
		 ENET_RX_BD_NON_OCTET_ALIGNED_FRAME_MASK |
		 ENET_RX_BD_CRC_ERROR_MASK |
		 ENET_RX_BD_FIFO_OVERRRUN_MASK |
		 ENET_RX_BD_FRAME_TRUNCATED_MASK)) {
		(void)CAPTURE_FDC_ERROR("Received bad frame (Rx packet dropped)",
					buffer_desc_p->control, buffer_desc_p);

		rx_packet_p->state_flags = NET_PACKET_RX_FAILED;
	    } else {
		FDC_ASSERT(buffer_desc_p->data_length <= ETHERNET_MAX_FRAME_DATA_SIZE,
			   buffer_desc_p->data_length, buffer_desc_p);

		rx_packet_p->total_length = buffer_desc_p->data_length;
	    }

#if 0 /* To enable this, need to increase RTOS_MAX_TIME_INTERRUPTS_DISABLED */
	    DEBUG_PRINTF("Received packet %p (type %#x, state_flags %#x)\n",
			 rx_packet_p, buffer_desc_p->protocol_type,
			 rx_packet_p->state_flags);
#endif
	    /*
	     * Queue received packet for layer-3 processing:
	     */
	    net_enqueue_rx_packet(enet_var_p->local_l3_end_point_p, rx_packet_p);

	    if (buffer_desc_p->control & ENET_RX_BD_WRAP_MASK) {
		buffer_desc_p = &enet_var_p->rx_buffer_descriptors[0];
	    } else {
		buffer_desc_p ++;
	    }

	    enet_var_p->rx_ring_entries_filled --;
	} while (enet_var_p->rx_ring_entries_filled != 0);

	enet_var_p->rx_ring_read_cursor = buffer_desc_p;
	rtos_k_restore_cpu_interrupts(cpu_status_register);
    }
}


void
k64f_enet_error_interrupt_e_handler(
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
    uint32_t reg_value = read_32bit_mmio_register(&enet_regs_p->EIR);

    uint32_t error_interrupt_mask = (reg_value &
				     (ENET_EIMR_BABR_MASK |
				      ENET_EIMR_BABT_MASK |
				      ENET_EIMR_EBERR_MASK |
				      ENET_EIMR_UN_MASK |
				      ENET_EIMR_PLR_MASK));

    if (error_interrupt_mask != 0) {
	capture_fdc_msg_printf("ENET %s error (error interrupt mask: %#x)\n",
			       enet_device_p->name, error_interrupt_mask);

	/*
	 * Clear interrupt source (w1c):
	 */
	write_32bit_mmio_register(&enet_regs_p->EIR, error_interrupt_mask);
        enet_var_p->tx_rx_error_count ++;
    }
}

/**
 * Initiates the transmission of a Tx packet, by assigning it to the next
 * available Tx descriptor in the Tx descriptor ring, marking that descriptor
 * as "ready" and re-activating the Tx descriptor ring.
 */
void
enet_start_xmit(const struct enet_device *enet_device_p,
	        struct network_packet *tx_packet_p)
{
    struct enet_device_var *const enet_var_p = enet_device_p->var_p;
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;

    DBG_ASSERT(tx_packet_p->signature == NET_TX_PACKET_SIGNATURE,
	       tx_packet_p->signature, tx_packet_p);

    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    FDC_ASSERT(enet_var_p->tx_ring_entries_filled < NET_MAX_TX_PACKETS,
	       enet_var_p->tx_ring_entries_filled, tx_packet_p);

    FDC_ASSERT(enet_var_p->tx_ring_write_cursor != enet_var_p->tx_ring_read_cursor ||
	       enet_var_p->tx_ring_entries_filled == 0,
	       enet_var_p->tx_ring_write_cursor, enet_var_p->tx_ring_read_cursor);

    volatile struct enet_tx_buffer_descriptor *tx_buf_desc_p =
	enet_var_p->tx_ring_write_cursor;

    FDC_ASSERT((tx_buf_desc_p->control & ENET_TX_BD_READY_MASK) == 0,
	       tx_buf_desc_p->control, tx_buf_desc_p);
    FDC_ASSERT(tx_buf_desc_p->data_buffer == NULL,
	       tx_buf_desc_p->data_buffer, tx_buf_desc_p);
    FDC_ASSERT(tx_packet_p->tx_buf_desc_p == NULL,
	       tx_packet_p->tx_buf_desc_p, tx_packet_p);

    tx_buf_desc_p->data_buffer = tx_packet_p->data_buffer;
    tx_packet_p->tx_buf_desc_p = tx_buf_desc_p;

    FDC_ASSERT(tx_packet_p->state_flags & NET_PACKET_IN_TX_USE_BY_APP,
	       tx_packet_p->state_flags, tx_packet_p);
    FDC_ASSERT(tx_packet_p->total_length != 0, tx_packet_p, enet_device_p);

    tx_buf_desc_p->data_length = tx_packet_p->total_length;
    tx_buf_desc_p->control_extend1 |= ENET_TX_BD_INTERRUPT_MASK;

    FDC_ASSERT(!(tx_packet_p->state_flags & NET_PACKET_IN_TX_TRANSIT),
	        tx_packet_p->state_flags, tx_packet_p);

    tx_packet_p->state_flags |= NET_PACKET_IN_TX_TRANSIT;

    /*
     * Mark buffer descriptor as "ready for transmission":
     */
    tx_buf_desc_p->control |= ENET_TX_BD_READY_MASK;

    /*
     * Advance Tx ring write cursor:
     */
    if (tx_buf_desc_p->control & ENET_TX_BD_WRAP_MASK) {
	enet_var_p->tx_ring_write_cursor = enet_var_p->tx_buffer_descriptors;
    } else {
	enet_var_p->tx_ring_write_cursor ++;
    }

    enet_var_p->tx_ring_entries_filled ++;
    rtos_k_restore_cpu_interrupts(cpu_status_register);

    /*
     * Re-activate Tx buffer descriptor ring, to start transmitting the frame:
     * (the Tx descriptor ring has at least one descriptor with the "ready"
     *  bit set in its control field)
     */
    __DSB();
    write_32bit_mmio_register(&enet_regs_p->TDAR, ENET_TDAR_TDAR_MASK);

#   ifdef DEBUG
    uint32_t reg_value = enet_phy_read(enet_device_p, ENET_PHY_STATUS_REG);

    if ((reg_value & ENET_PHY_LINK_UP_MASK) == 0) {
	DEBUG_PRINTF("link down for enet device %#p\n", enet_device_p);
    }
#   endif
}


/**
 * Re-post the given Rx packet to the ENET Rx ring, by assigning it to the next
 * available Rx descriptor in the Rx descriptor ring, marking that descriptor as
 * "empty" and re-activating the Rx descriptor ring.
 */
void
enet_repost_rx_packet(const struct enet_device *enet_device_p,
		      struct network_packet *rx_packet_p)
{
    FDC_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *enet_var_p = enet_device_p->var_p;
    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;

    FDC_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE,
	       rx_packet_p->signature, rx_packet_p);
    FDC_ASSERT(rx_packet_p->state_flags == NET_PACKET_IN_RX_USE_BY_APP ||
	       rx_packet_p->state_flags == NET_PACKET_RX_FAILED,
	       rx_packet_p->state_flags, rx_packet_p);

    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    FDC_ASSERT(enet_var_p->rx_ring_entries_filled < NET_MAX_RX_PACKETS,
	       enet_var_p->rx_ring_entries_filled, rx_packet_p);

    volatile struct enet_rx_buffer_descriptor *rx_buf_desc_p =
	enet_var_p->rx_ring_write_cursor;

    FDC_ASSERT((rx_buf_desc_p->control & ENET_RX_BD_EMPTY_MASK) == 0,
	       rx_buf_desc_p->control, rx_buf_desc_p);
    FDC_ASSERT(rx_buf_desc_p->data_buffer == NULL,
               rx_buf_desc_p->data_buffer, rx_buf_desc_p);
    DBG_ASSERT(rx_packet_p->rx_buf_desc_p == NULL,
	       rx_packet_p->rx_buf_desc_p, rx_packet_p);

    rx_buf_desc_p->data_buffer = rx_packet_p->data_buffer;
    rx_packet_p->rx_buf_desc_p = rx_buf_desc_p;
    rx_buf_desc_p->control_extend1 |= ENET_RX_BD_GENERATE_INTERRUPT_MASK;

    FDC_ASSERT(!(rx_packet_p->state_flags & NET_PACKET_IN_RX_TRANSIT),
	        rx_packet_p->state_flags, rx_packet_p);

    rx_packet_p->state_flags = NET_PACKET_IN_RX_TRANSIT;

    /*
     * Mark buffer descriptor as "ready for reception":
     */
    rx_buf_desc_p->control |= ENET_RX_BD_EMPTY_MASK;

    /*
     * Advance Rx ring write cursor:
     */
    if (rx_buf_desc_p->control & ENET_RX_BD_WRAP_MASK) {
	enet_var_p->rx_ring_write_cursor = enet_var_p->rx_buffer_descriptors;
    } else {
	enet_var_p->rx_ring_write_cursor ++;
    }

    enet_var_p->rx_ring_entries_filled ++;
    rtos_k_restore_cpu_interrupts(cpu_status_register);

    /*
     * Re-activate Rx buffer descriptor ring:
     * (the Rx descriptor ring has at least one descriptor with the "empty"
     *  bit set in its control field)
     */
    __DSB();
    write_32bit_mmio_register(&enet_regs_p->RDAR, ENET_RDAR_RDAR_MASK);
}


