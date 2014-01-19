/**
 * @file ethernet.c
 *
 * LPC2478 Ethernet driver
 *
 * @author German Rivera 
 */ 

#include "utils.h"
#include "lpc2478_stk_board.h"
#include "hardware.h"
#include "debug.h"
#include "rtos_wrapper.h"
#include "lpc2478_ethernet.h"
#include "ethernet.h"

/**
 * Buffer to hold a data fragment of an Ethernet frame
 */
typedef uint32_t eth_data_fragment_buffer_t[ETH_MAX_FRAGMENT_SIZE / sizeof(uint32_t)];

/**
 * Ethernet device (constant part)
 */
typedef const struct ethernet_device const_ethernet_device_t;
struct ethernet_device
{
#   define   ETH_DEVICE_SIGNATURE  GEN_SIGNATURE('E', 'T', 'H', 'D')
    uint32_t eth_signature;

    /**
     * Memory-mapped registers of the LPC2478 ethernet controller
     */
    lpc2478_ethernet_t *eth_lpc2478_ethernet_p;
   
    /**
     * VIC channel for Ethernet interruptsq
     */ 
    vic_interrupt_channel_t eth_vic_channel;
    
    /**
     * PCONP register mask
     */
    uint32_t eth_pconp_mask;

    /**
     * LPC2478 RMII (reduced media independent interface) pins connected to the
     * Ethernet PHY chip (KS8721 chip)
     */
#   define ETH_NUM_TX_RX_DATA_BITS  2
    struct pin_config_info eth_pin_enet_tx_en;
    struct pin_config_info eth_pin_enet_txd[ETH_NUM_TX_RX_DATA_BITS];
    struct pin_config_info eth_pin_enet_rxd[ETH_NUM_TX_RX_DATA_BITS];
    struct pin_config_info eth_pin_enet_rx_er;
    struct pin_config_info eth_pin_enet_crs;
    struct pin_config_info eth_pin_enet_ref_clk;
    
    /**
     * LPC2478 MIIM pins connected to the Ethernet PHY chip
     */
    struct pin_config_info eth_pin_enet_mdc;
    struct pin_config_info eth_pin_enet_mdio;

    /**
     * Pointer to the Ethernet device state variables stored in the Ethernet SRAM
     */
    struct eth_device_sram_variables *eth_device_sram_variables_p;

    /**
     * Pointer to array of data fragment buffers for Ethernet frames
     */
    eth_data_fragment_buffer_t  *eth_data_fragment_buffers_p;
};

/**
 * Header of an Ethernet frame
 */
struct eth_frame_header
{
    /**
     * Destination MAC address
     */ 
    uint8_t efh_destination_mac_address[6];

    /**
     * Source MAC address
     */
    uint8_t efh_source_mac_address[6];

    /**
     * Frame type (in big endian)
     */
    uint16_t efh_frame_type;
};

C_ASSERT(
    offsetof(struct eth_frame_header, efh_frame_type)  + sizeof(uint16_t) ==
    sizeof(struct eth_frame_header));

/**
 * State variables of the Ethernet device (stored in the Ethernet SRAM)
 */
struct eth_device_sram_variables
{
    /**
     * PHY address value to use for the PHY address field of the reg_MADR register
     */
    uint8_t eth_phy_address;

    /*
     * Reserved fields (alignment holes)
     */
    uint8_t eth_reserved1;
    uint16_t eth_reserved2;

    /**
     * Array of transmit descriptors
     */
    struct eth_transmit_descriptor eth_transmit_descriptors[ETH_NUM_TRANSMIT_DESCRIPTORS];

    /**
     * Array of transmit status entries
     */
    struct eth_transmit_status eth_transmit_statuses[ETH_NUM_TRANSMIT_DESCRIPTORS];

    /**
     * Array of receive descriptors
     */
    struct eth_receive_descriptor eth_receive_descriptors[ETH_NUM_RECEIVE_DESCRIPTORS];

    /**
     * Array of receive status entries
     */
    struct eth_receive_status eth_receive_statuses[ETH_NUM_RECEIVE_DESCRIPTORS];

    /**
     * Pool of fragments to store Ethernet headers for outgoing Ethernet frames
     */
    struct eth_frame_header eth_tx_frame_header_fragments[ETH_NUM_TX_FRAME_HEADER_FRAGMENTS];

    /**
     * Pool of fragments to store Ethernet headers for incoming Ethernet frames
     */
    struct eth_frame_header eth_rx_frame_header_fragments[ETH_NUM_RX_FRAME_HEADER_FRAGMENTS];
};

C_ASSERT(sizeof(struct eth_device_sram_variables) <= ETHERNET_SRAM_SIZE);

/**
 * Ethernet device (constant part)
 */
static const_ethernet_device_t g_ethernet_device =
{
    .eth_signature = ETH_DEVICE_SIGNATURE,
    .eth_lpc2478_ethernet_p = (lpc2478_ethernet_t *)LPC2478_ETHERNET_BASE_ADDR,
    .eth_vic_channel = VIC_CHANNEL_ETHERNET,
    .eth_pconp_mask = PCONP_PCENET,
    .eth_pin_enet_tx_en = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_TX_EN_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_txd[0] = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_TXD0_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_txd[1] = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_TXD1_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_rxd[0] = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_RXD0_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_rxd[1] = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_RXD1_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_rx_er = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_RX_ER_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_crs = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_CRS_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_ref_clk = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_REF_CLK_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_mdc = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_MDC_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_pin_enet_mdio = PIN_COFIG_INFO_INITIALIZER(
        ETHERNET_PINS_GPIO_PORT, ETHERNET_ENET_MDIO_PIN_BIT_INDEX,
        ETHERNET_PINS_GPIO_FUNCTION, true),
    .eth_device_sram_variables_p =
        (struct eth_device_sram_variables *)ETHERNET_SRAM_BASE,
    .eth_data_fragment_buffers_p =
        (eth_data_fragment_buffer_t *)ETHERNET_DATA_FRAGMENT_BUFFER_POOL_BASE_ADDR,
};


/**
 * MAC address (in big-endian) for the Ethernet interface
 */
static const uint8_t g_mac_address[] = {
    0x00,	
    0x1A,
    0xF1,
    0x00,
    0x00,
    0xF6
};


static uint32_t
read_phy_mii_management_interface(
    lpc2478_ethernet_t *lpc2478_ethernet_p,
    uint_fast8_t phy_addr,
    enum eth_mii_register_indexes phy_reg)
{
    uint32_t reg_value;

    ASSERT(
        phy_addr >= ETH_MADR_MIN_PHY_ADDRESS &&
        phy_addr <= ETH_MADR_MAX_PHY_ADDRESS,
        phy_addr, 0);

   /*
    * Set read command
    */
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MCMD, ETH_MCMD_READ_MASK);
  
    /*
     * Set MIIM address register:
     */
    
    reg_value = 0;
    SET_BIT_FIELD(
        reg_value,
        ETH_MADR_REGISTER_ADDRESS_MASK,
        ETH_MADR_REGISTER_ADDRESS_SHIFT,
        phy_reg);
    
    SET_BIT_FIELD(
        reg_value,
        ETH_MADR_PHY_ADDRESS_MASK,
        ETH_MADR_PHY_ADDRESS_SHIFT,
        phy_addr);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MADR, reg_value);

    do {
        reg_value = read_32bit_mmio_register(&lpc2478_ethernet_p->reg_MIND);
    } while ((reg_value & (ETH_MIND_BUSY_MASK | ETH_MIND_NOT_VALID_MASK)) != 0);

   /*
    * Clear command register to de-assert the read command
    */
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MCMD, 0);

    /*
     * Retrieve data read
     */ 
    reg_value = read_32bit_mmio_register(&lpc2478_ethernet_p->reg_MRDD);
    return reg_value;
}


static void
write_phy_mii_management_interface(
    lpc2478_ethernet_t *lpc2478_ethernet_p,
    uint_fast8_t phy_addr,
    enum eth_mii_register_indexes phy_reg,
    uint32_t data_value)
{
    uint32_t reg_value;

    ASSERT(
        phy_addr >= ETH_MADR_MIN_PHY_ADDRESS &&
        phy_addr <= ETH_MADR_MAX_PHY_ADDRESS,
        phy_addr, 0);

   /*
    * Clear command register to indicate that we want to do a write cycle
    * to the MIIM
    */
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MCMD, 0);
  
    /*
     * Set MIIM address register:
     */
    
    reg_value = 0;
    SET_BIT_FIELD(
        reg_value,
        ETH_MADR_REGISTER_ADDRESS_MASK,
        ETH_MADR_REGISTER_ADDRESS_SHIFT,
        phy_reg);
    
    SET_BIT_FIELD(
        reg_value,
        ETH_MADR_PHY_ADDRESS_MASK,
        ETH_MADR_PHY_ADDRESS_SHIFT,
        phy_addr);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MADR, reg_value);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MWTD, data_value);

    do {
        reg_value = read_32bit_mmio_register(&lpc2478_ethernet_p->reg_MIND);
    } while ((reg_value & ETH_MIND_BUSY_MASK) != 0);
}


static void
write_eth_reg_supp(lpc2478_ethernet_t *lpc2478_ethernet_p, uint32_t reg_value)
{

    CLEAR_BIT_FIELD(reg_value, ETH_SUPP_RESERVED1_MASK);
    CLEAR_BIT_FIELD(reg_value, ETH_SUPP_RESERVED2_MASK);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_SUPP, reg_value);
}


static void
write_eth_reg_mac2(lpc2478_ethernet_t *lpc2478_ethernet_p, uint32_t reg_value)
{

    CLEAR_BIT_FIELD(reg_value, ETH_MAC2_RESERVED1_MASK);
    CLEAR_BIT_FIELD(reg_value, ETH_MAC2_RESERVED2_MASK);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MAC2, reg_value);
}


static void
write_eth_reg_command(lpc2478_ethernet_t *lpc2478_ethernet_p, uint32_t reg_value)
{

    CLEAR_BIT_FIELD(reg_value, ETH_CMD_RESERVED1_MASK);
    CLEAR_BIT_FIELD(reg_value, ETH_CMD_RESERVED2_MASK);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_Command, reg_value);
}


/**
 * Configures the PHY via the MIIM interface of the MAC
 */
static dbg_error_t
init_ethernet_phy(
    lpc2478_ethernet_t *lpc2478_ethernet_p,
    uint8_t *phy_address_p,
    uint32_t *link_partner_ability_p)
{
    uint32_t reg_value;
    dbg_error_t dbg_error;
    uint_fast8_t phy_addr;
    
    *phy_address_p = 0;
    *link_partner_ability_p = 0;

    /*
     * Initialize the access to the PHY, by setting the MCFG regtister
     */

    reg_value = 0;  
    SET_BIT_FIELD(
        reg_value,
        ETH_MCFG_CLOCK_SELECT_MASK,
        ETH_MCFG_CLOCK_SELECT_SHIFT,
        ETH_MCFG_HOST_CLOCK_DIVIDED_BY_20);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MCFG, reg_value);

    /*
     * Find phy address for the PHY chip
     */
    for (phy_addr = ETH_MADR_MIN_PHY_ADDRESS;
         phy_addr <= ETH_MADR_MAX_PHY_ADDRESS;
         phy_addr ++)
    {
        /*
         * Read the PHY ID registers 
         */

        uint32_t phy_id1 = read_phy_mii_management_interface(
                            lpc2478_ethernet_p,
                            phy_addr,
                            ETH_MII_PHYSID1);

        uint32_t phy_id2 = read_phy_mii_management_interface(
                            lpc2478_ethernet_p,
                            phy_addr,
                            ETH_MII_PHYSID2);
      
        ASSERT(
            phy_id1 <= UINT16_MAX && phy_id2 <= UINT16_MAX,
            phy_id1, phy_id2);

        uint32_t phy_id = (phy_id1 << 16) | phy_id2;

        if (phy_id == ETHERNET_PHY_ID)
        {
            break;
        }
    }

    if (phy_addr > ETH_MADR_MAX_PHY_ADDRESS)
    {
        dbg_error = CAPTURE_DBG_ERROR(
            "PHY address not found", ETHERNET_PHY_ID, 0);

        goto error_exit;
    }

    /*
     * Perform PHY reset
     */
    write_phy_mii_management_interface(
        lpc2478_ethernet_p,
        phy_addr,
        ETH_MII_BMCR,
        ETH_MII_BMCR_RESET_MASK);

    /*
     * Wait until PHY reset is complete
     */
    do {
        reg_value = read_phy_mii_management_interface(
                        lpc2478_ethernet_p,
                        phy_addr,
                        ETH_MII_BMCR);
    } while ((reg_value & ETH_MII_BMCR_RESET_MASK) != 0);

    /*
     * Set auto-negotiation:
     */
    write_phy_mii_management_interface(
        lpc2478_ethernet_p,
        phy_addr,
        ETH_MII_BMCR,
        ETH_MII_BMCR_ANENABLE_MASK | ETH_MII_BMCR_ANRESTART_MASK);

    /*
     * Wait until auto-negotiation is complete:
     */
    do {
        reg_value = read_phy_mii_management_interface(
                        lpc2478_ethernet_p,
                        phy_addr,
                        ETH_MII_BMSR);
    } while ((reg_value & ETH_MII_BMSR_ANEGCOMPLETE_MASK) == 0);

    /*
     * Read link partner ability register:
     */
    reg_value = read_phy_mii_management_interface(
                    lpc2478_ethernet_p,
                    phy_addr,
                    ETH_MII_LPA);

    *phy_address_p = phy_addr;
    *link_partner_ability_p = reg_value;
    return 0;

error_exit:
    return dbg_error;
}


/**
 * Set link speed and duplex mode according to the link partner's abilities
 */
static void
set_ethernet_link_speed(
    lpc2478_ethernet_t *lpc2478_ethernet_p,
    uint32_t link_partner_ability)
{
    uint32_t reg_supp_value = read_32bit_mmio_register(
                                &lpc2478_ethernet_p->reg_SUPP);

    uint32_t reg_mac2_value = read_32bit_mmio_register(
                                &lpc2478_ethernet_p->reg_MAC2);

    uint32_t reg_command_value = read_32bit_mmio_register(
                                &lpc2478_ethernet_p->reg_Command);

    uint32_t reg_ipgt_value = 0;

    if ((link_partner_ability &
            (ETH_MII_LPA_100BASE_FULL_MASK | ETH_MII_LPA_10BASE_FULL_MASK)) != 0)
    {
        /* 
         * Full-duplex
         */
        reg_mac2_value |= (
            ETH_MAC2_FULL_DUPLEX_MASK |
            ETH_MAC2_CRC_ENABLE_MASK |
            ETH_MAC2_PAD_CRC_ENABLE_MASK |
            ETH_MAC2_AUTO_DETECT_PAD_MASK);

        reg_command_value |= ETH_CMD_FULL_DUPLEX_MASK;

        SET_BIT_FIELD(
            reg_ipgt_value,
            ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_MASK,
            ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_SHIFT,
            ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_FULL_VALUE);
    }
    else
    {
        /*
         * Half-duplex
         */
        reg_mac2_value &= ~ETH_MAC2_FULL_DUPLEX_MASK;
        reg_mac2_value |= (
            ETH_MAC2_CRC_ENABLE_MASK |
            ETH_MAC2_PAD_CRC_ENABLE_MASK |
            ETH_MAC2_AUTO_DETECT_PAD_MASK);

        reg_command_value &= ~ETH_CMD_FULL_DUPLEX_MASK;

        SET_BIT_FIELD(
            reg_ipgt_value,
            ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_MASK,
            ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_SHIFT,
            ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_HALF_VALUE);
    }

    if ((link_partner_ability & (ETH_MII_LPA_100BASE_MASK | ETH_MII_LPA_100BASE_FULL_MASK)) != 0)
    {
        reg_supp_value |= ETH_SUPP_SPEED_MASK;
    }
    else
    {
        ASSERT(
            (link_partner_ability & (ETH_MII_LPA_10BASE_MASK | ETH_MII_LPA_10BASE_FULL_MASK)) != 0,
            link_partner_ability, lpc2478_ethernet_p);

        reg_supp_value &= ~ETH_SUPP_SPEED_MASK;
    }

    write_eth_reg_supp(lpc2478_ethernet_p, reg_supp_value);
    write_eth_reg_mac2(lpc2478_ethernet_p, reg_mac2_value);
    write_eth_reg_command(lpc2478_ethernet_p, reg_command_value);
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_IPGT, reg_ipgt_value);
}


static void
init_dma_engine(const_ethernet_device_t *eth_device_p)
{
    uint32_t reg_value;
    lpc2478_ethernet_t *lpc2478_ethernet_p =
        eth_device_p->eth_lpc2478_ethernet_p;

    struct eth_device_sram_variables *eth_sram_variables_p =
        eth_device_p->eth_device_sram_variables_p;

    eth_data_fragment_buffer_t *data_fragment_buffers_p = 
        eth_device_p->eth_data_fragment_buffers_p;

}


static dbg_error_t
init_ethernet_device(const_ethernet_device_t *eth_device_p)
{
    uint32_t reg_value;
    dbg_error_t dbg_error;
    lpc2478_ethernet_t *lpc2478_ethernet_p = eth_device_p->eth_lpc2478_ethernet_p;

    /*
     * Configure pins:
     */

    configure_pin(&eth_device_p->eth_pin_enet_tx_en, false);
    configure_pin(&eth_device_p->eth_pin_enet_rx_er, false);

    for (uint_fast8_t i = 0; i < ETH_NUM_TX_RX_DATA_BITS; i ++)
    {
        configure_pin(&eth_device_p->eth_pin_enet_txd[i], false);
        configure_pin(&eth_device_p->eth_pin_enet_rxd[i], false);
    }

    configure_pin(&eth_device_p->eth_pin_enet_crs, false);
    configure_pin(&eth_device_p->eth_pin_enet_ref_clk, false);
    configure_pin(&eth_device_p->eth_pin_enet_mdc, false);
    configure_pin(&eth_device_p->eth_pin_enet_mdio, false);

    /*
     * Turn on Ethernet power in the System Control Block
     */
    turn_on_power(eth_device_p->eth_pconp_mask);

    /*
     * Reset the Ethernet controller:
     */
    
    reg_value = ETH_MAC1_RESET_TX_MASK |
                ETH_MAC1_RESET_MCS_TX_MASK |
                ETH_MAC1_RESET_RX_MASK |
                ETH_MAC1_RESET_MCS_RX_MASK |
                ETH_MAC1_SIMULATION_RESET_MASK |
                ETH_MAC1_SOFT_RESET_MASK;

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MAC1, reg_value);
  
    micro_delay(10);

    /*
     * Remove the soft reset condition from the MAC:
     */

    reg_value = read_32bit_mmio_register(
                    &lpc2478_ethernet_p->reg_MAC1);

    CLEAR_BIT_FIELD(reg_value, ETH_MAC1_RESERVED1_MASK);

    ASSERT(
        (reg_value & ETH_MAC1_SOFT_RESET_MASK) != 0,
        reg_value, ETH_MAC1_SOFT_RESET_MASK);

    reg_value &= ~ETH_MAC1_SOFT_RESET_MASK;

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_MAC1, reg_value);
   
    /*
     * Select RMII mode to interface with the PHY chip
     */

    reg_value = read_32bit_mmio_register(
                    &lpc2478_ethernet_p->reg_Command);

    reg_value |= ETH_CMD_RMII_MASK;

    write_eth_reg_command(lpc2478_ethernet_p, reg_value);

    /*
     * Select 100Mbps speed
     */
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_SUPP, ETH_SUPP_SPEED_MASK);

    micro_delay(50); // XXX

    /*
     * Set IPGR register
     */

    reg_value = 0;
    SET_BIT_FIELD(
        reg_value,
        ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART2_MASK,
        ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART2_SHIFT,
        ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART2_VALUE);

    SET_BIT_FIELD(
        reg_value,
        ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART1_MASK,
        ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART1_SHIFT,
        ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART1_VALUE);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_IPGR, reg_value);

    /*
     * Set the CLRT register:
     */

    reg_value = 0;
    SET_BIT_FIELD(
        reg_value,
        ETH_CLRT_RETRANSMISSION_MAXIMUM_MASK,
        ETH_CLRT_RETRANSMISSION_MAXIMUM_SHIFT,
        ETH_CLRT_RETRANSMISSION_MAXIMUM_VALUE);

    SET_BIT_FIELD(
        reg_value,
        ETH_CLRT_COLLISION_WINDOW_MASK,
        ETH_CLRT_COLLISION_WINDOW_SHIFT,
        ETH_CLRT_COLLISION_WINDOW_VALUE);

    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_CLRT, reg_value);

    /*
     * Program the MAC Address:
     */
    reg_value = 0;
    SET_BIT_FIELD(reg_value, ETH_SA_HIGH_OCTET_MASK, ETH_SA_HIGH_OCTET_SHIFT, g_mac_address[4]);
    SET_BIT_FIELD(reg_value, ETH_SA_LOW_OCTET_MASK, ETH_SA_LOW_OCTET_SHIFT, g_mac_address[5]);
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_SA[0], reg_value);

    reg_value = 0;
    SET_BIT_FIELD(reg_value, ETH_SA_HIGH_OCTET_MASK, ETH_SA_HIGH_OCTET_SHIFT, g_mac_address[2]);
    SET_BIT_FIELD(reg_value, ETH_SA_LOW_OCTET_MASK, ETH_SA_LOW_OCTET_SHIFT, g_mac_address[3]);
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_SA[1], reg_value);

    reg_value = 0;
    SET_BIT_FIELD(reg_value, ETH_SA_HIGH_OCTET_MASK, ETH_SA_HIGH_OCTET_SHIFT, g_mac_address[0]);
    SET_BIT_FIELD(reg_value, ETH_SA_LOW_OCTET_MASK, ETH_SA_LOW_OCTET_SHIFT, g_mac_address[1]);
    write_32bit_mmio_register(
        &lpc2478_ethernet_p->reg_SA[2], reg_value);

    uint32_t link_partner_ability = 0;

    /*
     * Initialize Ethernet PHY chip:
     */
    dbg_error = init_ethernet_phy(
                    lpc2478_ethernet_p,
                    &eth_device_p->eth_device_sram_variables_p->eth_phy_address,
                    &link_partner_ability);
    if (dbg_error != 0)
    {
        goto error_exit;
    }

    /*
     * Set link speed and full-duplex/half-duplex mode:
     */
    set_ethernet_link_speed(lpc2478_ethernet_p, link_partner_ability);

    /*
     * Initialize DMA engine
     */
    init_dma_engine(eth_device_p);


#if 0 //XXX This could be in open_ethernet_device()
    	/* Enable Receiver and Transmitter */
  	REG_ETH_COMMAND |= 3; // (KREG_ETH_COMMAND_RX_ENABLE | KREG_ETH_COMMAND_TX_ENABLE);
	REG_ETH_MAC1 |= 1;
#endif

    return 0;

error_exit:
    return dbg_error;
}


void
init_ethernet(void)
{
    dbg_error_t dbg_error;

    dbg_error = init_ethernet_device(&g_ethernet_device);
    if (dbg_error != 0)
    {
        fatal_error_handler(fdc_error);
    }
}


