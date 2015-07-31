/**
 * @file lpc2478_ethernet.h
 *
 * LPC2478 Ethernet Controller 
 *
 * @author German Rivera 
 */ 
#ifndef _LPC2478_ETHERNET_H
#define _LPC2478_ETHERNET_H

#include "hardware_abstractions.h"
#include "compile_time_checks.h"

/**
 * Base address for the Ethernet controller
 */
#define LPC2478_ETHERNET_BASE_ADDR    UINT32_C(0xFFE00000)

/**
 * Register space size for the ETHERNET
 */
#define LPC2478_ETHERNET_SIZE    UINT32_C(0xFFC)

/**
 * Memory-mapped I/O registers of the ETHERNET
 */
typedef volatile struct lpc2478_ethernet
{
    /*
     * MAC registers
     */

    /**
     * /R/W MAC configuration register 1.
     */ 
    uint32_t reg_MAC1;

    /**
     * Set this to allow receive frames to be received. Internally the MAC synchronizes
     * this control bit to the incoming receive stream.
     */ 
#   define  ETH_MAC1_RECEIVE_ENABLE_MASK    BIT(0)

    /** 
     * When enabled (set to ’1’), the MAC will pass all frames regardless of type (normal
     * vs. Control). When disabled, the MAC does not pass valid Control frames.
     */ 
#   define  ETH_MAC1_PASS_ALL_FRAMES_MASK    BIT(1)

    /** 
     * When enabled (set to ’1’), the MAC acts upon received PAUSE Flow Control
     * frames. When disabled, received PAUSE Flow Control frames are ignored.
     */ 
#   define  ETH_MAC1_RX_FLOW_CONTROL_MASK    BIT(2)

    /** 
     * When enabled (set to ’1’), PAUSE Flow Control frames are allowed to be
     * transmitted. When disabled, Flow Control frames are blocked.
     */ 
#   define  ETH_MAC1_TX_FLOW_CONTROL_MASK    BIT(3)

    /** 
     * Setting this bit will cause the MAC Transmit interface to be looped back to the MAC
     * Receive interface. Clearing this bit results in normal operation.
     */ 
#   define  ETH_MAC1_LOOPBACK_MASK          BIT(4)

#   define  ETH_MAC1_UNUSED1_MASK           MULTI_BIT_MASK(7, 5)
#   define  ETH_MAC1_UNUSED1_SHIFT          5

    /** 
     * Setting this bit will put the Transmit Function logic in reset.
     */ 
#   define  ETH_MAC1_RESET_TX_MASK          BIT(8)

    /** 
     * Setting this bit resets the MAC Control Sublayer / Transmit logic. The MCS logic
     * implements flow control.
     */ 
#   define  ETH_MAC1_RESET_MCS_TX_MASK      BIT(9)

    /** 
     * Setting this bit will put the Ethernet receive logic in reset. 
     */ 
#   define  ETH_MAC1_RESET_RX_MASK          BIT(10)

    /** 
     * Setting this bit resets the MAC Control Sublayer / Receive logic. The MCS logic
     * implements flow control.
     */ 
#   define  ETH_MAC1_RESET_MCS_RX_MASK      BIT(11)

    /** 
     * Reserved. User software should not write ones to reserved bits. The value read
     * from a reserved bit is not defined.
     */ 
#   define  ETH_MAC1_RESERVED1_MASK         MULTI_BIT_MASK(13, 12)
#   define  ETH_MAC1_RESERVED1_SHIFT        12

    /** 
     * Setting this bit will cause a reset to the random number generator within the
     * Transmit Function.
     */ 
#   define  ETH_MAC1_SIMULATION_RESET_MASK  BIT(14)

    /** 
     * Setting this bit will put all modules within the MAC in reset except the Host
     * Interface.
     */ 
#   define  ETH_MAC1_SOFT_RESET_MASK        BIT(15)

    /** 
     * Reserved.
     */ 
#   define  ETH_MAC1_RESERVED2_MASK         MULTI_BIT_MASK(31, 16)
#   define  ETH_MAC1_RESERVED2_SHIFT        16

    /**
     * R/W MAC configuration register 2. 
     */
    uint32_t reg_MAC2;

    /** 
     * When enabled (set to ’1’), the MAC operates in Full-Duplex mode. When disabled,
     * the MAC operates in Half-Duplex mode.
     */ 
#   define  ETH_MAC2_FULL_DUPLEX_MASK           BIT(0)

    /** 
     * When enabled (set to ’1’), both transmit and receive frame lengths are compared to
     * the Length/Type field. If the Length/Type field represents a length then the check is
     * performed. Mismatches are reported in the StatusInfo word for each received frame.
     */ 
#   define  ETH_MAC2_FRAME_LENGTH_CHECKING_MASK BIT(1)

    /** 
     * When enabled (set to ’1’), frames of any length are transmitted and received. 
     */ 
#   define  ETH_MAC2_HUGE_FRAME_ENABLE_MASK BIT(2)

    /** 
     * This bit determines the number of bytes, if any, of proprietary header information
     * that exist on the front of IEEE 802.3 frames. When 1, four bytes of header (ignored
     * by the CRC function) are added. When 0, there is no proprietary header.
     */ 
#   define  ETH_MAC2_DELAYED_CRC_MASK       BIT(3)

    /** 
     * Set this bit to append a CRC to every frame whether padding was required or not.
     * Must be set if PAD/CRC ENABLE is set. Clear this bit if frames presented to the
     * MAC contain a CRC.
     */ 
#   define  ETH_MAC2_CRC_ENABLE_MASK        BIT(4)

    /** 
     * Set this bit to have the MAC pad all short frames. Clear this bit if frames presented
     * to the MAC have a valid length. This bit is used in conjunction with AUTO PAD
     * ENABLE and VLAN PAD ENABLE, as shown below:
     * 
     * MAC2[7] MAC2[6] MAC2[5] Action
     * x       x       0       No pad or CRC check
     * 0       0       1       Pad to 60 bytes, append CRC
     * x       1       1       Pad to 64 bytes, append CRC
     * 1       0       1       If untagged, pad to 60 bytes and append CRC. If VLAN tagged:
     *                         pad to 64 bytes and append CRC.
     */ 
#   define  ETH_MAC2_PAD_CRC_ENABLE_MASK    BIT(5)

    /** 
     * Set this bit to cause the MAC to pad all short frames to 64 bytes and append a valid
     * CRC.
     * Note: This bit is ignored if PAD / CRC ENABLE is cleared.
     */ 
#   define  ETH_MAC2_VLAN_PAD_ENABLE_MASK    BIT(6)

    /** 
     * Set this bit to cause the MAC to automatically detect the type of frame, either tagged
     * or un-tagged, by comparing the two octets following the source address with
     * 0x8100 (VLAN Protocol ID) and pad accordingly.
     * Note: This bit is ignored if PAD / CRC ENABLE is cleared.
     */ 
#   define  ETH_MAC2_AUTO_DETECT_PAD_MASK    BIT(7)

    /** 
     * When enabled (set to ’1’), the MAC will verify the content of the preamble to ensure
     * it contains 0x55 and is error-free. A packet with an incorrect preamble is discarded.
     * When disabled, no preamble checking is performed.
     */ 
#   define  ETH_MAC2_PURE_PREAMBLE_ENFORCEMENT_MASK    BIT(8)

    /** 
     * When enabled (set to ’1’), the MAC only allows receive packets which contain
     * preamble fields less than 12 bytes in length. When disabled, the MAC allows any
     * length preamble as per the Standard.
     */ 
#   define  ETH_MAC2_LONG_PREAMBLE_ENFORCEMENT_MASK    BIT(9)

    /** 
     * Reserved. 
     */ 
#   define  ETH_MAC2_RESERVED1_MASK         MULTI_BIT_MASK(11, 10)
#   define  ETH_MAC2_RESERVED1_SHIFT        10

    /** 
     * When enabled (set to ’1’), the MAC will immediately retransmit following a collision
     * rather than using the Binary Exponential Backoff algorithm as specified in the
     * Standard.
     */ 
#   define  ETH_MAC2_NO_BACKOFF_MASK    BIT(12)

    /** 
     * When enabled (set to ’1’), after the MAC incidentally causes a collision during back
     * pressure, it will immediately retransmit without backoff, reducing the chance of
     * further collisions and ensuring transmit packets get sent.
     */ 
#   define  ETH_MAC2_BACK_PRESSURE_NO_BACKOFF_MASK    BIT(13)

    /** 
     * When enabled (set to ’1’) the MAC will defer to carrier indefinitely as per the
     * Standard. When disabled, the MAC will abort when the excessive deferral limit is
     * reached.
     */ 
#   define  ETH_MAC2_EXCESS_DEFER_MASK    BIT(14)

    /** 
     * Reserved. 
     */ 
#   define  ETH_MAC2_RESERVED2_MASK         MULTI_BIT_MASK(31, 15)
#   define  ETH_MAC2_RESERVED2_SHIFT        15

    /**
     * R/W Back-to-Back Inter-Packet-Gap register.
     */ 
    uint32_t reg_IPGT;

    /** 
     * This is a programmable field representing the nibble time offset of the minimum
     * possible period between the end of any transmitted packet to the beginning of the
     * next. In Full-Duplex mode, the register value should be the desired period in
     * nibble times minus 3. In Half-Duplex mode, the register value should be the
     * desired period in nibble times minus 6. In Full-Duplex the recommended setting is
     * 0x15 (21d), which represents the minimum IPG of 960 ns (in 100 Mbps mode) or
     * 9.6 us (in 10 Mbps mode). In Half-Duplex the recommended setting is 0x12 (18d),
     * which also represents the minimum IPG of 960 ns (in 100 Mbps mode) or 9.6 us
     * (in 10 Mbps mode).
     */ 
#   define  ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_MASK         MULTI_BIT_MASK(6, 0)
#   define  ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_SHIFT        0
#   define  ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_FULL_VALUE   0x15
#   define  ETH_IPGT_BACK_TO_BACK_INTER_PACKET_GAP_HALF_VALUE   0x12
    
    /** 
     * Reserved. 
     */ 
#   define  ETH_IPGT_RESERVED1_MASK         MULTI_BIT_MASK(31, 7)
#   define  ETH_IPGT_RESERVED1_SHIFT        0

    /** 
     * R/W Non Back-to-Back Inter-Packet-Gap register.
     */ 
    uint32_t reg_IPGR;

    /** 
     * This is a programmable field representing the Non-Back-to-Back
     * Inter-Packet-Gap. The recommended value is 0x12 (18d), which
     * represents the minimum IPG of 960 ns (in 100 Mbps mode) or 9.6 us (in
     * 10 Mbps mode).
     */ 
#   define  ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART2_MASK         MULTI_BIT_MASK(6, 0)
#   define  ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART2_SHIFT        0
#   define  ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART2_VALUE        0x12

    /** 
     * Reserved. 
     */ 
#   define  ETH_IPGR_RESERVED1_MASK           BIT(7)

    /** 
     * This is a programmable field representing the optional carrierSense
     * window referenced in IEEE 802.3/4.2.3.2.1 'Carrier Deference'. If carrier is
     * detected during the timing of IPGR1, the MAC defers to carrier. If,
     * however, carrier becomes active after IPGR1, the MAC continues timing
     * IPGR2 and transmits, knowingly causing a collision, thus ensuring fair
     * access to medium. Its range of values is 0x0 to IPGR2. The recommended
     * value is 0xC (12d)
     */ 
#   define  ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART1_MASK         MULTI_BIT_MASK(14, 8)
#   define  ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART1_SHIFT        8
#   define  ETH_IPGR_NON_BACK_TO_BACK_INTER_PACKET_GAP_PART1_VALUE        0xC

    /** 
     * Reserved. 
     */ 
#   define  ETH_IPGR_RESERVED2_MASK         MULTI_BIT_MASK(31, 15)
#   define  ETH_IPGR_RESERVED2_SHIFT        15

    /** 
     * R/W Collision window / Retry register.
     */ 
    uint32_t reg_CLRT;

    /** 
     * This is a programmable field specifying the number of retransmission attempts
     * following a collision before aborting the packet due to excessive collisions. The
     * Standard specifies the attemptLimit to be 0xF (15d). See IEEE 802.3/4.2.3.2.5.
     */
#   define  ETH_CLRT_RETRANSMISSION_MAXIMUM_MASK         MULTI_BIT_MASK(3, 0)
#   define  ETH_CLRT_RETRANSMISSION_MAXIMUM_SHIFT        0
#   define  ETH_CLRT_RETRANSMISSION_MAXIMUM_VALUE        0xF

    /** 
     * Reserved. 
     */ 
#   define  ETH_CLRT_RESERVED1_MASK         MULTI_BIT_MASK(7, 4)
#   define  ETH_CLRT_RESERVED1_SHIFT        4

    /** 
     * This is a programmable field representing the slot time or collision window during
     * which collisions occur in properly configured networks. The default value of 0x37
     * (55d) represents a 56 byte window following the preamble and SFD.
     */
#   define  ETH_CLRT_COLLISION_WINDOW_MASK  MULTI_BIT_MASK(13, 8)
#   define  ETH_CLRT_COLLISION_WINDOW_SHIFT 8
#   define  ETH_CLRT_COLLISION_WINDOW_VALUE 0x37

    /** 
     * Reserved. 
     */ 
#   define  ETH_CLRT_RESERVED2_MASK         MULTI_BIT_MASK(31, 14)
#   define  ETH_CLRT_RESERVED2_SHIFT        14

    /** 
     * R/W Maximum Frame register.
     */ 
    uint32_t reg_MAXF;

    /** 
     * This field resets to the value 0x0600, which represents a maximum receive frame of
     * 1536 octets. An untagged maximum size Ethernet frame is 1518 octets. A tagged
     * frame adds four octets for a total of 1522 octets. If a shorter maximum length
     * restriction is desired, program this 16 bit field.
     */ 
#   define  ETH_MAXF_MAXIMUM_FRAME_LENGTH_MASK  MULTI_BIT_MASK(15, 0)
#   define  ETH_MAXF_MAXIMUM_FRAME_LENGTH_SHIFT 0

    /** 
     * Reserved. 
     */ 
#   define  ETH_MAXF_RESERVED_MASK              MULTI_BIT_MASK(31, 16)
#   define  ETH_MAXF_RESERVED_SHIFT             16

    /** 
     * R/W PHY Support register.
     */ 
     uint32_t reg_SUPP;

     /** 
      * Reserved. 
      */ 
#   define  ETH_SUPP_RESERVED1_MASK         MULTI_BIT_MASK(7, 0)
#   define  ETH_SUPP_RESERVED1_SHIFT        0

     /**
      * This bit configures the Reduced MII logic for the current operating speed. When set,
      * 100 Mbps mode is selected. When cleared, 10 Mbps mode is selected.
      */ 
#   define  ETH_SUPP_SPEED_MASK             BIT(8)

     /** 
      * Reserved. 
      */ 
#   define  ETH_SUPP_RESERVED2_MASK         MULTI_BIT_MASK(31, 9)
#   define  ETH_SUPP_RESERVED2_SHIFT        9

    /** 
     * R/W Test register.
     */ 
     uint32_t reg_TEST;

    /** 
     * This bit reduces the effective PAUSE quanta from 64 byte-times to 1 byte-time.
     */ 
#   define  ETH_TEST_SHORTCUT_PAUSE_QUANTA_MASK    BIT(0)

    /** 
     * This bit causes the MAC Control sublayer to inhibit transmissions, just as if a
     * PAUSE Receive Control frame with a nonzero pause time parameter was received.
     */ 
#   define  ETH_TEST_PAUSE_MASK         BIT(1)

    /** 
     * Setting this bit will cause the MAC to assert backpressure on the link. Backpressure
     * causes preamble to be transmitted, raising carrier sense. A transmit packet from the
     * system will be sent during backpressure.
     */ 
#   define  ETH_TEST_BACKPRESSURE_MASK  BIT(2)

    /** 
     * Reserved.
     */ 
#   define  ETH_TEST_RESERVED_MASK      MULTI_BIT_MASK(31, 3)
#   define  ETH_TEST_RESERVED_SHIFT     3

     
    /** 
     * R/W MII Mgmt Configuration register.
     */ 
    uint32_t reg_MCFG;

    /** 
     * Set this bit to cause the MII Management hardware to perform read cycles across a
     * range of PHYs. When set, the MII Management hardware will perform read cycles
     * from address 1 through the value set in PHY ADDRESS[4:0]. Clear this bit to allow
     * continuous reads of the same PHY.
     */ 
#   define  ETH_MCFG_SCAN_INCREMENT_MASK    BIT(0)

    /** 
     * Set this bit to cause the MII Management hardware to perform read/write cycles
     * without the 32 bit preamble field. Clear this bit to cause normal cycles to be
     * performed. Some PHYs support suppressed preamble.
     */ 
#   define  ETH_MCFG_SUPPRESS_PREAMBLE_MASK BIT(1)

    /** 
     * This field is used by the clock divide logic in creating the MII Management Clock
     * (MDC) which IEEE 802.3u defines to be no faster than 2.5 MHz. Some PHYs
     * support clock rates up to 12.5 MHz, however. Values for this field are
     * from enum eth_mcfg_clock_select.
     */ 
#   define  ETH_MCFG_CLOCK_SELECT_MASK      MULTI_BIT_MASK(4, 2)
#   define  ETH_MCFG_CLOCK_SELECT_SHIFT     2

    /** 
     * Reserved. 
     */ 
#   define  ETH_MCFG_RESERVED1_MASK         MULTI_BIT_MASK(14, 5)
#   define  ETH_MCFG_RESERVED1_SHIFT        5

    /** 
     * This bit resets the MII Management hardware.
     */ 
#   define  ETH_MCFG_RESET_MII_MGMT_MASK    BIT(15)

    /** 
     * Reserved. 
     */ 
#   define  ETH_MCFG_RESERVED2_MASK         MULTI_BIT_MASK(31, 16)
#   define  ETH_MCFG_RESERVED2_SHIFT        16
    
    /** 
     * R/W MII Mgmt Command register.
     */ 
    uint32_t reg_MCMD;

    /** 
     * This bit causes the MII Management hardware to perform a single Read cycle.
     * The Read data is returned in Register MRDD (MII Mgmt Read Data).
     */ 
#   define  ETH_MCMD_READ_MASK      BIT(0)

    /** 
     * This bit causes the MII Management hardware to perform Read cycles continuously. This is
     * useful for monitoring Link Fail for example.
     */ 
#   define  ETH_MCMD_SCAN_MASK      BIT(1)

    /** 
     * Reserved. 
     */ 
#   define  ETH_MCMD_RESERVED_MASK  MULTI_BIT_MASK(31, 2)
#   define  ETH_MCMD_RESERVED_SHIFT 2

    /** 
     * R/W MII Mgmt Address register.
     */ 
    uint32_t reg_MADR;

    /** 
     * This field represents the 5 bit Register Address of a MII register.
     * Values for this field are from enum eth_mii_register_indexes.
     */ 
#   define  ETH_MADR_REGISTER_ADDRESS_MASK         MULTI_BIT_MASK(4, 0)
#   define  ETH_MADR_REGISTER_ADDRESS_SHIFT        0

    /** 
     * Reserved. 
     */ 
#   define  ETH_MADR_RESERVED1_MASK         MULTI_BIT_MASK(7, 5)
#   define  ETH_MADR_RESERVED1_SHIFT        5

    /** 
     * This field represents the 5 bit PHY Address field of Mgmt
     * cycles. Up to 31 PHYs can be addressed (0 is reserved).
     */ 
#   define  ETH_MADR_PHY_ADDRESS_MASK         MULTI_BIT_MASK(12, 8)
#   define  ETH_MADR_PHY_ADDRESS_SHIFT        8
#   define  ETH_MADR_MIN_PHY_ADDRESS          0x1
#   define  ETH_MADR_MAX_PHY_ADDRESS          0x1f

    /** 
     * Reserved. 
     */ 
#   define  ETH_MADR_RESERVED2_MASK         MULTI_BIT_MASK(31, 13)
#   define  ETH_MADR_RESERVED2_SHIFT        13
    
    /** 
     * WO MII Mgmt Write Data register.
     */ 
    uint32_t reg_MWTD;

    /** 
     * When written, an MII Mgmt write cycle is performed using the 16 bit
     * data and the pre-configured PHY and Register addresses from the
     * MII Mgmt Address register (MADR).
     */ 
#   define  ETH_MWTD_WRITE_DATA_MASK         MULTI_BIT_MASK(15, 0)
#   define  ETH_MWTD_WRITE_DATA_SHIFT        0

    /** 
     * Reserved. 
     */ 
#   define  ETH_MWTD_RESERVED_MASK          MULTI_BIT_MASK(31, 16)
#   define  ETH_MWTD_RESERVED_SHIFT         16

    /** 
     * RO MII Mgmt Read Data register.
     */ 
    uint32_t reg_MRDD;

    /** 
     * Following an MII Mgmt Read Cycle, the 16 bit data can be read from
     * this location.
     */ 
#   define  ETH_MRDD_READ_DATA_MASK         MULTI_BIT_MASK(15, 0)
#   define  ETH_MRDD_READ_DATA_SHIFT        0

    /** 
     * Reserved. 
     */ 
#   define  ETH_MRDD_RESERVED_MASK          MULTI_BIT_MASK(31, 16)
#   define  ETH_MRDD_RESERVED_SHIFT         16

    /** 
     * RO MII Mgmt Indicators register.
     */ 
    uint32_t reg_MIND;
    /** 
     * When ’1’ is returned - indicates MII Mgmt is currently performing an
     * MII Mgmt Read or Write cycle.
     */ 
#   define  ETH_MIND_BUSY_MASK          BIT(0)

    /** 
     * When ’1’ is returned - indicates a scan operation (continuous MII
     * Mgmt Read cycles) is in progress.
     */ 
#   define  ETH_MIND_SCANNING_MASK      BIT(1)

    /** 
     * When ’1’ is returned - indicates MII Mgmt Read cycle has not
     * completed and the Read Data is not yet valid.
     */ 
#   define  ETH_MIND_NOT_VALID_MASK     BIT(2)

    /** 
     * When ’1’ is returned - indicates that an MII Mgmt link fail has
     * occurred.
     */ 
#   define  ETH_MIND_MII_LINK_FAIL_MASK BIT(3)

    /** 
     * Reserved. 
     */ 
#   define  ETH_MIND_RESERVED_MASK      MULTI_BIT_MASK(31, 4)
#   define  ETH_MIND_RESERVED_SHIFT     4

    /**
     * Reserved, user software should not write ones to
     * reserved bits. The value read from a reserved bit
     * is not defined.
     */
    uint32_t reg_reserved1[0x2];

    /** 
     * R/W Station Address registers
     *
     * reg_SA[0] = (2nd octet, 1st octet)
     * reg_SA[1] = (4th octet, 3rd octet)
     * reg_SA[2] = (6th octet, 5th octet)
     */ 
    uint32_t reg_SA[3];

    /** 
     * Most significant octet of pair of octets. 
     */ 
#   define  ETH_SA_HIGH_OCTET_MASK         MULTI_BIT_MASK(7, 0)
#   define  ETH_SA_HIGH_OCTET_SHIFT        0

    /** 
     * Least significant octet of pair of octets. 
     */ 
#   define  ETH_SA_LOW_OCTET_MASK         MULTI_BIT_MASK(15, 8)
#   define  ETH_SA_LOW_OCTET_SHIFT        8

    /** 
     * Reserved. 
     */ 
#   define  ETH_SA_RESERVED_MASK         MULTI_BIT_MASK(31, 16)
#   define  ETH_SA_RESERVED_SHIFT        16

    /** 
     * Reserved
     */ 
    uint32_t reg_reserved2[0x2D];

    /* 
     * Control registers
     */ 

    /** 
     * R/W Command register.
     */ 
    uint32_t reg_Command;

    /** 
     * Enable receive.
     */ 
#   define  ETH_CMD_RX_ENABLE_MASK  BIT(0)

    /** 
     * Enable transmit.
     */ 
#   define  ETH_CMD_TX_ENABLE_MASK  BIT(1)

    /** 
     * Reserved. 
     */ 
#   define  ETH_CMD_RESERVED1_MASK  BIT(2)

    /** 
    * When a ’1’ is written, all datapaths and the host registers are
    * reset. The MAC needs to be reset separately.
    */ 
#   define  ETH_CMD_REG_RESET_MASK  BIT(3)

    /** 
     * When a ’1’ is written, the transmit datapath is reset. 
     */ 
#   define  ETH_CMD_TX_RESET_MASK   BIT(4)

    /** 
     * When a ’1’ is written, the receive datapath is reset. 
     */ 
#   define  ETH_CMD_RX_RESET_MASK   BIT(5)
    
    /** 
     * When set to ’1’, passes runt frames smaller than 64 bytes to
     * memory unless they have a CRC error. If ’0’ runt frames are
     * filtered out.
     */ 
#   define  ETH_CMD_PASS_RUNT_FRAME_MASK    BIT(6)

    /** 
     * When set to ’1’, disables receive filtering i.e. all frames
     * received are written to memory.
     */
#   define  ETH_CMD_PASS_RX_FILTER_MASK     BIT(7)

    /** 
     * Enable IEEE 802.3 / clause 31 flow control sending pause
     * frames in full duplex and continuous preamble in half duplex.
     */ 
#   define  ETH_CMD_TX_FLOW_CONTROL_MASK    BIT(8)

    /** 
     * When set to ’1’, RMII mode is selected; if ’0’, MII mode is
     * selected.
     */ 
#   define  ETH_CMD_RMII_MASK               BIT(9)

    /** 
     * When set to ’1’, indicates full duplex operation.
     */ 
#   define  ETH_CMD_FULL_DUPLEX_MASK        BIT(10)
    
    /** 
     * Reserved. 
     */ 
#   define  ETH_CMD_RESERVED2_MASK          MULTI_BIT_MASK(31, 11)
#   define  ETH_CMD_RESERVED2_SHIFT         11

    /** 
     * RO Status register.
     */ 
    uint32_t reg_Status;

    /** 
     * If 1, the receive channel is active. If 0, the receive channel is inactive.
     */ 
#   define  ETH_STS_RX_STATUS_MASK       BIT(0)

    /** 
     * If 1, the transmit channel is active. If 0, the transmit channel is inactive.
     */ 
#   define  ETH_STS_TX_STATUS_MASK       BIT(1)

    /** 
     * Reserved. 
     */ 
#   define  ETH_STS_RESERVED_MASK        MULTI_BIT_MASK(31, 2)
#   define  ETH_STS_RESERVED_SHIFT       2

    /** 
     * R/W Receive descriptor base address register.
     */ 
    uint32_t reg_RxDescriptor;

    /** 
     * Fixed to ’00’, since the Rx descriptor base address is
     * 4-byte aligned aligned. 
     */ 
#   define  ETH_RXDES_RESERVED_MASK     MULTI_BIT_MASK(1, 0)
#   define  ETH_RXDES_RESERVED_SHIFT    0

    /** 
     * MSBs of receive descriptor base address.
     */ 
#   define  ETH_RXDES_RX_DESCRIPTOR_MASK    MULTI_BIT_MASK(31, 2)
#   define  ETH_RXDES_RX_DESCRIPTOR_SHIFT   2
    
    /** 
     * R/W Receive status base address register.
     */ 
    uint32_t reg_RxStatus;

    /** 
     * Fixed to ’000’, since the Rx status base address is
     * 8-byte aligned. 
     */ 
#   define  ETH_RXSTS_RESERVED_MASK     MULTI_BIT_MASK(2, 0)
#   define  ETH_RXSTS_RESERVED_SHIFT    0

    /** 
     * MSBs of receive status base address.
     */ 
#   define  ETH_RXSTS_RX_STATUS_MASK    MULTI_BIT_MASK(31, 3)
#   define  ETH_RXSTS_RX_STATUS_SHIFT   3

    /** 
     * R/W Receive number of descriptors register.
     */ 
    uint32_t reg_RxDescriptorNumber;

    /** 
     * Number of descriptors in the descriptor array for which
     * RxDescriptor is the base address. The number of
     * descriptors is minus one encoded.
     */ 
#   define  ETH_RXDN_RX_DESCRIPTOR_NUMBER_MASK         MULTI_BIT_MASK(15, 0)
#   define  ETH_RXDN_RX_DESCRIPTOR_NUMBER_SHIFT        0

    /** 
     * Reserved. 
     */ 
#   define  ETH_MWTD_RESERVED_MASK          MULTI_BIT_MASK(31, 16)
#   define  ETH_MWTD_RESERVED_SHIFT         16
    
    /** 
     * RO Receive produce index register.
     */ 
    uint32_t reg_RxProduceIndex;

    /** 
     * Index of the descriptor that is going to be filled next by the
     * receive datapath.
     */ 
#   define  ETH_RXPI_RX_PRODUCE_INDEX_MASK  MULTI_BIT_MASK(15, 0)
#   define  ETH_RXPI_RX_PRODUCE_INDEX_SHIFT 0

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXPI_RESERVED_MASK          MULTI_BIT_MASK(31, 16)
#   define  ETH_RXPI_RESERVED_SHIFT         16

    /** 
     * R/W Receive consume index register.
     */ 
    uint32_t reg_RxConsumeIndex;

    /** 
     * Index of the descriptor that is going to be processed next by
     * the receive
     */ 
#   define  ETH_RXCI_RX_CONSUME_INDEX_MASK  MULTI_BIT_MASK(15, 0)
#   define  ETH_RXCI_RX_CONSUME_INDEX_SHIFT 0
    
    /** 
     * Reserved. 
     */ 
#   define  ETH_RXCI_RESERVED_MASK          MULTI_BIT_MASK(31, 16)
#   define  ETH_RXCI_RESERVED_SHIFT         16

    /** 
     * R/W Transmit descriptor base address register.
     */ 
    uint32_t reg_TxDescriptor;

    /** 
     * Fixed to ’00’, since the Tx descriptor base address is
     * 4-byte aligned aligned. 
     */ 
#   define  ETH_TXDES_RESERVED_MASK     MULTI_BIT_MASK(1, 0)
#   define  ETH_TXDES_RESERVED_SHIFT    0

    /** 
     * MSBs of transmit descriptor base address.
     */ 
#   define  ETH_TXDES_TX_DESCRIPTOR_MASK    MULTI_BIT_MASK(31, 2)
#   define  ETH_TXDES_TX_DESCRIPTOR_SHIFT   2

    /** 
     * R/W Transmit status base address register.
     */ 
    uint32_t reg_TxStatus;

    /** 
     * Fixed to ’00’, since the Tx status base address is
     * 4-byte aligned. 
     */ 
#   define  ETH_TXSTS_RESERVED_MASK     MULTI_BIT_MASK(1, 0)
#   define  ETH_TXSTS_RESERVED_SHIFT    0

    /** 
     * MSBs of transmit status base address.
     */ 
#   define  ETH_TXSTS_TX_STATUS_MASK    MULTI_BIT_MASK(31, 2)
#   define  ETH_TXSTS_TX_STATUS_SHIFT   2

    /** 
     * R/W Transmit number of descriptors register.
     */ 
    uint32_t reg_TxDescriptorNumber;

    /** 
     * Number of descriptors in the descriptor array for which
     * TxDescriptor is the base address. The register is minus
     * one encoded.
     */ 
#   define  ETH_TXDN_TX_DESCRIPTOR_NUMBER_MASK  MULTI_BIT_MASK(15, 0)
#   define  ETH_TXDN_TX_DESCRIPTOR_NUMBER_SHIFT 0
    
    /** 
     * Reserved. 
     */ 
#   define  ETH_TXDN_RESERVED_MASK              MULTI_BIT_MASK(31, 16)
#   define  ETH_TXDN_RESERVED_SHIFT             16

    /** 
     * R/W Transmit produce index register.
     */ 
    uint32_t reg_TxProduceIndex;

    /** 
     * Index of the descriptor that is going to be filled next by the
     * transmit software driver.
     */ 
#   define  ETH_TXPI_TX_PRODUCE_INDEX_MASK  MULTI_BIT_MASK(15, 0)
#   define  ETH_TXPI_TX_PRODUCE_INDEX_SHIFT 0
    
    /** 
     * Reserved. 
     */ 
#   define  ETH_TXPI_RESERVED_MASK          MULTI_BIT_MASK(31, 16)
#   define  ETH_TXPI_RESERVED_SHIFT         16

    /** 
     * RO Transmit consume index register.
     */ 
    uint32_t reg_TxConsumeIndex;

    /** 
     * Index of the descriptor that is going to be transmitted next by
     * the transmit datapath
     */ 
#   define  ETH_TXCI_TX_CONSUME_INDEX_MASK  MULTI_BIT_MASK(15, 0)
#   define  ETH_TXCI_TX_CONSUME_INDEX_SHIFT 0
    
    /** 
     * Reserved. 
     */ 
#   define  ETH_TXCI_RESERVED_MASK          MULTI_BIT_MASK(31, 16)
#   define  ETH_TXCI_RESERVED_SHIFT         16

    /** 
     * Reserved
     */ 
    uint32_t reg_reserved3[0xA];

    /** 
     * RO Transmit status vector register 0 (TSV first part).
     */ 
    uint32_t reg_TSV0;

    /** 
     * The attached CRC in the packet did not match the
     * internally generated CRC.
     */ 
#   define  ETH_TSV0_CRC_ERROR_MASK             BIT(0)

    /** 
     * Indicates the frame length field does not match the actual
     * number of data items and is not a type field.
     */ 
#   define  ETH_TSV0_LENGTH_CHECK_ERROR_MASK    BIT(1)

    /** 
     * range Indicates that frame type/length field was larger than
     * 1500 bytes.
     *
     * NOTE: The EMAC doesn't distinguish the frame type and frame length,
     * so, e.g. when the IP(0x8000) or ARP(0x0806) packets are received, it
     * compares the frame type with the max length and gives the "Length
     * out of range" error. In fact, this bit is not an error indication,
     * but simply a statement by the chip regarding the status of the received
     * frame.
     */ 
#   define  ETH_TSV0_LENGTH_OUT_OF_RANGE_MASK   BIT(2)

    /** 
     * Transmission of packet was completed. 
     */ 
#   define  ETH_TSV0_DONE_MASK                  BIT(3)

    /** 
     * Packet’s destination was a multicast address.
     */ 
#   define  ETH_TSV0_MULTICAST_MASK             BIT(4)

    /** 
     * Packet’s destination was a broadcast address.
     */ 
#   define  ETH_TSV0_BROADCAST_MASK             BIT(5)

    /** 
     * Packet was deferred for at least one attempt, but less than
     * an excessive defer.
     */ 
#   define  ETH_TSV0_PACKET_DEFER_MASK          BIT(6)

    /** 
     * Packet was deferred in excess of 6071 nibble times in
     * 100 Mbps or 24287 bit times in 10 Mbps mode.
     */ 
#   define  ETH_TSV0_EXCESSIVE_DEFER_MASK       BIT(7)

    /** 
     * Packet was aborted due to exceeding of maximum allowed
     * number of collisions.
     */ 
#   define  ETH_TSV0_EXCESSIVE_COLLISION_MASK   BIT(8)

    /** 
     * Collision occurred beyond collision window, 512 bit times.
     */ 
#   define  ETH_TSV0_LATE_COLLISION_MASK        BIT(9)
    
    /** 
     * Byte count in frame was greater than can be represented
     * in the transmit byte count field in TSV1.
     */ 
#   define  ETH_TSV0_GIANT_MASK                 BIT(10)

    /** 
     * Host side caused buffer underrun.
     */ 
#   define  ETH_TSV0_UNDERRUN_MASK              BIT(11)
    /** 
     * The total number of bytes transferred including collided
     * attempts.
     */ 
#   define  ETH_TSV0_TOTAL_BYTES_MASK           MULTI_BIT_MASK(27, 12)
#   define  ETH_TSV0_TOTAL_BYTES_SHIFT          12 

    /** 
     * The frame was a control frame. 
     */ 
#   define  ETH_TSV0_CONTROL_FRAME_MASK         BIT(28)

    /** 
     * The frame was a control frame with a valid PAUSE
     * opcode.
     */ 
#   define  ETH_TSV0_PAUSE_MASK                 BIT(29)

    /** 
     * Carrier-sense method backpressure was previously
     * applied.
     */ 
#   define  ETH_TSV0_BACKPRESSURE_MASK          BIT(30)

    /** 
     * Frame’s length/type field contained 0x8100 which is the
     * VLAN protocol identifier.
     */ 
#   define  ETH_TSV0_VLAN_MASK                  BIT(31)
    
    /** 
     * RO Transmit status vector register 1 (TSV second part).
     */ 
    uint32_t reg_TSV1;

    /** 
     * The total number of bytes in the frame, not counting the
     * collided bytes.
     */ 
#   define  ETH_TSV1_TRANSMIT_BYTE_COUNT_MASK   MULTI_BIT_MASK(15, 0)
#   define  ETH_TSV1_TRANSMIT_BYTE_COUNT_SHIFT  0

    /** 
     * Number of collisions the current packet incurred during
     * transmission attempts. The maximum number of collisions
     * (16) cannot be represented.
     */ 
#   define  ETH_TSV1_TRANSMIT_COLLISION_COUNT_MASK  MULTI_BIT_MASK(19, 16)
#   define  ETH_TSV1_TRANSMIT_COLLISION_COUNT_SHIFT 16 

    /** 
     * Reserved. 
     */ 
#   define  ETH_TSV1_RESERVED_MASK                  MULTI_BIT_MASK(31, 20)
#   define  ETH_TSV1_RESERVED_SHIFT                 20

    /** 
     * RO Receive status vector register.
     */ 
    uint32_t reg_RSV;

    /** 
     * Indicates length of received frame.
     */ 
#   define  ETH_RSV_RECEIVED_BYTE_COUNT_MASK         MULTI_BIT_MASK(15, 0)
#   define  ETH_RSV_RECEIVED_BYTE_COUNT_SHIFT        0

    /** 
     * Indicates that a packet was dropped.
     */ 
#   define  ETH_RSV_PACKET_PREVIOUSLY_IGNORED_MASK  BIT(16)

    /** 
    * Indicates that the last receive event seen was not long
    * enough to be a valid packet.
    */ 
#   define  ETH_RSV_RXDV_EVENT_PREVIOUSLY_SEEN_MASK BIT(17)

    /** 
     * Indicates that at some time since the last receive statistics,
     * a carrier event was detected.
     */ 
#   define  ETH_RSV_CARRIER_EVENT_PREVIOUSLY_SEEN_MASK  BIT(18)

    /** 
     * Indicates that MII data does not represent a valid receive
     * code.
     */ 
#   define  ETH_RSV_RECEIVE_CODE_VIOLATION_MASK         BIT(19)

    /** 
     * The attached CRC in the packet did not match the
     * internally generated CRC.
     */ 
#   define  ETH_RSV_CRC_ERROR_MASK                      BIT(20)

    /** 
     * Indicates the frame length field does not match the actual
     * number of data items and is not a type field.
     */ 
#   define  ETH_RSV_LENGTH_CHECK_ERROR_MASK             BIT(21)

    /** 
     * Indicates that frame type/length field was larger than
     * 1518 bytes.
     */ 
#   define  ETH_RSV_LENGTH_OUT_OF_RANGE_MASK            BIT(22)

    /** 
     * The packet had valid CRC and no symbol errors.
     */ 
#   define  ETH_RSV_RECEIVE_OK_MASK                     BIT(23)

    /** 
     * Multicast The packet destination was a multicast address. 
     */ 
#   define  ETH_RSV_MULTICAST_MASK                      BIT(24)

    /** 
     * The packet destination was a broadcast address.
     */ 
#   define  ETH_RSV_BROADCAST_MASK                      BIT(25)

    /** 
     * Indicates that after the end of packet another 1-7 bits were
     * received. A single nibble, called dribble nibble, is formed
     * but not sent out.
     */ 
#   define  ETH_RSV_DRIBBLE_NIBBLE_MASK                 BIT(26)

    /** 
     * The frame was a control frame.
     */ 
#   define  ETH_RSV_CONTROL_FRAME_MASK                  BIT(27)

    /** 
     * The frame was a control frame with a valid PAUSE
     * opcode.
     */ 
#   define  ETH_RSV_PAUSE_MASK                          BIT(28)

    /** 
     * The current frame was recognized as a Control Frame but
     * contains an unknown opcode.
     */ 
#   define  ETH_RSV_UNSUPPORTED_OPCODE_MASK             BIT(29)

    /** 
     * Frame’s length/type field contained 0x8100 which is the
     * VLAN protocol identifier.
     */ 
#   define  ETH_RSV_VLAN_MASK                           BIT(30)

    /** 
     * Reserved
     */ 
#   define  ETH_RSV_RESERVED_MASK                       BIT(31)
    
    /** 
     * Reserved
     */ 
    uint32_t reg_reserved4[0x3];

    /** 
     * R/W Flow control counter register.
     */ 
    uint32_t reg_FlowControlCounter;

    /** 
     * In full duplex mode the MirrorCounter specifies the number
     * of cycles before re-issuing the Pause control frame.
     */ 
#   define  ETH_FCC_MIRROR_COUNTER_MASK         MULTI_BIT_MASK(15, 0)
#   define  ETH_FCC_MIRROR_COUNTER_SHIFT        0

    /** 
     * In full-duplex mode the PauseTimer specifies the value
     * that is inserted into the pause timer field of a pause flow
     * control frame. In half duplex mode the PauseTimer
     * specifies the number of backpressure cycles.
     */ 
#   define  ETH_FCC_PAUSE_TIMER_MASK            MULTI_BIT_MASK(31, 16)
#   define  ETH_FCC_PAUSE_TIMER_SHIFT           16 
    
    /** 
     * RO Flow control status register.
     */ 
    uint32_t reg_FlowControlStatus;

    /** 
     * In full duplex mode this register represents the current
     * value of the datapath’s mirror counter which counts up to
     * the value specified by the MirrorCounter field in the
     * FlowControlCounter register. In half duplex mode the
     * register counts until it reaches the value of the PauseTimer
     * bits in the FlowControlCounter register.
     */ 
#   define  ETH_FCS_MIRROR_COUNTER_CURRENT_MASK     MULTI_BIT_MASK(15, 0)
#   define  ETH_FCS_MIRROR_COUNTER_CURRENT_SHIFT    0

    /** 
     * Reserved. 
     */ 
#   define  ETH_FCS_RESERVED_MASK                   MULTI_BIT_MASK(31, 16)
#   define  ETH_FCS_RESERVED_SHIFT                  16

    /** 
     * Reserved
     */ 
    uint32_t reg_reserved5[0x22];

    /* 
     * Rx filter registers
     */

    /** 
     * Receive filter control register.
     */ 
    uint32_t reg_RxFliterCtrl;

    /** 
     * When set to ’1’, all unicast frames are accepted.
     */ 
#   define  ETH_RXFC_ACCEPT_UNICAST_EN_MASK    BIT(0)

    /** 
     * When set to ’1’, all broadcast frames are accepted.
     */ 
#   define  ETH_RXFC_AcceptBroadcastEn_MASK    BIT(1)

    /** 
     * When set to ’1’, all multicast frames are accepted.
     */ 
#   define  ETH_RXFC_ACCEPT_MULTICAST_EN_MASK    BIT(2)

    /** 
     * When set to ’1’, unicast frames that pass the imperfect
     * hash filter are accepted.
     */ 
#   define  ETH_RXFC_ACCEPT_UNICAST_HASH_EN_MASK    BIT(3)

    /** 
     * When set to ’1’, multicast frames that pass the
     * imperfect hash filter are accepted.
     */ 
#   define  ETH_RXFC_ACCEPT_MULTICAST_HASH_EN_MASK  BIT(4)

    /** 
     * When set to ’1’, the frames with a destination address
     * identical to the station address are accepted.
     */ 
#   define  ETH_RXFC_ACCEPT_PERFECT_EN_MASK     BIT(5)

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXFC_RESERVED1_MASK             MULTI_BIT_MASK(11, 6)
#   define  ETH_RXFC_RESERVED1_SHIFT            6

    /** 
     * MagicPacketEnWoL When set to ’1’, the result of the magic packet filter will
     * generate a WoL interrupt when there is a match.
     */ 
#   define  ETH_RXFC_MAGIC_PACKET_EN_WOL_MASK   BIT(12)

    /** 
     * When set to ’1’, the result of the perfect address
     * matching filter and the imperfect hash filter will
     * generate a WoL interrupt when there is a match.
     */ 
#   define  ETH_RXFC_RX_FILTER_EN_WOL_MASK      BIT(13)

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXFC_RESERVED2_MASK             MULTI_BIT_MASK(31, 14)
#   define  ETH_RXFC_RESERVED2_SHIFT            14
    
    /** 
     * Receive filter WoL status register.
     */ 
    uint32_t reg_RxFilterWoLStatus;

    /** 
     * When the value is ’1’, a unicast frames caused WoL.
     */ 
#   define  ETH_RXFS_ACCEPT_UNICAST_WOL_MASK    BIT(0)

    /** 
     * When the value is ’1’, a broadcast frame caused WoL.
     */ 
#   define  ETH_RXFS_ACCEPT_BROADCAST_WOL_MASK  BIT(1)

    /** 
     * When the value is ’1’, a multicast frame caused WoL.
     */ 
#   define  ETH_RXFS_ACCEPT_MULTICAST_WOL_MASK  BIT(2)

    /** 
     * When the value is ’1’, a unicast frame that passes the
     * imperfect hash filter caused WoL.
     */ 
#   define  ETH_RXFS_ACCEPT_UNICAST_HASH_WOL_MASK   BIT(3)

    /** 
     * When the value is ’1’, a multicast frame that passes the
     * imperfect hash filter caused WoL.
     */ 
#   define  ETH_RXFS_ACCEPT_MULTICAST_HASH_WOL_MASK BIT(4)

    /** 
     * AcceptPerfectWoL When the value is ’1’, the perfect address matching filter
     * caused WoL.
     */ 
#   define  ETH_RXFS_ACCEPT_PERFECT_WOL_MASK    BIT(5)

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXFS_RESERVED1_MASK             BIT(6)

    /** 
     * When the value is ’1’, the receive filter caused WoL.
     */ 
#   define  ETH_RXFS_RX_FILTER_WOL_MASK         BIT(7)

    /** 
     * When the value is ’1’, the magic packet filter caused
     * WoL.
     */ 
#   define  ETH_RXFS_MAGIC_PACKET_WOL_MASK      BIT(8)

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXFS_RESERVED2_MASK         MULTI_BIT_MASK(31, 9)
#   define  ETH_RXFS_RESERVED2_SHIFT        9

    
    /** 
     * Receive filter WoL clear register.
     */ 
    uint32_t reg_RxFilterWoLClear;

    /** 
     * When a ’1’ is written to one of these bits (0 to 5), the
     * corresponding status bit in the RxFilterWoLStatus
     * register is cleared.
     */ 
#   define  ETH_RXFWC_ACCEPT_UNICAST_WOL_CLR_MASK           BIT(0)
#   define  ETH_RXFWC_ACCEPT_BROADCAST_WOL_CLR_MASK         BIT(1)
#   define  ETH_RXFWC_ACCEPT_MULTICAST_WOL_CLR_MASK         BIT(2)
#   define  ETH_RXFWC_Accept_Unicast_HASH_WOL_CLR_MASK      BIT(3)
#   define  ETH_RXFWC_ACCEPT_MULTICAST_HASH_WOL_CLR_MASK    BIT(4)

#   define  ETH_RXFWC_ACCEPT_PERFECT_WOL_CLR_MASK           BIT(5)

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXFWC_RESERVED1_MASK                        BIT(6)

    /** 
     * When a ’1’ is written to one of these bits (7 and/or 8),
     * the corresponding status bit in the RxFilterWoLStatus
     * register is cleared.
     */ 
#   define  ETH_RXFWC_RX_FILTER_WOL_CLR_MASK                BIT(7)
#   define  ETH_RXFWC_MAGIC_PACKET_WOL_CLR_MASK             BIT(8)

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXFWC_RESERVED2_MASK          MULTI_BIT_MASK(31, 9)
#   define  ETH_RXFWC_RESERVED2_SHIFT         9

    /** 
     * Reserved
     */ 
    uint32_t reg_reserved6;

    /** 
     * Hash filter table LSBs register (low part).
     * Bit 31:0 of the imperfect filter hash table for receive
     * filtering.
     */
    uint32_t reg_HashFilterL;

    /** 
     * Hash filter table MSBs register (high part).
     * Bit 63:32 of the imperfect filter hash table for receive
     * filtering.
     */ 
    uint32_t reg_HashFilterH;

    /** 
     * Reserved
     */ 
    uint32_t reg_reserved7[0x372];

    /* 
     * Module control registers
     */ 

    /** 
     * RO Interrupt status register.
     */ 
    uint32_t reg_IntStatus;

    /** 
     * Interrupt set on a fatal overrun error in the receive queue. The
     * fatal interrupt should be resolved by a Rx soft-reset. The bit is not
     * set when there is a nonfatal overrun error.
     */ 
#   define  ETH_INTST_RX_OVERRUN_INT_MASK    BIT(0)

    /** 
     * Interrupt trigger on receive errors: AlignmentError, RangeError,
     * LengthError, SymbolError, CRCError or NoDescriptor or Overrun.
     */ 
#   define  ETH_INTST_RX_ERROR_INT_MASK      BIT(1)

    /** 
     * Interrupt triggered when all receive descriptors have been
     * processed i.e. on the transition to the situation where
     * ProduceIndex == ConsumeIndex.
     */ 
#   define  ETH_INTST_RX_FINISHED_INT_MASK   BIT(2)

    /** 
     * Interrupt triggered when a receive descriptor has been processed
     * while the Interrupt bit in the Control field of the descriptor was set.
     */ 
#   define  ETH_INTST_RX_DONE_INT_MASK       BIT(3)

    /** 
     * Interrupt set on a fatal underrun error in the transmit queue. The
     * fatal interrupt should be resolved by a Tx soft-reset. The bit is not
     * set when there is a nonfatal underrun error.
     */ 
#   define  ETH_INTST_TX_UNDERRUN_INT_MASK   BIT(4)

    /** 
     * Interrupt trigger on transmit errors: LateCollision,
     * ExcessiveCollision and ExcessiveDefer, NoDescriptor or
     * Underrun.
     */ 
#   define  ETH_INTST_TX_ERROR_INT_MASK      BIT(5)

    /** 
     * Interrupt triggered when all transmit descriptors have been
     * processed i.e. on the transition to the situation where
     * ProduceIndex == ConsumeIndex.
     */ 
#   define  ETH_INTST_TX_FINISHED_INT_MASK   BIT(6)

    /** 
     * Interrupt triggered when a descriptor has been transmitted while
     * the Interrupt bit in the Control field of the descriptor was set.
     */ 
#   define  ETH_INTST_TX_DONE_INT_MASK       BIT(7)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTST_RESERVED1_MASK     MULTI_BIT_MASK(11, 8)
#   define  ETH_INTST_RESERVED1_SHIFT    8

    /** 
     * Interrupt triggered by software writing a 1 to the SoftintSet bit in
     * the IntSet register.
     */ 
#   define  ETH_INTST_SOFT_INT_MASK      BIT(12)

    /** 
     * Interrupt triggered by a Wakeup event detected by the receive
     * filter.
     */ 
#   define  ETH_INTST_WAKEUP_INT_MASK    BIT(13)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTST_RESERVED2_MASK     MULTI_BIT_MASK(31, 14)
#   define  ETH_INTST_RESERVED2_SHIFT    14

    /** 
     * R/W Interrupt enable register.
     */ 
    uint32_t reg_IntEnable;

    /** 
     * Enable for interrupt trigger on receive buffer overrun or
     * descriptor underrun situations.
     */ 
#   define  ETH_INTE_RX_OVERRUN_INT_EN_MASK    BIT(0)

    /** 
     * Enable for interrupt trigger on receive errors.
     */ 
#   define  ETH_INTE_RX_ERROR_INT_EN_MASK    BIT(1)

    /** 
     * Enable for interrupt triggered when all receive descriptors have
     * been processed i.e. on the transition to the situation where
     * ProduceIndex == ConsumeIndex.
     */ 
#   define  ETH_INTE_RX_FINISHED_INT_EN_MASK    BIT(2)

    /** 
     * Enable for interrupt triggered when a receive descriptor has
     * been processed while the Interrupt bit in the Control field of the
     * descriptor was set.
     */ 
#   define  ETH_INTE_RX_DONE_INT_EN_MASK        BIT(3)

    /** 
     * Enable for interrupt trigger on transmit buffer or descriptor
     * underrun situations.
     */ 
#   define  ETH_INTE_TX_UNDERRUN_INT_EN_MASK    BIT(4)

    /** 
     * Enable for interrupt trigger on transmit errors.
     */ 
#   define  ETH_INTE_TX_ERROR_INT_EN_MASK    BIT(5)

    /** 
     * Enable for interrupt triggered when all transmit descriptors
     * have been processed i.e. on the transition to the situation
     * where ProduceIndex == ConsumeIndex.
     */ 
#   define  ETH_INTE_TX_FINISHED_INT_EN_MASK    BIT(6)

    /** 
     * Enable for interrupt triggered when a descriptor has been
     * transmitted while the Interrupt bit in the Control field of the
     * descriptor was set.
     */ 
#   define  ETH_INTE_TX_DONE_INT_EN_MASK        BIT(7)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTE_RESERVED1_MASK         MULTI_BIT_MASK(11, 8)
#   define  ETH_INTE_RESERVED1_SHIFT        8

    /** 
     * Enable for interrupt triggered by the SoftInt bit in the IntStatus
     * register, caused by software writing a 1 to the SoftIntSet bit in
     * the IntSet register.
     */ 
#   define  ETH_INTE_SOFT_INT_EN_MASK       BIT(12)

    /** 
     * Enable for interrupt triggered by a Wakeup event detected by
     * the receive filter.
     */ 
#   define  ETH_INTE_WAKEUP_INT_EN_MASK    BIT(13)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTE_RESERVED2_MASK         MULTI_BIT_MASK(31, 14)
#   define  ETH_INTE_RESERVED2_SHIFT        14
    
    /** 
     * WO Interrupt clear register.
     */ 
    uint32_t reg_IntClear;

    /** 
     * Writing a ’1’ to one of these bits clears (0 to 7) the
     * corresponding status bit in interrupt status register
     * IntStatus.
     */ 
#   define  ETH_INTC_RX_OVERRUN_INT_CLR_MASK    BIT(0)
#   define  ETH_INTC_RX_ERROR_INT_CLR_MASK      BIT(1)
#   define  ETH_INTC_RX_FINISHED_INT_CLR_MASK   BIT(2)
#   define  ETH_INTC_RX_DONE_INT_CLR_MASK       BIT(3)
#   define  ETH_INTC_TX_UNDERRUN_INT_CLR_MASK   BIT(4)
#   define  ETH_INTC_TX_ERROR_INT_CLR_MASK      BIT(5)
#   define  ETH_INTC_TX_FINISHED_INT_CLR_MASK   BIT(6)
#   define  ETH_INTC_Tx_DONE_INT_CLR_MASK       BIT(7)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTC_RESERVED1_MASK         MULTI_BIT_MASK(11, 8)
#   define  ETH_INTC_RESERVED1_SHIFT        8
    
    /** 
     * Writing a ’1’ to one of these bits (12 and/or 13) clears the
     * corresponding status bit in interrupt status register
     * IntStatus.
     */ 
#   define  ETH_INTC_SOFT_INT_CLR_MASK      BIT(12)
#   define  ETH_INTC_WAKEUP_INT_CLR_MASK    BIT(13)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTC_RESERVED2_MASK         MULTI_BIT_MASK(31, 14)
#   define  ETH_INTC_RESERVED2_SHIFT        14

    /** 
     * WO Interrupt set register.
     */ 
    uint32_t reg_IntSet;

    /** 
     * Writing a ’1’ to one of these bits sets (0 to 7) the
     * corresponding status bit in interrupt status register
     * IntStatus.
     */ 
#   define  ETH_INTS_RX_OVERRUN_INT_SET_MASK    BIT(0)
#   define  ETH_INTS_RX_ERROR_INT_SET_MASK      BIT(1)
#   define  ETH_INTS_RX_FINISHED_INT_SET_MASK   BIT(2)
#   define  ETH_INTS_RX_DONE_INT_SET_MASK       BIT(3)
#   define  ETH_INTS_TX_UNDERRUN_INT_SET_MASK   BIT(4)
#   define  ETH_INTS_TX_ERROR_INT_SET_MASK      BIT(5)
#   define  ETH_INTS_TX_FINISHED_INT_SET_MASK   BIT(6)
#   define  ETH_INTS_Tx_DONE_INT_SET_MASK       BIT(7)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTS_RESERVED1_MASK         MULTI_BIT_MASK(11, 8)
#   define  ETH_INTS_RESERVED1_SHIFT        8
    
    /** 
     * Writing a ’1’ to one of these bits (12 and/or 13) sets the
     * corresponding status bit in interrupt status register
     * IntStatus.
     */ 
#   define  ETH_INTS_SOFT_INT_SET_MASK      BIT(12)
#   define  ETH_INTS_WAKEUP_INT_SET_MASK    BIT(13)

    /** 
     * Reserved. 
     */ 
#   define  ETH_INTS_RESERVED2_MASK         MULTI_BIT_MASK(31, 14)
#   define  ETH_INTS_RESERVED2_SHIFT        14

    /** 
     * Reserved
     */ 
    uint32_t reg_reserved8;

    /** 
     * R/W Power-down register.
     */ 
    uint32_t reg_PowerDown;

    /** 
     * Reserved. 
     */ 
#   define  ETH_PWRD_RESERVED_MASK      MULTI_BIT_MASK(30, 0)
#   define  ETH_PWRD_RESERVED_SHIFT     0
    
    /** 
     * If true, all AHB accesses will return a read/write error,
     * except accesses to the PowerDown register
     */ 
#   define  ETH_PWRD_POWER_DOWN_MAC_AHB_MASK    BIT(31)
    
    /** 
     * Reserved
     */ 
    uint32_t reg_reserved9;
} lpc2478_ethernet_t;

/*
 * Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_ethernet_t) == LPC2478_ETHERNET_SIZE);

C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MAC1) == 0x00);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MAC2) == 0x04);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_IPGT) == 0x08);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_IPGR) == 0x0C);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_CLRT) == 0x10);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MAXF) == 0x14);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_SUPP) == 0x18);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TEST) == 0x1C);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MCFG) == 0x20);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MCMD) == 0x24);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MADR) == 0x28);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MWTD) == 0x2C);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MRDD) == 0x30);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_MIND) == 0x34);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_SA[0]) == 0x40);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_SA[1]) == 0x44);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_SA[2]) == 0x48);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_Command) == 0x100);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_Status) == 0x104);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxDescriptor) == 0x108);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxStatus) == 0x10C);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxDescriptorNumber) == 0x110);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxProduceIndex) == 0x114);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxConsumeIndex) == 0x118);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TxDescriptor) == 0x11C);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TxStatus) == 0x120);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TxDescriptorNumber) == 0x124);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TxProduceIndex) == 0x128);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TxConsumeIndex) == 0x12C);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TSV0) == 0x158);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_TSV1) == 0x15C);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RSV) == 0x160);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_FlowControlCounter) == 0x170);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_FlowControlStatus) == 0x174);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxFliterCtrl) == 0x200);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxFilterWoLStatus) == 0x204);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_RxFilterWoLClear) == 0x208);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_HashFilterL) == 0x210);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_HashFilterH) == 0x214);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_IntStatus) == 0xFE0);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_IntEnable) == 0xFE4);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_IntClear) == 0xFE8);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_IntSet) == 0xFEC);
C_ASSERT(offsetof(lpc2478_ethernet_t, reg_PowerDown) == 0xFF4);

/**
 * Layout of a receive descriptor in memory 
 * (one descriptor per frame fragment)
 */
struct eth_receive_descriptor
{
    /** 
     * Base address of the data buffer for storing receive data.
     */ 
    uint32_t erd_packet;

    /** 
     * Control information
     */ 
    uint32_t erd_control;
   
    /** 
     * Size in bytes of the data buffer. This is the size of the buffer reserved by the
     * device driver for a frame or frame fragment i.e. the byte size of the buffer
     * pointed to by the Packet field. The size is -1 encoded e.g. if the buffer is 8
     * bytes the size field should be equal to 7.
     */ 
#   define  ETH_RXD_SIZE_MASK         MULTI_BIT_MASK(10, 0)
#   define  ETH_RXD_SIZE_SHIFT        0

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXD_RESERVED_MASK         MULTI_BIT_MASK(30, 11)
#   define  ETH_RXD_RESERVED_SHIFT        11
    
    /** 
     * If true generate an RxDone interrupt when the data in this frame or frame
     * fragment and the associated status information has been committed to
     * memory
     */ 
#   define  ETH_RXD_INTERRUPT_MASK    BIT(31)
};

/**
 * Layout of a receive status entry in memory 
 * (one status entry per receive descriptor or received frame fragment)
 */
struct eth_receive_status
{
    /** 
     * Receive status return flags.
     */ 
    uint32_t ers_status_info;

    /** 
     * The size in bytes of the actual data transferred into one fragment buffer. In
     * other words, this is the size of the frame or fragment as actually written by
     * the DMA manager for one descriptor. This may be different from the Size
     * bits of the Control field in the descriptor that indicate the size of the buffer
     * allocated by the device driver. Size is -1 encoded e.g. if the buffer has
     * 8 bytes the RxSize value will be 7.
     */ 
#   define  ETH_RXSI_RX_SIZE_MASK       MULTI_BIT_MASK(10, 0)
#   define  ETH_RXSI_RX_SIZE_SHIFT      0

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXSI_RESERVED2_MASK     MULTI_BIT_MASK(17, 11)
#   define  ETH_RXSI_RESERVED2_SHIFT    11
    
    /** 
     * Indicates this is a control frame for flow control, either a pause frame or a
     * frame with an unsupported opcode.
     */ 
#   define  ETH_RXSI_CONTROl_FRAME_MASK BIT(18)

    /** 
     * Indicates a VLAN frame.
     */ 
#   define  ETH_RXSI_VLAN_MASK          BIT(19)

    /** 
     * Indicates this frame has failed the Rx filter. These frames will not normally
     * pass to memory. But due to the limitation of the size of the buffer, part of
     * this frame may already be passed to memory. Once the frame is found to
     * have failed the Rx filter, the remainder of the frame will be discarded
     * without being passed to the memory. However, if the PassRxFilter bit in
     * the Command register is set, the whole frame will be passed to memory.
     */ 
#   define  ETH_RXSI_FailFilter_MASK    BIT(20)

    /** 
     * Set when a multicast frame is received.
     */ 
#   define  ETH_RXSI_MULTICAST_MASK    BIT(21)

    /** 
     * Set when a broadcast frame is received.
     */ 
#   define  ETH_RXSI_BROADCAST_MASK    BIT(22)

    /** 
     * The received frame had a CRC error.
     */ 
#   define  ETH_RXSI_CRC_ERROR_MASK    BIT(23)

    /** 
     * The PHY reports a bit error over the MII during reception.
     */ 
#   define  ETH_RXSI_SYMBOL_ERROR_MASK  BIT(24)

    /** 
     * The frame length field value in the frame specifies a valid length, but does
     * not match the actual data length.
     */ 
#   define  ETH_RXSI_LENGTH_ERROR_MASK    BIT(25)

    /** 
     * The received packet exceeds the maximum packet size.
     * 
     * NOTE: The EMAC doesn't distinguish the frame type and frame length,
     * so, e.g. when the IP(0x8000) or ARP(0x0806) packets are received,
     * it compares the frame type with the max length and gives the "Range"
     * error. In fact, this bit is not an error indication, but simply a
     * statement by the chip regarding the status of the received frame.
     */ 
#   define  ETH_RXSI_RANGE_ERROR_MASK    BIT(26)

    /** 
     * An alignment error is flagged when dribble bits are detected and also a
     * CRC error is detected. This is in accordance with IEEE std. 802.3/clause
     * 4.3.2.
     */ 
#   define  ETH_RXSI_ALIGNMENT_ERROR_MASK   BIT(27)

    /** 
     * Receive overrun. The adapter can not accept the data stream.
     */ 
#   define  ETH_RXSI_OVERRUN_MASK           BIT(28)

    /** 
     * No new Rx descriptor is available and the frame is too long for the buffer
     * size in the current receive descriptor.
     */
#   define  ETH_RXSI_NO_DESCRIPTOR_MASK     BIT(29)

    /** 
     * When set to 1, indicates this descriptor is for the last fragment of a frame.
     * If the frame consists of a single fragment, this bit is also set to 1.
     */
#   define  ETH_RXSI_LAST_FLAG_MASK         BIT(30)

    /** 
     * An error occurred during reception of this frame. This is a logical OR of
     * AlignmentError, RangeError, LengthError, SymbolError, CRCError, and Overrun.
     */
#   define  ETH_RXSI_ERROR_MASK             BIT(31)

    /** 
     * The concatenation of the destination address hash CRC and
     * the source address hash CRC.
     */ 
    uint32_t ers_status_hash_crc;

    /** 
     * Hash CRC calculated from the source address.
     */ 
#   define  ETH_RXSH_SA_HASH_CRC_MASK         MULTI_BIT_MASK(8, 0)
#   define  ETH_RXSH_SA_HASH_CRC_SHIFT        0

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXSH_RESERVED1_MASK         MULTI_BIT_MASK(15, 9)
#   define  ETH_RXSH_RESERVED1_SHIFT        9
    
    /** 
     * Hash CRC calculated from the destination address.
     */ 
#   define  ETH_RXSH_DA_HASH_CRC_MASK         MULTI_BIT_MASK(24, 16)
#   define  ETH_RXSH_DA_HASH_CRC_SHIFT        16 

    /** 
     * Reserved. 
     */ 
#   define  ETH_RXSH_RESERVED2_MASK         MULTI_BIT_MASK(31, 25)
#   define  ETH_RXSH_RESERVED2_SHIFT        25
};

/**
 * Layout of a transmit descriptor in memory
 * (one descriptor per frame fragment)
 */
struct eth_transmit_descriptor
{
    /** 
     * Base address of the data buffer containing transmit data.
     */ 
    uint32_t etd_packet;
   
    /** 
     * Control information
     */ 
    uint32_t etd_control;

    /** 
     * Size in bytes of the data buffer. This is the size of the frame or fragment as it
     * needs to be fetched by the DMA manager. In most cases it will be equal to the
     * byte size of the data buffer pointed to by the Packet field of the descriptor. Size
     * is -1 encoded e.g. a buffer of 8 bytes is encoded as the Size value 7.
     */ 
#   define  ETH_TXC_SIZE_MASK       MULTI_BIT_MASK(10, 0)
#   define  ETH_TXC_SIZE_SHIFT      0

    /** 
     * Reserved. 
     */ 
#   define  ETH_TXC_RESERVED_MASK   MULTI_BIT_MASK(25, 11)
#   define  ETH_TXC_RESERVED_SHIFT  11
    
    /** 
     * Per frame override. If true, bits 30:27 will override the defaults from the MAC
     * internal registers. If false, bits 30:27 will be ignored and the default values
     * from the MAC will be used.
     */ 
#   define  ETH_TXC_OVERRIDE_MASK   BIT(26)

    /** 
     * If true, enables huge frame, allowing unlimited frame sizes. When false,
     * prevents transmission of more than the maximum frame length (MAXF[15:0]).
     */ 
#   define  ETH_TXC_HUGE_MASK       BIT(27)

    /** 
     * If true, pad short frames to 64 bytes.
     */ 
#   define  ETH_TXC_PAD_MASK        BIT(28)

    /** 
     * If true, append a hardware CRC to the frame.
     */ 
#   define  ETH_TXC_CRC_MASK        BIT(29)

    /** 
     * If true, indicates that this is the descriptor for the last fragment in the transmit
     * frame. If false, the fragment from the next descriptor should be appended.
     */ 
#   define  ETH_TXC_Last_MASK       BIT(30)

    /** 
     * If true, a TxDone interrupt will be generated when the data in this frame or
     * frame fragment has been sent and the associated status information has been
     * committed to memory.
     */ 
#   define  ETH_TXC_INTERRUPT_MASK  BIT(31)
};


/**
 * Layout of a transmit status entry in memory
 * (one status entry per transmit descriptor or transmitted frame fragment)
 */
struct eth_transmit_status
{
    /** 
     * Transmit status return flags
     */ 
    uint32_t ets_status_info;

    /** 
     * Reserved. 
     */ 
#   define  ETH_TXSI_RESERVED_MASK              MULTI_BIT_MASK(20, 0)
#   define  ETH_TXSI_RESERVED_SHIFT             0
    
    /** 
     * The number of collisions this packet incurred, up to the
     * Retransmission Maximum.
     */ 
#   define  ETH_TXSI_COLLISION_COUNT_MASK       MULTI_BIT_MASK(24, 21)
#   define  ETH_TXSI_COLLISION_COUNT_SHIFT      21

    /** 
     * This packet incurred deferral, because the medium was occupied.
     * This is not an error unless excessive deferral occurs.
     */ 
#   define  ETH_TXSI_DEFER_MASK                 BIT(25)

    /** 
     * This packet incurred deferral beyond the maximum deferral limit and
     * was aborted.
     */ 
#   define  ETH_TXSI_EXCESSIVE_DEFER_MASK       BIT(26)

    /** 
     * Indicates this packet exceeded the maximum collision limit and was
     * aborted.
     */ 
#   define  ETH_TXSI_EXCESSIVE_COLLISION_MASK   BIT(27)

    /** 
     * An Out of window Collision was seen, causing packet abort.
     */ 
#   define  ETH_TXSI_LATE_COLLISION_MASK        BIT(28)
    /** 
     * A Tx underrun occurred due to the adapter not producing transmit
     * data.
     */ 
#   define  ETH_TXSI_UNDERRUN_MASK              BIT(29)

    /** 
     * The transmit stream was interrupted because a descriptor was not
     * available.
     */ 
#   define  ETH_TXSI_NO_DESCRIPTOR_MASK         BIT(30)

    /** 
     * An error occurred during transmission. This is a logical OR of
     * Underrun, LateCollision, ExcessiveCollision, and ExcessiveDefer.
     */ 
#   define  ETH_TXSI_ERROR_MASK                 BIT(31)
};

/**
 * Clock select encodings for field ETH_MCFG_CLOCK_SELECT_MASK
 */
enum eth_mcfg_clock_select
{
    ETH_MCFG_HOST_CLOCK_DIVIDED_BY_4 =  0x0, /* 0 0 x */
    ETH_MCFG_HOST_CLOCK_DIVIDED_BY_6 =  0x2, /* 0 1 0 */
    ETH_MCFG_HOST_CLOCK_DIVIDED_BY_8 =  0x3, /* 0 1 1 */
    ETH_MCFG_HOST_CLOCK_DIVIDED_BY_10 = 0x4, /* 1 0 0 */
    ETH_MCFG_HOST_CLOCK_DIVIDED_BY_14 = 0x5, /* 1 0 1 */
    ETH_MCFG_HOST_CLOCK_DIVIDED_BY_20 = 0x6, /* 1 1 0 */
    ETH_MCFG_HOST_CLOCK_DIVIDED_BY_28 = 0x7, /* 1 1 1 */
};

/**
 * Indexes of MII registers
 */
enum eth_mii_register_indexes
{
    ETH_MII_BMCR =          0x00,       // Basic mode control register
    ETH_MII_BMSR =          0x01,       // Basic mode status register
    ETH_MII_PHYSID1 =       0x02,       // PHY identifier 1         
    ETH_MII_PHYSID2 =       0x03,       // PHY identifier 2        
    ETH_MII_ADVERTISE =     0x04,       // Auto-Negotiation Advertisement register
    ETH_MII_LPA =           0x05,       // Auto-Negotiation Link partner ability register
    ETH_MII_EXPANSION =     0x06,       // Auto-Negotiation Expansion register
    ETH_MII_NEXT_PAGE =     0x07,       // Auto-Negotiation Next Page Register
    ETH_MII_LPA_NEXT_PAGE = 0x08,       // Link Partner Next Page Ability register
    ETH_MII_CTRL1000 =      0x09,       // 1000BASE-T control      
    ETH_MII_STAT1000 =      0x0a,       // 1000BASE-T status      
    ETH_MII_ESTATUS =	    0x0f,	// Extended Status 
    ETH_MII_DCOUNTER =      0x12,       // Disconnect counter 
    ETH_MII_FCSCOUNTER =    0x13,       // False carrier counter
    ETH_MII_NWAYTEST =      0x14,       // N-way auto-neg test reg 
    ETH_MII_RX_ER_COUNTER = 0x15,       // Receive error counter  
    ETH_MII_SREVISION =     0x16,       // Silicon revision      
    ETH_MII_RESERVED1 =     0x17,
    ETH_MII_LBRERROR =      0x18,       // Loopback, rx, bypass error 
    ETH_MII_PHYADDR =       0x19,       // PHY address             
    ETH_MII_RESERVED2 =     0x1a, 
    ETH_MII_INT_OR_STATUS = 0x1b,       // Interrupt Control/Status Register  
    ETH_MII_NCONFIG =       0x1c,       // Network interface config
    ETH_MII_100BT_CTRL =    0x1f,       // 100BASE-TX PHY Control Register
};


/*
 * Basic mode control register flags
 */
#define ETH_MII_BMCR_RESERVED_MASK          MULTI_BIT_MASK(5, 0)
#define ETH_MII_BMCR_SPEED1000_MASK	    BIT(6)  /* MSB of Speed (1000)         */
#define ETH_MII_BMCR_CTST_MASK              BIT(7)  /* Collision test              */
#define ETH_MII_BMCR_FULLDPLX_MASK          BIT(8)  /* Full duplex                 */
#define ETH_MII_BMCR_ANRESTART_MASK         BIT(9)  /* Auto negotiation restart    */
#define ETH_MII_BMCR_ISOLATE_MASK           BIT(10) /* Disconnect DP83840 from MII */
#define ETH_MII_BMCR_PDOWN_MASK             BIT(11) /* Powerdown the DP83840       */
#define ETH_MII_BMCR_ANENABLE_MASK          BIT(12) /* Enable auto negotiation     */
#define ETH_MII_BMCR_SPEED100_MASK          BIT(13) /* Select 100Mbps              */
#define ETH_MII_BMCR_LOOPBACK_MASK          BIT(14) /* TXD loopback bits           */
#define ETH_MII_BMCR_RESET_MASK             BIT(15) /* Reset the DP83840           */

/*
 * Basic mode status register (BMSR) flags
 */
#define ETH_MII_BMSR_ERCAP_MASK             BIT(0)  /* Ext-reg capability          */
#define ETH_MII_BMSR_JCD_MASK               BIT(1)  /* Jabber detected             */
#define ETH_MII_BMSR_LSTATUS_MASK           BIT(2)  /* Link status                 */
#define ETH_MII_BMSR_ANEGCAPABLE_MASK       BIT(3)  /* Able to do auto-negotiation */
#define ETH_MII_BMSR_RFAULT_MASK            BIT(4)  /* Remote fault detected       */
#define ETH_MII_BMSR_ANEGCOMPLETE_MASK      BIT(5)  /* Auto-negotiation complete   */
#define ETH_MII_BMSR_RESERVED_MASK          MULTI_BIT_MASK(7, 6)
#define ETH_MII_BMSR_ESTATEN_MASK	    BIT(8)  /* Extended Status in R15 */
#define ETH_MII_BMSR_100HALF2_MASK          BIT(9)  /* Can do 100BASE-T2 HDX */
#define ETH_MII_BMSR_100FULL2_MASK          BIT(10) /* Can do 100BASE-T2 FDX */
#define ETH_MII_BMSR_10HALF_MASK            BIT(11) /* Can do 10mbps, half-duplex  */
#define ETH_MII_BMSR_10FULL_MASK            BIT(12) /* Can do 10mbps, full-duplex  */
#define ETH_MII_BMSR_100HALF_MASK           BIT(13) /* Can do 100mbps, half-duplex */
#define ETH_MII_BMSR_100FULL_MASK           BIT(14) /* Can do 100mbps, full-duplex */
#define ETH_MII_BMSR_100BASE4_MASK          BIT(15) /* Can do 100mbps, 4k packets  */

/* 
 * Link partner ability register (LPA) flags
 */
#define ETH_MII_LPA_SELECTOR_MASK           MULTI_BIT_MASK(4, 0)
#define ETH_MII_LPA_10BASE_MASK             BIT(5)  // Can do 10mbps
#define ETH_MII_LPA_10BASE_FULL_MASK        BIT(6)  // Can do 10mbps full-duplex
#define ETH_MII_LPA_100BASE_MASK            BIT(7)  // Can do 100mbps
#define ETH_MII_LPA_100BASE_FULL_MASK       BIT(8)  // Can do 100mbps full-duplex
#define ETH_MII_LPA_100BASE_T4_MASK         BIT(9)  // Can do 100mbps 4k packets
#define ETH_MII_LPA_PAUSE_CAP_MASK          BIT(10) // Can pause               
#define ETH_MII_LPA_PAUSE_ASYM_MASK         BIT(11) // Can pause asymetrically
#define ETH_MII_LPA_RESERVED_MASK           BIT(12) // Unused
#define ETH_MII_LPA_REMOTE_FAULT_MASK       BIT(13) // Link partner faulted
#define ETH_MII_LPA_ACK_MASK                BIT(14) // Link partner acked us
#define ETH_MII_LPA_NEXT_PAGE_MASK          BIT(15) // Next page bit

/**
 * Maximum size of an Ethernet frame fragment in bytes
 */
#define ETH_MAX_FRAGMENT_SIZE   (2 * UINT32_C(1024))

C_ASSERT(ETH_MAX_FRAGMENT_SIZE % sizeof(uint32_t) == 0);

/**
 * Total number of transmit descriptors
 */
#define ETH_NUM_TRANSMIT_DESCRIPTORS  UINT32_C(384)

/**
 * Total number of receive descriptors
 */
#define ETH_NUM_RECEIVE_DESCRIPTORS  UINT32_C(384)

/**
 * Total number of fragments to store Ethernet headers for outgoing Ethernet
 * frames
 */
#define ETH_NUM_TX_FRAME_HEADER_FRAGMENTS   (ETH_NUM_TRANSMIT_DESCRIPTORS / 2)

/**
 * Total number of fragments to store data payload for outgoing Ethernet frames
 */
#define ETH_NUM_TX_FRAME_DATA_FRAGMENTS \
        (ETH_NUM_TRANSMIT_DESCRIPTORS - ETH_NUM_TX_FRAME_HEADER_FRAGMENTS)

/**
 * Total number of fragments to store Ethernet headers for incoming Ethernet
 * frames
 */
#define ETH_NUM_RX_FRAME_HEADER_FRAGMENTS   (ETH_NUM_RECEIVE_DESCRIPTORS / 2)

/**
 * Total number of fragments to store data payload for incoming Ethernet
 * frames
 */
#define ETH_NUM_RX_FRAME_DATA_FRAGMENTS \
        (ETH_NUM_RECEIVE_DESCRIPTORS - ETH_NUM_RX_FRAME_HEADER_FRAGMENTS)

C_ASSERT(
    (ETH_NUM_TX_FRAME_DATA_FRAGMENTS + ETH_NUM_RX_FRAME_DATA_FRAGMENTS) *
        ETH_MAX_FRAGMENT_SIZE * 64 <= ETHERNET_DATA_FRAGMENT_BUFFER_POOL_SIZE);

#endif /* _LPC2478_ETHERNET_H */
