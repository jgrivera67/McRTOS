/**
 * @file mp3.c
 *
 * MP3 player device (VS1002 chip) abstraction layer implementation for
 * the lpc2478-stk board
 *
 * @author German Rivera 
 */ 

#include "mp3.h"
#include "utils.h"
#include "lpc2478_stk_board.h"
#include "hardware.h"
#include "lcd.h"
#include "debug.h"
#include "rtos_wrapper.h"

/*
 * The SPI-based interface to the VS1002 chip consists of two protocols:
 *
 * - Serial command interface (SCI) protocol (selected when the CS_AU pin assert low):
 *   1. Activate the LPC2478 CS_AU pin (asserted low)
 *   2. Send SCI command header on the SPI port (2 bytes)
 *   3. Send or receive data word on the SPI port (2 bytes)
 *   4. Deactivate the LPC2478 CS_AU pin
 *
 * - Serial data interface (SDI) protocol (selected when the CS_AU pin is high):
 *   Send a data packet of up to 32 bytes on the SPI port
 *
 * The SCI command header has the format:
 * - command opcode (e.g, VS1002_READ_CMD, VS1002_WRITE_CMD) - 1 byte
 * - internal address of a 16-bit VS1002's register - 1 byte
 *
 * Each read or write operation can read or write a single register. Bytes are
 * always send MSb first (high byte first).
 *
 * The SM SDINEW bit in the VS1002's SCI_MODE register is set 1 by default (upon reset)
 * The SM_SDISHARE SDINEW bit in the VS1002's SCI_MODE register is set 0 by default (upon reset)
 * If SM_SDISHARE is 1 in the VS1002's SCI_MODE register, the XDCS signal is internally generated
 * by inverting the XCS input. This is necessary for the LPC2478-SDK board.
 */

#define VS1002_SDI_RECEIVE_FIFO_SIZE_IN_BYTES 32

#define VS1002_READ_CMD         UINT8_C(0x03)

#define VS1002_WRITE_CMD        UINT8_C(0x02)

/**
 * Internal addresses of the VS1002's registers
 */
typedef enum vs1002_register_index
{
    REG_SCI_MODE =           0x0,    /* Mode control */
    REG_SCI_STATUS =         0x1,    /* Status of the VS1002 chip */
    REG_SCI_BASS =           0x2,    /* Built-in bass enhancer */
    REG_SCI_CLOCKF =         0x3,    /* Clock freq + doubler */
    REG_SCI_DECODE_TIME =    0x4,    /* Decode time in seconds (read-only) */
    REG_SCI_AUDATA =         0x5,    /* Misc. audio data */
    REG_SCI_WRAM =           0x6,    /* RAM write */
    REG_SCI_WRAMADDR =       0x7,    /* Base address for RAM write */
    REG_SCI_HDAT0 =          0x8,    /* Stream header data 0 */
    REG_SCI_HDAT1 =          0x9,    /* Stream header data 1 */
    REG_SCI_AIADDR =         0xa,    /* Start address of application */
    REG_SCI_VOLUME =         0xb,    /* Volume control */
    REG_SCI_AICTRL0 =        0xc,    /* Application control register 0 */
    REG_SCI_AICTRL1 =        0xd,    /* Application control register 1 */
    REG_SCI_AICTRL2 =        0xe,    /* Application control register 2 */
    REG_SCI_AICTRL =         0xf,    /* Application control register 3 */
        
    REG_SCI_NUM_REGISTERS
} vs1002_register_index_t;


/**
 * Bit masks for the REG_SCI_MODE register
 */
#define SM_DIFF_MASK        BIT(0)
#define SM_SETTOZERO_MASK   BIT(1)   
#define SM_RESET_MASK       BIT(2)
#define SM_OUTOFWAV_MASK    BIT(3)
#define SM_PDOWN_MASK       BIT(4)
#define SM_TESTS_MASK       BIT(5)
#define SM_STREAM_MASK      BIT(6)
#define SM_PLUSV_MASK       BIT(7)
#define SM_DACT_DCLK_MASK   BIT(8)
#define SM_SDIORD_SDI_MASK  BIT(9)
#define SM_SDISHARE_MASK    BIT(10)
#define SM_SDINEW_MASK      BIT(11)
#define SM_ADPCM_MASK       BIT(12)
#define SM_ADPCM_HP_MASK    BIT(13)

//XXX #define MP3_PLAY_MODE_MASK   (SM_SDINEW_MASK | SM_SDISHARE_MASK | SM_STREAM_MASK)
//XXX #define MP3_PLAY_MODE_MASK   (SM_SDINEW_MASK | SM_SDISHARE_MASK)
#define MP3_PLAY_MODE_MASK   (SM_SDINEW_MASK)

/*
 * Bit masks for the REG_SCI_STATUS register
 */
#define SS_VERSION_MASK     MULTI_BIT_MASK(6, 4) 
#define SS_VERSION_SHIFT    4
#define SS_APDOWN2_MASK     BIT(3)                  /* Analog driver powerdown */
#define SS_APDOWN1_MASK     BIT(2)                  /* Analog internal powerdown */
#define SS_AVOL_MASK        MULTI_BIT_MASK(1, 0)    /* Analog volume control */
#define SS_AVOL_SHIFT       0

#define VS1002_CLOCK_FREQ   UINT16_C(0x9800)    /* 12.288MHz * 2 */

#define BOTH_SPEAKERS_VOLUME(_volume) \
        (((uint16_t)(_volume) << 8) | (_volume))

/**
 * LPC2478 CS_AU (audio chip select) output pin (asserted low)
 */
static const struct pin_config_info g_pin_cs_au =
    PIN_COFIG_INFO_INITIALIZER(
        VS1002_PINS_GPIO_PORT, VS1002_CS_AU_PIN_BIT_INDEX, PINSEL_PRIMARY, false);

/**
 * LPC2478 DREQ (data request) input pin
 */
static const struct pin_config_info g_pin_dreq =
    PIN_COFIG_INFO_INITIALIZER(
        VS1002_PINS_GPIO_PORT, VS1002_DREQ_PIN_BIT_INDEX, PINSEL_PRIMARY, true);

/**
 * LPC2478 P0.16 output pin used to drive the VS1002's DCS input pin (asserted low)
 */
static const struct pin_config_info g_pin_dcs =
    PIN_COFIG_INFO_INITIALIZER(
        VS1002_DCS_PIN_GPIO_PORT, VS1002_DCS_PIN_BIT_INDEX, PINSEL_PRIMARY, false);
    
/**
 * SSP port used to communicate wit the MP3 device
 */ 
static const struct ssp_controller *g_mp3_ssp_controller_p = NULL;


static void 
mp3_sci_write_vs1002_register(
    vs1002_register_index_t vs1002_register_index,
    uint_fast16_t value)
{
    ASSERT(
        vs1002_register_index < REG_SCI_NUM_REGISTERS && value <= UINT16_MAX,
        vs1002_register_index, value);

    activate_output_pin(&g_pin_cs_au);

    uint_fast16_t cmd_header = (VS1002_WRITE_CMD << 8) | vs1002_register_index;

    (void)ssp_transmit_receive_16bit_value(g_mp3_ssp_controller_p, cmd_header);

    (void)ssp_transmit_receive_16bit_value(g_mp3_ssp_controller_p, value);

    deactivate_output_pin(&g_pin_cs_au);
}


static uint_fast16_t
mp3_sci_read_vs1002_register(
    vs1002_register_index_t vs1002_register_index)
{
    ASSERT(vs1002_register_index < REG_SCI_NUM_REGISTERS,
        vs1002_register_index, 0);

    activate_output_pin(&g_pin_cs_au);

    uint_fast16_t cmd_header = (VS1002_READ_CMD << 8) | vs1002_register_index;

    (void)ssp_transmit_receive_16bit_value(g_mp3_ssp_controller_p, cmd_header);

    uint_fast16_t value = ssp_transmit_receive_16bit_value(g_mp3_ssp_controller_p, 0);

    deactivate_output_pin(&g_pin_cs_au);

    return value;
}


static void 
mp3_sdi_write_buffer(
    const uint8_t *buffer,
    size_t length)
{
    ASSERT(buffer != NULL, 0, 0);
    ASSERT(length != 0, 0, 0);

    //XXX ssp_flush_transmit_receive_fifos(g_mp3_ssp_controller_p);

    const uint8_t *buffer_cursor_p = buffer;
    size_t remaining_bytes = length;
    size_t sdi_chunk_size = VS1002_SDI_RECEIVE_FIFO_SIZE_IN_BYTES;

    /*
     * transmit & receive the buffer in SDI chunks:
     */ 
    for ( ; ; )
    {
        if (remaining_bytes < sdi_chunk_size)
        {
            sdi_chunk_size = remaining_bytes;
        }

        while (!read_input_pin(&g_pin_dreq))
        {
            rtos_delay_task(10);
        }

        activate_output_pin(&g_pin_dcs);

        ssp_transmit_receive_buffer(
            g_mp3_ssp_controller_p, buffer_cursor_p, NULL, sdi_chunk_size);

        deactivate_output_pin(&g_pin_dcs);

        delay_loop(10);

        ASSERT(remaining_bytes >= sdi_chunk_size,
            remaining_bytes, sdi_chunk_size);

        buffer_cursor_p += sdi_chunk_size;
        remaining_bytes -= sdi_chunk_size;
        
        if (remaining_bytes == 0)
        {
            break;
        }
    }

    //XXX ssp_flush_transmit_receive_fifos(g_mp3_ssp_controller_p);
}


void
init_mp3(
    const struct ssp_controller *ssp_controller_p)
{
    ASSERT(g_mp3_ssp_controller_p == NULL, g_mp3_ssp_controller_p, 0);

    g_mp3_ssp_controller_p = ssp_controller_p;

    init_ssp(ssp_controller_p, true);

    /*
     * Configure the LPC2478 input/output pins:
     */
    configure_pin(&g_pin_cs_au, true);
    configure_pin(&g_pin_dreq, false);
    configure_pin(&g_pin_dcs, true);

    /*
     * Configure the VS1002's mode control register:
     */
    mp3_sci_write_vs1002_register(REG_SCI_MODE, MP3_PLAY_MODE_MASK);

    /*
     * Configure the VS1002's clock frequency register:
     */
    mp3_sci_write_vs1002_register(REG_SCI_CLOCKF, VS1002_CLOCK_FREQ);

    mp3_set_volume(MP3_DEFAULT_VOLUME);

    /*
     * Read back the SCI mode control register:
     */
    uint_fast16_t sci_register = mp3_sci_read_vs1002_register(REG_SCI_MODE);

    ASSERT(sci_register == MP3_PLAY_MODE_MASK,
        sci_register, MP3_PLAY_MODE_MASK);

    /*
     * Read the SCI status register:
     */
    sci_register = mp3_sci_read_vs1002_register(REG_SCI_STATUS);

    uint_fast8_t version =
        GET_BIT_FIELD(sci_register, SS_VERSION_MASK, SS_VERSION_SHIFT);

    ASSERT(version == 2,
        version, sci_register);
}


void 
mp3_soft_reset(void)
{
    uint_fast16_t mode_value =
        mp3_sci_read_vs1002_register(REG_SCI_MODE);

    /*
     * Start soft reset:
     */
    mp3_sci_write_vs1002_register(REG_SCI_MODE, mode_value | SM_RESET_MASK);
  
    delay_loop(20);
   
    while (!read_input_pin(&g_pin_dreq))
        ;

    /*
     * NOTE: the soft reset bit clears by itself after the the VS1002 completes
     * the reset
     */
    mode_value = mp3_sci_read_vs1002_register(REG_SCI_MODE);
    ASSERT((mode_value & SM_RESET_MASK) == 0, mode_value, 0);
}


void 
mp3_play_sine_test(void)
{
    static const uint8_t start_sine_test[] =  { 0x53, 0xEF, 0x6E, 0x44, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t stop_sine_test[] =   { 0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00 };

    uint_fast16_t saved_mode = mp3_sci_read_vs1002_register(REG_SCI_MODE);

    /*
     * Change to test mode:
     */ 
    mp3_sci_write_vs1002_register(
        REG_SCI_MODE, (saved_mode & ~SM_STREAM_MASK) | SM_TESTS_MASK);

    delay_loop(20);

    for (uint_fast8_t i = 0; i < 3; i ++)
    {
        mp3_sdi_write_buffer(start_sine_test, sizeof start_sine_test);
        rtos_delay_task(1000); /* 1 sec */
        mp3_sdi_write_buffer(stop_sine_test, sizeof stop_sine_test);
        rtos_delay_task(1000); /* 1 sec */
    }

    /*
     * Restore original mode: 
     */
    mp3_sci_write_vs1002_register(REG_SCI_MODE, saved_mode);
    delay_loop(20);
}


void 
mp3_play_buffer(const uint8_t *buffer, size_t length)
{
    mp3_sdi_write_buffer(buffer, length);
}

void
mp3_set_volume(uint_fast8_t volume)
{
    /*
     * Higher value indicates lower volume and lower value indicates
     * higher volume
     */
    ASSERT(
        volume <= MP3_MIN_VOLUME,
        volume, 0);

    /*
     * Configure the VS1002's volume control register:
     */
    mp3_sci_write_vs1002_register(
        REG_SCI_VOLUME, 
        BOTH_SPEAKERS_VOLUME(volume));
}
