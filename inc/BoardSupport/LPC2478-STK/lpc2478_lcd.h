#ifndef _LCD_H
#define _LCD_H

/**
 * Base address for the LPC2478 LCD controller
 */
#define LPC2478_LCD_CONTROLLER_BASE_ADDR    UINT32_C(0xE01FC1B8)

/**
 * Register space size for the LCD controller
 */
#define LPC2478_LCD_CONTROLLER_SIZE    UINT32_C(0x1FC14A78)

/**
 * Memory-mapped I/O registers of the LCD controller
 */
typedef volatile struct lpc2478_lcd_controller {
    uint32_t reg_LCD_CFG;
#   define  LCD_CLKDIV_MASK  MULTI_BIT_MASK(4, 0)

    uint32_t padding1[0x7F04F91];

    uint32_t reg_LCD_TIMH;
#   define  LCD_TIMH_HBP_MASK    MULTI_BIT_MASK(31, 24)
#   define  LCD_TIMH_HFP_MASK    MULTI_BIT_MASK(23, 16)
#   define  LCD_TIMH_HSW_MASK    MULTI_BIT_MASK(15, 8)
#   define  LCD_TIMH_PPL_MASK    MULTI_BIT_MASK(7, 2) 
#   define  LCD_TIMH_HBP_SHIFT   24
#   define  LCD_TIMH_HFP_SHIFT   16
#   define  LCD_TIMH_HSW_SHIFT   8
#   define  LCD_TIMH_PPL_SHIFT   2 

    uint32_t reg_LCD_TIMV;
#   define  LCD_TIMV_VBP_MASK	MULTI_BIT_MASK(31, 24)
#   define  LCD_TIMV_VFP_MASK	MULTI_BIT_MASK(23, 16)
#   define  LCD_TIMV_VSW_MASK	MULTI_BIT_MASK(15, 10)
#   define  LCD_TIMV_LPP_MASK	MULTI_BIT_MASK(9, 0)
#   define  LCD_TIMV_VBP_SHIFT	24
#   define  LCD_TIMV_VFP_SHIFT	16
#   define  LCD_TIMV_VSW_SHIFT	10
    
    uint32_t reg_LCD_POL;
#   define  LCD_POL_PCD_HI_MASK	    MULTI_BIT_MASK(31, 27)
#   define  LCD_POL_BCD_MASK	    BIT(26)
#   define  LCD_POL_CPL_MASK	    MULTI_BIT_MASK(25, 16)
#   define  LCD_POL_IOE_MASK	    BIT(14)
#   define  LCD_POL_IPC_MASK	    BIT(13)
#   define  LCD_POL_IHS_MASK	    BIT(12)
#   define  LCD_POL_IVS_MASK	    BIT(11)
#   define  LCD_POL_ACB_MASK	    MULTI_BIT_MASK(10, 6)
#   define  LCD_POL_CLKSEL_MASK	    BIT(5)
#   define  LCD_POL_PCD_LO_MASK     MULTI_BIT_MASK(4, 0)
#   define  LCD_POL_PCD_HI_SHIFT    27
#   define  LCD_POL_CPL_SHIFT	    16
#   define  LCD_POL_ACB_SHIFT	    6

    uint32_t reg_LCD_LE;
#   define  LCD_LE_LEE_MASK BIT(16)
#   define  LCD_LE_LED_MASK MULTI_BIT_MASK(6, 0)

    uint32_t reg_LCD_UPBASE;
#   define  LCD_LCDUPBASE_MASK   MULTI_BIT_MASK(31, 3)
#   define  LCD_LCDUPBASE_SHIFT  3

    uint32_t reg_LCD_LPBASE;
#   define  LCD_LCDLPBASE_MASK   MULTI_BIT_MASK(31, 3)
#   define  LCD_LCDLPBASE_SHIFT  3

    uint32_t reg_LCD_CTRL;
#   define  LCD_CTRL_RESERVED1_MASK     MULTI_BIT_MASK(31, 17)
#   define  LCD_CTRL_RESERVED1_SHIFT    17
#   define  LCD_CTRL_WATERMARK_MASK     BIT(16)
#   define  LCD_CTRL_RESERVED2_MASK     MULTI_BIT_MASK(15, 14)
#   define  LCD_CTRL_RESERVED2_SHIFT    14
#   define  LCD_CTRL_LCDVCOMP_MASK      MULTI_BIT_MASK(13, 12)
#   define  LCD_CTRL_LCDVCOMP_SHIFT     12
#   define  LCD_CTRL_LCDPWR_MASK        BIT(11)
#   define  LCD_CTRL_BEPO_MASK          BIT(10)
#   define  LCD_CTRL_BEBO_MASK          BIT(9)
#   define  LCD_CTRL_BGR_MASK           BIT(8)
#   define  LCD_CTRL_LCDDUAL_MASK       BIT(7)
#   define  LCD_CTRL_LCDMONO8_MASK      BIT(6)
#   define  LCD_CTRL_LCDTFT_MASK        BIT(5)
#   define  LCD_CTRL_LCDBW_MASK         BIT(4)
#   define  LCD_CTRL_LCDBPP_MASK        MULTI_BIT_MASK(3, 1)
#   define  LCD_CTRL_LCDBPP_SHIFT       1
#   define  LCD_CTRL_LCDEN_MASK         BIT(0)

    uint32_t reg_LCD_INTMSK;
#   define  LCD_INTMSK_BERIM_MASK    BIT(4)
#   define  LCD_INTMSK_VCOMPIM_MASK  BIT(3)
#   define  LCD_INTMSK_LNBUIM_MASK   BIT(2)
#   define  LCD_INTMSK_FUFIM_MASK    BIT(1)
    
    uint32_t reg_LCD_INTRAW;
#   define  LCD_INTRAW_BERRAW_MASK   BIT(4)
#   define  LCD_INTRAW_VCOMPRIS_MASK BIT(3)
#   define  LCD_INTRAW_LNBURIS_MASK  BIT(2)
#   define  LCD_INTRAW_FUFRIS_MASK   BIT(1)
    
    uint32_t reg_LCD_INTSTAT;
#   define  LCD_INTSTAT_BERMIS_MASK	BIT(4)
#   define  LCD_INTSTAT_VCOMPMIS_MASK	BIT(3)
#   define  LCD_INTSTAT_LNBUMIS_MASK	BIT(2)
#   define  LCD_INTSTAT_FUFMIS_MASK	BIT(1)
    
    uint32_t reg_LCD_INTCLR;
#   define  LCD_INTCLR_BERIC_MASK	BIT(4)
#   define  LCD_INTCLR_VCOMPIC_MASK	BIT(3)
#   define  LCD_INTCLR_LNBUIC_MASK	BIT(2)
#   define  LCD_INTCLR_FUFIC_MASK	BIT(1)
    
    uint32_t reg_LCD_UPCURR;

    uint32_t reg_LCD_LPCURR;

    uint32_t padding2[0x73];

    uint32_t reg_LCD_PAL[128];
#   define  LCD_PALETTE_ENTRY2_INTENSITY    BIT(31)
#   define  LCD_PALETTE_ENTRY2_BLUE_MASK    MULTI_BIT_MASK(30, 26)
#   define  LCD_PALETTE_ENTRY2_GREEN_MASK   MULTI_BIT_MASK(25, 21)
#   define  LCD_PALETTE_ENTRY2_RED_MASK     MULTI_BIT_MASK(20, 16)
#   define  LCD_PALETTE_ENTRY1_INTENSITY    BIT(15)
#   define  LCD_PALETTE_ENTRY1_BLUE_MASK    MULTI_BIT_MASK(14, 10)
#   define  LCD_PALETTE_ENTRY1_GREEN_MASK   MULTI_BIT_MASK(9, 5)
#   define  LCD_PALETTE_ENTRY1_RED_MASK     MULTI_BIT_MASK(4, 0)
#   define  LCD_PALETTE_ENTRY2_BLUE_SHIFT   26
#   define  LCD_PALETTE_ENTRY2_GREEN_SHIFT  21
#   define  LCD_PALETTE_ENTRY2_RED_SHIFT    16
#   define  LCD_PALETTE_ENTRY1_BLUE_SHIFT   10
#   define  LCD_PALETTE_ENTRY1_GREEN_SHIFT  5

    uint32_t padding3[0x100];

    uint32_t reg_CRSR_IMG[256];

    uint32_t reg_CRSR_CTRL;
#   define  LCD_CURSOR_NUM_MASK             MULTI_BIT_MASK(5, 4)
#   define  LCD_CURSOR_ON_MASK              BIT(0)
#   define  LCD_CURSOR_NUM_SHIFT            4

    uint32_t reg_CRSR_CFG;
#   define  LCD_FRAME_SYNC_MASK             BIT(1)
#   define  LCD_CURSOR_SIZE_MASK            BIT(0)

    uint32_t reg_CRSR_PAL[2];
#   define  LCD_BLUE_MASK                   MULTI_BIT_MASK(23, 16)
#   define  LCD_GREEN_MASK                  MULTI_BIT_MASK(15, 8)
#   define  LCD_RED_MASK                    MULTI_BIT_MASK(7, 0)
#   define  LCD_BLUE_SHIFT                  16
#   define  LCD_GREEN_SHIFT                 8

    uint32_t reg_CRSR_XY;
#   define  LCD_CURSOR_Y_MASK               MULTI_BIT_MASK(25, 16)
#   define  LCD_CURSOR_X_MASK               MULTI_BIT_MASK(9, 0)
#   define  LCD_CURSOR_Y_SHIFT              16

    uint32_t reg_CRSR_CLIP;
#   define  LCD_CURSOR_CLIP_Y_MASK          MULTI_BIT_MASK(13, 8)
#   define  LCD_CURSOR_CLIP_X_MASK          MULTI_BIT_MASK(5, 0)
#   define  LCD_CURSOR_CLIP_Y_SHIFT         8

    uint32_t padding4[0x2];

    uint32_t reg_CRSR_INTMSK;
#   define  LCD_CURSOR_INTR_MASK                BIT(0)

    uint32_t reg_CRSR_INTCLR;
#   define  LCD_CURSOR_INTR_CLEAR_MASK          BIT(0)

    uint32_t reg_CRSR_INTRAW;
#   define  LCD_CURSOR_RAW_INTR_STATUS_MASK     BIT(0)

    uint32_t reg_CRSR_INTSTAT;
#   define  LCD_CURSOR_MASKED_INTR_STATUS_MASK  BIT(0)
} lpc2478_lcd_controller_t;

/*
 * Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_lcd_controller_t) == LPC2478_LCD_CONTROLLER_SIZE);

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_CFG) == 
    UINT32_C(0xE01FC1B8));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_TIMH) == 
    UINT32_C(0xFFE10000));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_TIMV) == 
    UINT32_C(0xFFE10004));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_POL) == 
    UINT32_C(0xFFE10008));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_LE) == 
    UINT32_C(0xFFE1000C));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_UPBASE) == 
    UINT32_C(0xFFE10010));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_LPBASE) == 
    UINT32_C(0xFFE10014));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_CTRL) == 
    UINT32_C(0xFFE10018));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_INTMSK) == 
    UINT32_C(0xFFE1001C));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_INTRAW) == 
    UINT32_C(0xFFE10020));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_INTSTAT) == 
    UINT32_C(0xFFE10024));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_INTCLR) == 
    UINT32_C(0xFFE10028));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_UPCURR) == 
    UINT32_C(0xFFE1002C));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_LPCURR) == 
    UINT32_C(0xFFE10030));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_PAL[0]) == 
    UINT32_C(0xFFE10200));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_LCD_PAL[127]) == 
    UINT32_C(0xFFE103FC));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_IMG[0]) == 
    UINT32_C(0xFFE10800));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_IMG[255]) == 
    UINT32_C(0xFFE10BFC));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_CTRL) == 
    UINT32_C(0xFFE10C00));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_CFG) == 
    UINT32_C(0xFFE10C04));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_PAL[0]) == 
    UINT32_C(0xFFE10C08));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_PAL[1]) == 
    UINT32_C(0xFFE10C0C));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_XY) == 
    UINT32_C(0xFFE10C10));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_CLIP) == 
    UINT32_C(0xFFE10C14));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_INTMSK) == 
    UINT32_C(0xFFE10C20));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_INTCLR) == 
    UINT32_C(0xFFE10C24));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_INTRAW) == 
    UINT32_C(0xFFE10C28));

C_ASSERT(
    LPC2478_LCD_CONTROLLER_BASE_ADDR + offsetof(lpc2478_lcd_controller_t, reg_CRSR_INTSTAT) == 
    UINT32_C(0xFFE10C2C));


/**
 * Values for the LCD_CTRL_LCDBPP_MASK field of the reg_LCD_CTRL register
 */
enum lcd_bits_per_pixel
{
    LCD_1_BITS_PER_PIXEL = 0x0,
    LCD_2_BITS_PER_PIXEL,
    LCD_4_BITS_PER_PIXEL,
    LCD_8_BITS_PER_PIXEL,
    LCD_16_BITS_PER_PIXEL,              /* 16 bpp true-color non-palettized 1:5:5:5 */
    LCD_24_BITS_PER_PIXEL,              /* 24 bpp true-color non-palettized */
    LCD_16_BITS_PER_PIXEL_5_6_5_MODE,
    LCD_12_BITS_PER_PIXEL_4_4_4_MODE
};

#endif /* _LCD_H */
