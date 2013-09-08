#ifndef __EMC_H
#define __EMC_H

#include <inttypes.h>   /* C99 int types */
#include "compile_time_checks.h"

/**
 * grivera: Base address for the LPC2478 External Memory Controller (EMC)
 */
#define LPC2478_EMC_BASE_ADDR   UINT32_C(0xFFE08000)

/**
 * grivera: Register space size for the EMC
 */
#define LPC2478_EMC_SIZE    UINT32_C(0x280)

typedef volatile struct lpc2478_emc {
    uint32_t    reg_CONTROL;            /* RW */
    uint32_t    reg_STATUS;             /* RO */
    uint32_t    reg_CONFIG;             /* RW */
    uint32_t    reg_padding0[5];
    uint32_t    reg_DYNAMICCONTROL;     /* RW */
    uint32_t    reg_DYNAMICREFRESH;     /* RW */
    uint32_t    reg_DYNAMICREADCONFIG;  /* RW */
    uint32_t    reg_padding1;
    uint32_t    reg_DYNAMICRP;          /* RW */
    uint32_t    reg_DYNAMICRAS;         /* RW */
    uint32_t    reg_DYNAMICSREX;        /* RW */
    uint32_t    reg_DYNAMICAPR;         /* RW */
    uint32_t    reg_DYNAMICDAL;         /* RW */
    uint32_t    reg_DYNAMICWR;          /* RW */
    uint32_t    reg_DYNAMICRC;          /* RW */
    uint32_t    reg_DYNAMICRFC;         /* RW */
    uint32_t    reg_DYNAMICXSR;         /* RW */
    uint32_t    reg_DYNAMICRRD;         /* RW */
    uint32_t    reg_DYNAMICMRD;         /* RW */
    uint32_t    reg_padding2[9];
    uint32_t    reg_STATICEXTENDEDWAIT; /* RW */
    uint32_t    reg_padding3[31];

    struct {
        uint32_t    reg_DYNAMICCONFIG;     /* RW */
        uint32_t    reg_DYNAMICRASCAS;     /* RW */
        uint32_t    reg_padding4[6];
    } dynamic_config[4];

    uint32_t    reg_padding5[32];

    struct {
        uint32_t reg_STATICCONFIG;      /* RW */
        uint32_t reg_STATICWAITWEN;     /* RW */
        uint32_t reg_STATICWAITOEN;     /* RW */
        uint32_t reg_STATICWAITRD;      /* RW */
        uint32_t reg_STATICWAITPAGE;    /* RW */
        uint32_t reg_STATICWAITWR;      /* RW */
        uint32_t reg_STATICWAITTURN;    /* RW */
        uint32_t reg_padding6;
    } static_config[4];                 
} lpc2478_emc_t;


/*
 * grivera - Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_emc_t) == LPC2478_EMC_SIZE);

C_ASSERT(offsetof(lpc2478_emc_t, reg_CONTROL) == 0x00);
C_ASSERT(offsetof(lpc2478_emc_t, reg_STATUS) == 0x04);
C_ASSERT(offsetof(lpc2478_emc_t, reg_CONFIG) == 0x08);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICCONTROL) == 0x020);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICREFRESH) == 0x024);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICREADCONFIG) == 0x028);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICRP) == 0x030);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICRAS) == 0x034);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICSREX) == 0x038);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICAPR) == 0x03C);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICDAL) == 0x040);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICWR) == 0x044);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICRC) == 0x048);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICRFC) == 0x04C);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICXSR) == 0x050);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICRRD) == 0x054);
C_ASSERT(offsetof(lpc2478_emc_t, reg_DYNAMICMRD) == 0x058);
C_ASSERT(offsetof(lpc2478_emc_t, reg_STATICEXTENDEDWAIT) == 0x080);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[0].reg_DYNAMICCONFIG) == 0x100);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[0].reg_DYNAMICRASCAS) == 0x104);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[1].reg_DYNAMICCONFIG) == 0x120);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[1].reg_DYNAMICRASCAS) == 0x124);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[2].reg_DYNAMICCONFIG) == 0x140);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[2].reg_DYNAMICRASCAS) == 0x144);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[3].reg_DYNAMICCONFIG) == 0x160);
C_ASSERT(offsetof(lpc2478_emc_t, dynamic_config[3].reg_DYNAMICRASCAS) == 0x164);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[0].reg_STATICCONFIG) == 0x200);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[0].reg_STATICWAITWEN) == 0x204);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[0].reg_STATICWAITOEN) == 0x208);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[0].reg_STATICWAITRD) == 0x20C);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[0].reg_STATICWAITPAGE) == 0x210);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[0].reg_STATICWAITWR) == 0x214);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[0].reg_STATICWAITTURN) == 0x218);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[1].reg_STATICCONFIG) == 0x220);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[1].reg_STATICWAITWEN) == 0x224);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[1].reg_STATICWAITOEN) == 0x228);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[1].reg_STATICWAITRD) == 0x22C);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[1].reg_STATICWAITPAGE) == 0x230);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[1].reg_STATICWAITWR) == 0x234);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[1].reg_STATICWAITTURN) == 0x238);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[2].reg_STATICCONFIG) == 0x240);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[2].reg_STATICWAITWEN) == 0x244);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[2].reg_STATICWAITOEN) == 0x248);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[2].reg_STATICWAITRD) == 0x24C);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[2].reg_STATICWAITPAGE) == 0x250);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[2].reg_STATICWAITWR) == 0x254);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[2].reg_STATICWAITTURN) == 0x258);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[3].reg_STATICCONFIG) == 0x260);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[3].reg_STATICWAITWEN) == 0x264);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[3].reg_STATICWAITOEN) == 0x268);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[3].reg_STATICWAITRD) == 0x26C);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[3].reg_STATICWAITPAGE) == 0x270);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[3].reg_STATICWAITWR) == 0x274);
C_ASSERT(offsetof(lpc2478_emc_t, static_config[3].reg_STATICWAITTURN) == 0x278);

/**
 * grivera: Global declaration of const pointer to the LPC2478 SCB registers
 */
extern lpc2478_emc_t *const g_emc_mmio_registers_p;

#define EMCCONTROL_ENABLE       (1<<0)
#define EMCCONTROL_ADDR_MIRROR  (1<<1)
#define EMCCONTROL_LOW_PWR_MODE (1<<2)

#define EMCSTATUS_BUSY          (1<<0)
#define EMCSTATUS_S_DATA        (1<<1)
#define EMCSTATUS_SA            (1<<2)

#define EMCDYNAMICREADCONFIG_RD_CLKOUT      0
#define EMCDYNAMICREADCONFIG_RD_CMDDELAY    1
#define EMCDYNAMICREADCONFIG_RD_CMDDELAY1   2
#define EMCDYNAMICREADCONFIG_RD_CMDDELAY3   3

#define EMCDYNAMICRASCAS_RAS1   (1<<0)
#define EMCDYNAMICRASCAS_RAS2   (2<<0)
#define EMCDYNAMICRASCAS_RAS3   (3<<0)
#define EMCDYNAMICRASCAS_CAS1   (1<<8)
#define EMCDYNAMICRASCAS_CAS2   (2<<8)
#define EMCDYNAMICRASCAS_CAS3   (3<<8)

#define EMCDYNAMICCONTROL_CE        (1<<0)
#define EMCDYNAMICCONTROL_CS        (1<<1)
#define EMCDYNAMICCONTROL_SR        (1<<2)
#define EMCDYNAMICCONTROL_MMC       (1<<5)
#define EMCDYNAMICCONTROL_I_NORMAL  (0<<7)
#define EMCDYNAMICCONTROL_I_MODE    (1<<7)
#define EMCDYNAMICCONTROL_I_PALL    (2<<7)
#define EMCDYNAMICCONTROL_I_NOP     (3<<7)
#define EMCDYNAMICCONTROL_I_MASK    (3<<7)
#define EMCDYNAMICCONTROL_DP        (1<<13)

#define EMCDYNAMICCONFIG_MD_SDRAM       (0<<3)
#define EMCDYNAMICCONFIG_MD_LPSDRAM     (1<<3)
#define EMCDYNAMICCONFIG_MD_SYNCFLASH   (2<<3)
#define EMCDYNAMICCONFIG_AM_32BIT_HP_256MB_4BANKS_R13_C9  ((1<<14) | (0<<12) | (3<<9) | (1<<7))
#define EMCDYNAMICCONFIG_AM_32BIT_LP_256MB_4BANKS_R13_C9  ((1<<14) | (1<<12) | (3<<9) | (1<<7))
#define EMCDYNAMICCONFIG_BUFFER_ENABLE  (1<<19)
#define EMCDYNAMICCONFIG_WRITE_PROTECT  (1<<20)

#endif /* __EMC_H */
