/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_PM33XX_H
#define __ARCH_ARM_MACH_OMAP2_PM33XX_H

#include <mach/hardware.h>	/* XXX Is this the right one to include? */
#include "control.h"

#ifndef __ASSEMBLER__
extern void __iomem *am33xx_get_ram_base(void);

/*
 * This enum is used to index the array passed to suspend routine with
 * parameters that vary across DDR2 and DDR3 sleep sequence.
 *
 * Since these are used to load into registers by suspend code,
 * entries here must always be in sync with the suspend code
 * in arm/mach-omap2/sleep33xx.S
 */
enum suspend_cfg_params {
	MEMORY_TYPE = 0,
	SUSP_VTP_CTRL_VAL,
	EVM_ID,
	CPU_REV,
	SLEEP_MODE,
	SUSPEND_CFG_PARAMS_END /* Must be the last entry */
};

struct a8_wkup_m3_ipc_data {
	int resume_addr;
	int sleep_mode;
	int ipc_data1;
	int ipc_data2;
} am33xx_lp_ipc;
#endif /* ASSEMBLER */

#define M3_TXEV_EOI			(AM33XX_CTRL_BASE + 0x1324)
#define A8_M3_IPC_REGS			(AM33XX_CTRL_BASE + 0x1328)
#define DS_RESUME_BASE			0x40300000
#define DS_IPC_DEFAULT			0xffffffff
#define M3_UMEM				0x44D00000

#define	DS0_ID				0x3
#define DS1_ID				0x5

#define M3_STATE_UNKNOWN		-1
#define M3_STATE_RESET			0
#define M3_STATE_INITED			1
#define M3_STATE_MSG_FOR_LP		2
#define M3_STATE_MSG_FOR_RESET		3

#define VTP_CTRL_ENABLE		(0x1 << 6)

#define DDR_IO_CTRL		(AM33XX_CTRL_BASE + 0x0E04)
#define VTP0_CTRL_REG		(AM33XX_CTRL_BASE + 0x0E0C)
#define DDR_CMD0_IOCTRL		(AM33XX_CTRL_BASE + 0x1404)
#define DDR_CMD1_IOCTRL		(AM33XX_CTRL_BASE + 0x1408)
#define DDR_CMD2_IOCTRL		(AM33XX_CTRL_BASE + 0x140C)
#define DDR_DATA0_IOCTRL	(AM33XX_CTRL_BASE + 0x1440)
#define DDR_DATA1_IOCTRL	(AM33XX_CTRL_BASE + 0x1444)

#define OMAP_RTC_SECONDS_REG		0x00
#define OMAP_RTC_STATUS_REG		0x44
#define OMAP_RTC_INTERRUPTS_REG		0x48
#define OMAP_RTC_ALARM2_SECONDS_REG	0x80
#define OMAP_RTC_PMIC_REG		0x98
#define OMAP_RTC_STATUS_ALARM2		(1<<7)
#define OMAP_RTC_STATUS_BUSY		(1<<0)
#define OMAP_RTC_INTERRUPTS_IT_ALARM2	(1<<4)
#define OMAP_RTC_PMIC_POWER_EN_EN	(1<<16)

#define MEM_TYPE_DDR2		2

#define SUSP_VTP_CTRL_DDR2	0x10117
#define SUSP_VTP_CTRL_DDR3	0x0

#define CPU_REV_1		1
#define CPU_REV_2		2

#define M3_VERSION_UNKNOWN		0x0000ffff

#define AM33XX_SLEEP_STANDBY	0
#define AM33XX_SLEEP_DEEPSLEEP2	1
#define AM33XX_SLEEP_DEEPSLEEP1	2
#define AM33XX_SLEEP_DEEPSLEEP0	3
#define AM33XX_SLEEP_RTC_ONLY	4
#define AM33XX_SLEEP_END	5

#define RTC_SCRATCH_RESUME_REG		0
#define RTC_SCRATCH_MAGIC_REG		1

#define RTC_REG_BOOT_MAGIC		0x8cd0 /* RTC */
#define RTC_REG_DDR_TYPE_UNK		(0x00 << 16)
#define RTC_REG_DDR_TYPE_DDR2_0		(0x01 << 16)
#define RTC_REG_DDR_TYPE_DDR3_0		(0x02 << 16)
#define RTC_REG_BOARD_ID_UNK		(0x00 << 24)
#define RTC_REG_BOARD_ID_BONE		(0x01 << 24)
#define RTC_REG_BOARD_ID_BONE_LT	(0x02 << 24)
#define RTC_REG_BOARD_ID_EVM_SK		(0x03 << 24)
#define RTC_REG_BOARD_ID_IDK		(0x04 << 24)

#endif
