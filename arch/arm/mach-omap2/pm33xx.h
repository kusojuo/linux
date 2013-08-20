/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Inc.
 * Vaibhav Bedia <vaibhav.bedia@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ARCH_ARM_MACH_OMAP2_PM33XX_H
#define __ARCH_ARM_MACH_OMAP2_PM33XX_H

#include "control.h"

#ifndef __ASSEMBLER__

struct am33xx_pm_context {
	struct am33xx_ipc_data	ipc;
	struct firmware		*firmware;
	struct omap_mbox	*mbox;
	u8			state;
	u32			ver;
};

/*
 * Params passed to suspend routine
 *
 * Since these are used to load into registers by suspend code,
 * entries here must always be in sync with the suspend code
 * in arm/mach-omap2/sleep33xx.S
 */
struct am33xx_suspend_params {
	void __iomem *emif_addr_virt;
	u32 wfi_flags;
	void __iomem *dram_sync;
};

struct wakeup_src {
	int irq_nr;
	char src[10];
};

struct forced_standby_module {
	char oh_name[15];
	struct device *dev;
};

int wkup_m3_copy_code(const u8 *data, size_t size);
int wkup_m3_prepare(void);
void wkup_m3_register_txev_handler(void (*txev_handler)(void));
int am33xx_do_sram_cpuidle(u32, u32);

#endif

#define	IPC_CMD_DS0			0x4
#define	IPC_CMD_IDLE			0xd
#define IPC_CMD_RESET                   0xe
#define DS_IPC_DEFAULT			0xffffffff
#define M3_VERSION_UNKNOWN		0x0000ffff
#define M3_BASELINE_VERSION		0x21

#define M3_STATE_UNKNOWN		0
#define M3_STATE_RESET			1
#define M3_STATE_INITED			2
#define M3_STATE_MSG_FOR_LP		3
#define M3_STATE_MSG_FOR_RESET		4

#define AM33XX_OCMC_END			0x40310000
#define AM33XX_EMIF_BASE		0x4C000000

#define MEM_TYPE_DDR2		2
#define MEM_TYPE_DDR3		3

#define WFI_MEM_TYPE_DDR2	(1 << 0)
#define WFI_MEM_TYPE_DDR3	(1 << 1)
#define WFI_SELF_REFRESH	(1 << 2)
#define WFI_SAVE_EMIF		(1 << 3)
#define WFI_WAKE_M3		(1 << 4)
#define WFI_SAVE_MPU		(1 << 5)

#define MEM_BANK_RET_ST_OFF	0x0
#define MEM_BANK_RET_ST_RET	0x1

#define MEM_BANK_ON_ST_OFF	0x0
#define MEM_BANK_ON_ST_RET	0x1
#define MEM_BANK_ON_ST_ON	0x3

#define MPU_WAKE		0x800

#define M3_PARAM1_MOSC_STATE_SHIFT	0
#define M3_PARAM1_MOSC_STATE_MASK	(0x1 << 0)
#define M3_PARAM1_OSC_CYCLES_SHIFT	1
#define M3_PARAM1_OSC_CYCLES_MASK	(0xffff << 1)
#define M3_PARAM1_MPU_VOLT_SHIFT	17
#define M3_PARAM1_MPU_VOLT_MASK		(0x7fff << 17)

#define M3_PARAM2_MPU_STATE_SHIFT	0
#define M3_PARAM2_MPU_STATE_MASK	(0x3 << 0)
#define M3_PARAM2_MPU_RAM_RET_SHIFT	2
#define M3_PARAM2_MPU_RAM_RET_MASK	(0x1 << 2)
#define M3_PARAM2_MPU_L1_RET_SHIFT	3
#define M3_PARAM2_MPU_L1_RET_MASK	(0x1 << 3)
#define M3_PARAM2_MPU_L2_RET_SHIFT	4
#define M3_PARAM2_MPU_L2_RET_MASK	(0x1 << 4)
#define M3_PARAM2_MPU_RAM_ON_SHIFT	5
#define M3_PARAM2_MPU_RAM_ON_MASK	(0x3 << 5)
#define M3_PARAM2_PER_STATE_SHIFT	7
#define M3_PARAM2_PER_STATE_MASK	(0x3 << 7)
#define M3_PARAM2_PER_ICSS_RET_SHIFT	9
#define M3_PARAM2_PER_ICSS_RET_MASK	(0x1 << 9)
#define M3_PARAM2_PER_MEM_RET_SHIFT	10
#define M3_PARAM2_PER_MEM_RET_MASK	(0x1 << 10)
#define M3_PARAM2_PER_OCMC_RET_SHIFT	11
#define M3_PARAM2_PER_OCMC_RET_MASK	(0x1 << 11)
#define M3_PARAM2_PER_ICSS_ON_SHIFT	12
#define M3_PARAM2_PER_ICSS_ON_MASK	(0x3 << 12)
#define M3_PARAM2_PER_MEM_ON_SHIFT	14
#define M3_PARAM2_PER_MEM_ON_MASK	(0x3 << 14)
#define M3_PARAM2_PER_OCMC_SHIFT	16
#define M3_PARAM2_PER_OCMC_MASK		(0x3 << 16)
#define M3_PARAM2_WAKE_SOURCES_SHIFT	18
#define M3_PARAM2_WAKE_SOURCES_MASK	(0x1fff << 18)

#endif
