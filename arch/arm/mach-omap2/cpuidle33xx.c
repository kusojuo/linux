/*
 * AM33XX CPU idle Routines
 *
 * Copyright (C) 2011-2013 Texas Instruments, Inc.
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 * Rajendra Nayak <rnayak@ti.com>
 * Russ Dill <russ.dill@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>

#include <asm/cpuidle.h>

#include "common.h"
#include "pm33xx.h"
#include "powerdomain.h"

struct am33xx_idle_statedata {
	bool use_m3;
	bool self_refresh;

	u8 mpu_state;
	u8 mpu_ram_ret_state;
	u8 mpu_l1_ret_state;
	u8 mpu_l2_ret_state;
};

struct am33xx_idle_statedata am33xx_idle_data[] = {
	{
		.mpu_state		= PWRDM_POWER_ON,
	},
	{
		.self_refresh		= true,
		.mpu_state		= PWRDM_POWER_ON,
	},
	{
		.self_refresh		= true,
		.use_m3			= true,
		.mpu_state		= PWRDM_POWER_ON,
	},
	{
		.self_refresh		= true,
		.use_m3			= true,
		.mpu_state		= PWRDM_POWER_RET,
		.mpu_ram_ret_state	= MEM_BANK_RET_ST_OFF,
		.mpu_l1_ret_state	= MEM_BANK_RET_ST_RET,
		.mpu_l2_ret_state	= MEM_BANK_RET_ST_RET,
	},
	{
		.self_refresh		= true,
		.use_m3			= true,
		.mpu_state		= PWRDM_POWER_RET,
		.mpu_ram_ret_state	= MEM_BANK_RET_ST_OFF,
		.mpu_l1_ret_state	= MEM_BANK_RET_ST_OFF,
		.mpu_l2_ret_state	= MEM_BANK_RET_ST_OFF,
	},
	{
		.self_refresh		= true,
		.use_m3			= true,
		.mpu_state		= PWRDM_POWER_OFF,
	},
};

static int am33xx_enter_idle(struct cpuidle_device *dev,
					struct cpuidle_driver *drv, int index)
{
	struct am33xx_idle_statedata *cx = &am33xx_idle_data[index];
	u32 wfi_flags = 0;
	u32 m3_flags;

	if (omap_irq_pending() || need_resched())
		goto out;

	if (cx->self_refresh)
		wfi_flags |= WFI_SELF_REFRESH;

	if (cx->use_m3)
		wfi_flags |= WFI_WAKE_M3;

	if (cx->mpu_state != PWRDM_POWER_ON) {
		/*
		 * Call idle CPU PM enter notifier chain so that VFP
		 * context is saved.
		 */
		cpu_pm_enter();
		wfi_flags |= WFI_SAVE_MPU;
	}

	m3_flags = cx->mpu_state << M3_PARAM2_MPU_STATE_SHIFT |
			cx->mpu_ram_ret_state << M3_PARAM2_MPU_RAM_RET_SHIFT |
			cx->mpu_l1_ret_state << M3_PARAM2_MPU_L1_RET_SHIFT |
			cx->mpu_l2_ret_state << M3_PARAM2_MPU_L2_RET_SHIFT |
			PWRDM_POWER_ON << M3_PARAM2_PER_STATE_SHIFT |
			MEM_BANK_ON_ST_ON << M3_PARAM2_PER_ICSS_ON_SHIFT |
			MEM_BANK_ON_ST_ON << M3_PARAM2_PER_MEM_ON_SHIFT |
			MEM_BANK_ON_ST_ON << M3_PARAM2_PER_OCMC_SHIFT |
			MPU_WAKE << M3_PARAM2_WAKE_SOURCES_SHIFT;

	am33xx_do_sram_cpuidle(wfi_flags, m3_flags);

	if (cx->mpu_state != PWRDM_POWER_ON)
		cpu_pm_exit();

out:
	return index;
}

static struct cpuidle_driver am33xx_idle_driver = {
	.name		= "am33xx_idle",
	.owner		= THIS_MODULE,
	.states = {
		ARM_CPUIDLE_WFI_STATE,
		{
			.exit_latency = 50 + 50,
			.target_residency = 100,
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.enter = am33xx_enter_idle,
			.name = "C1",
			.desc = "DDR SR"
		},
		{
			.exit_latency = 100 + 100,
			.target_residency = 100,
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.enter = am33xx_enter_idle,
			.name = "C2",
			.desc = "DDR SR + MPU PLL bypass"
		},
		{
			.exit_latency = 500 + 500,
			.target_residency = 1000,
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.enter = am33xx_enter_idle,
			.name = "C3",
			.desc = "MPU RET"
		},
		{
			.exit_latency = 600 + 600,
			.target_residency = 1200,
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.enter = am33xx_enter_idle,
			.name = "C4",
			.desc = "MPU RET + L1/L2 OFF"
		},
		{
			.exit_latency = 800 + 800,
			.target_residency = 1600,
			.flags = CPUIDLE_FLAG_TIME_VALID,
			.enter = am33xx_enter_idle,
			.name = "C5",
			.desc = "MPU OFF"
		},
	},
	.state_count = ARRAY_SIZE(am33xx_idle_data),
	.safe_state_index = 0,
};

/**
 * am33xx_idle_init - Init routine for am33xx idle
 *
 * Registers the am33xx specific cpuidle driver to the cpuidle
 * framework with the valid set of states.
 */
int __init am33xx_idle_init(void)
{
	return cpuidle_register(&am33xx_idle_driver, NULL);
}
