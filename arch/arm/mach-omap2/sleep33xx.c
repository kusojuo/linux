/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ti_emif.h>
#include <linux/platform_data/emif_plat.h>
#include <linux/pie.h>

#include <asm/suspend.h>
#include <asm/cp15.h>
#include <asm/pie.h>

#include "pm33xx.h"
#include "cm33xx.h"
#include "cm-regbits-33xx.h"
#include "omap_hwmod.h"

#define CLKCTRL_IDLEST_FUNCTIONAL	0x0
#define CLKCTRL_IDLEST_DISABLED		0x3

struct emif_regs {
	u32 sdcfg;
	u32 ref_ctrl;
	u32 timing1;
	u32 timing2;
	u32 timing3;
	u32 pmcr;
	u32 pmcr_shdw;
	u32 zqcfg;
	u32 rd_lat;
};

extern int call_with_stack(int (*fn)(void *), void *arg, void *sp);
extern void v7_flush_dcache_all(void);

void (*__abs_v7_flush_dcache_all)(void) __pie_data(am33xx);
char sram_stack[1024] __pie_data(am33xx);
void __noreturn (*__cpu_resume_phys)(void) __pie_data(am33xx);
void __iomem *emif_virt_base __pie_data(am33xx);
void __iomem *dram_sync_addr __pie_data(am33xx);

EXPORT_PIE_SYMBOL(__abs_v7_flush_dcache_all);
EXPORT_PIE_SYMBOL(sram_stack);
EXPORT_PIE_SYMBOL(__cpu_resume_phys);
EXPORT_PIE_SYMBOL(emif_virt_base);
EXPORT_PIE_SYMBOL(dram_sync_addr);

static struct emif_regs emif_regs __pie_data(am33xx);
static void __iomem *emif_base __pie_data(am33xx);
static u32 mem_type __pie_data(am33xx);
static u32 cm_offset __pie_data(am33xx);

static struct pie_chunk *am33xx_chunk;

static inline void flush_dcache_all(void)
{
	__asm__ __volatile__("" : : : "r0", "r1", "r2", "r3", "r4", "r5",
				"r5", "r6", "r7", "r9", "r10", "r11");
	__abs_v7_flush_dcache_all();
}

static u32 __pie(am33xx) emif_read(u16 idx)
{
	return __raw_readl(emif_base + idx);
}

static void __pie(am33xx) emif_write(u32 val, u16 idx)
{
	__raw_writel(val, emif_base + idx);
}

static inline void am33xx_wkup_write(u32 val, void __iomem *reg)
{
	__raw_writel(val, reg + cm_offset);
}

static inline u32 am33xx_wkup_read(void __iomem *reg)
{
	return __raw_readl(reg + cm_offset);
}

static void __pie(am33xx) am33xx_module_set(u16 mode, void __iomem *reg)
{
	u32 val = am33xx_wkup_read(reg) & ~AM33XX_MODULEMODE_MASK;
	am33xx_wkup_write(val | mode, reg);
}

static void __pie(am33xx) am33xx_module_disable(void __iomem *reg)
{
	am33xx_module_set(0, reg);
}

static void __pie(am33xx) am33xx_module_disable_wait(void __iomem *reg)
{
	u32 val;
	am33xx_module_disable(reg);
	do {
		val = am33xx_wkup_read(reg) & AM33XX_IDLEST_MASK;
		val >>= AM33XX_IDLEST_SHIFT;
	} while (val != CLKCTRL_IDLEST_DISABLED);
}

static void __pie(am33xx) am33xx_module_enable(void __iomem *reg)
{
	am33xx_module_set(MODULEMODE_SWCTRL, reg);
}

static void __pie(am33xx) am33xx_module_enable_wait(void __iomem *reg)
{
	u32 val;
	am33xx_module_enable(reg);
	do {
		val = am33xx_wkup_read(reg) & AM33XX_IDLEST_MASK;
		val >>= AM33XX_IDLEST_SHIFT;
	} while (val != CLKCTRL_IDLEST_FUNCTIONAL);
}

static void __pie(am33xx) noinline am33xx_enable_sr(void)
{
	u32 val;

	emif_regs.sdcfg = emif_read(EMIF_SDRAM_CONFIG);
	val = emif_read(EMIF_POWER_MANAGEMENT_CONTROL);
	val &= ~SR_TIM_MASK;
	val |= 0xa << SR_TIM_SHIFT;
	emif_write(val, EMIF_POWER_MANAGEMENT_CONTROL);
	emif_write(val, EMIF_POWER_MANAGEMENT_CTRL_SHDW);

	__raw_readl(dram_sync_addr);
	val &= ~LP_MODE_MASK;
	val |= EMIF_LP_MODE_SELF_REFRESH << LP_MODE_SHIFT;
	emif_write(val, EMIF_POWER_MANAGEMENT_CONTROL);
}

static void __pie(am33xx) noinline am33xx_disable_sr(void)
{
	u32 val;

	val = emif_read(EMIF_POWER_MANAGEMENT_CONTROL);
	val &= ~LP_MODE_MASK;
	val |= EMIF_LP_MODE_DISABLE << LP_MODE_SHIFT;
	emif_write(val, EMIF_POWER_MANAGEMENT_CONTROL);
	emif_write(val, EMIF_POWER_MANAGEMENT_CTRL_SHDW);

	/*
	 * A write to SDRAM CONFIG register triggers
	 * an init sequence and hence it must be done
	 * at the end for DDR2
	 */
	emif_write(emif_regs.sdcfg, EMIF_SDRAM_CONFIG);
}

static void __pie(am33xx) noinline am33xx_emif_save(void)
{
	emif_regs.ref_ctrl = emif_read(EMIF_SDRAM_REFRESH_CONTROL);
	emif_regs.timing1 = emif_read(EMIF_SDRAM_TIMING_1);
	emif_regs.timing2 = emif_read(EMIF_SDRAM_TIMING_2);
	emif_regs.timing3 = emif_read(EMIF_SDRAM_TIMING_3);
	emif_regs.pmcr = emif_read(EMIF_POWER_MANAGEMENT_CONTROL);
	emif_regs.pmcr_shdw = emif_read(EMIF_POWER_MANAGEMENT_CTRL_SHDW);
	emif_regs.zqcfg = emif_read(EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG);
	emif_regs.rd_lat = emif_read(EMIF_DDR_PHY_CTRL_1);
}

static void __pie(am33xx) noinline am33xx_emif_restore(void)
{
	emif_write(emif_regs.rd_lat, EMIF_DDR_PHY_CTRL_1);
	emif_write(emif_regs.rd_lat, EMIF_DDR_PHY_CTRL_1_SHDW);
	emif_write(emif_regs.timing1, EMIF_SDRAM_TIMING_1);
	emif_write(emif_regs.timing1, EMIF_SDRAM_TIMING_1_SHDW);
	emif_write(emif_regs.timing2, EMIF_SDRAM_TIMING_2);
	emif_write(emif_regs.timing2, EMIF_SDRAM_TIMING_2_SHDW);
	emif_write(emif_regs.timing3, EMIF_SDRAM_TIMING_3);
	emif_write(emif_regs.timing3, EMIF_SDRAM_TIMING_3_SHDW);
	emif_write(emif_regs.ref_ctrl, EMIF_SDRAM_REFRESH_CONTROL);
	emif_write(emif_regs.ref_ctrl, EMIF_SDRAM_REFRESH_CTRL_SHDW);
	emif_write(emif_regs.pmcr, EMIF_POWER_MANAGEMENT_CONTROL);
	emif_write(emif_regs.pmcr_shdw, EMIF_POWER_MANAGEMENT_CTRL_SHDW);
	/*
	 * Output impedence calib needed only for DDR3
	 * but since the initial state of this will be
	 * disabled for DDR2 no harm in restoring the
	 * old configuration
	 */
	emif_write(emif_regs.zqcfg, EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG);

	/* Write to SDRAM_CONFIG only for DDR2 */
	if (mem_type == MEM_TYPE_DDR2)
		emif_write(emif_regs.sdcfg, EMIF_SDRAM_CONFIG);
}

int __pie(am33xx) am33xx_wfi_sram(void *data)
{
	mem_type = (unsigned long) data;
	emif_base = emif_virt_base;
	cm_offset = 0;

	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	flush_dcache_all();
	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data
	 * accesses strongly ordered and would not hit the cache.
	 */
	set_cr(get_cr() & ~CR_C);
	/*
	 * Invalidate L1 data cache. Even though only invalidate is
	 * necessary exported flush API is used here. Doing clean
	 * on already clean cache would be almost NOP.
	 */
	flush_dcache_all();

	am33xx_emif_save();
	am33xx_enable_sr();

	am33xx_module_disable_wait(AM33XX_CM_PER_EMIF_CLKCTRL);

	/*
	 * For the MPU WFI to be registered as an interrupt
	 * to WKUP_M3, MPU_CLKCTRL.MODULEMODE needs to be set
	 * to DISABLED
	 */
	am33xx_module_disable(AM33XX_CM_MPU_MPU_CLKCTRL);

	__asm__ __volatile__ (
		/*
		 * Execute an ISB instruction to ensure that all of the
		 * CP15 register changes have been committed.
		 */
		"isb\n\t"
		/*
		 * Execute a barrier instruction to ensure that all cache,
		 * TLB and branch predictor maintenance operations issued
		 * have completed.
		 */
		"dsb\n\t"
		"dmb\n\t"
		/*
		 * Execute a WFI instruction and wait until the
		 * STANDBYWFI output is asserted to indicate that the
		 * CPU is in idle and low power state. CPU can specualatively
		 * prefetch the instructions so add NOPs after WFI. Thirteen
		 * NOPs as per Cortex-A8 pipeline.
		 */
		"wfi\n\t"
		".rept 13\n\t"
		"nop\n\t"
		".endr" : : : "memory");

	/* We come here in case of an abort due to a late interrupt */

	am33xx_module_enable(AM33XX_CM_MPU_MPU_CLKCTRL);

	am33xx_module_enable_wait(AM33XX_CM_PER_EMIF_CLKCTRL);
	am33xx_disable_sr();
	/* Set SCTLR.C bit to allow data cache allocation */
	set_cr(get_cr() | CR_C);

	/* Let the suspend code know about the abort */
	return 1;
}
EXPORT_PIE_SYMBOL(am33xx_wfi_sram);

int am33xx_suspend(long unsigned int mem_type)
{
	pie_relocate_from_kern(am33xx_chunk);
	return call_with_stack(fn_to_pie(am33xx_chunk, &am33xx_wfi_sram),
			(void *) mem_type,
			kern_to_pie(am33xx_chunk, (char *) sram_stack) +
				sizeof(sram_stack));
}

static void __pie(am33xx) __noreturn noinline am33xx_resume(void)
{
	emif_base = (void *) AM33XX_EMIF_BASE;
	/* Undo the offset built into the register defines */
	cm_offset = -AM33XX_L4_WK_IO_OFFSET;

	am33xx_module_enable_wait(AM33XX_CM_PER_EMIF_CLKCTRL);
	am33xx_emif_restore();

	/* We are back. Branch to the common CPU resume routine */
	__cpu_resume_phys();
}

ARM_PIE_RESUME(am33xx, am33xx_resume, sram_stack + ARRAY_SIZE(sram_stack));

void am33xx_pie_init(struct pie_chunk *chunk, void __iomem *emif_base,
						void __iomem *dram_sync)
{
	am33xx_chunk = chunk;

	*kern_to_pie(chunk, &__abs_v7_flush_dcache_all) = v7_flush_dcache_all;
	*kern_to_pie(chunk, &__cpu_resume_phys) =
					(void *) virt_to_phys(cpu_resume);
	*kern_to_pie(chunk, &emif_virt_base) = emif_base;
	*kern_to_pie(chunk, &dram_sync_addr) = dram_sync;
}
