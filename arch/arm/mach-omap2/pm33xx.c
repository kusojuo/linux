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
#include <linux/init.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/mailbox.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_data/gpio-omap.h>
#include <linux/reboot.h>

#include <asm/suspend.h>
#include <asm/proc-fns.h>
#include <asm/sizes.h>
#include <asm/fncpy.h>
#include <asm/system_misc.h>

#include "pm.h"
#include "cm33xx.h"
#include "pm33xx.h"
#include "control.h"
#include "clockdomain.h"
#include "powerdomain.h"
#include "omap_hwmod.h"
#include "omap_device.h"
#include "soc.h"
#include "sram.h"

void (*am33xx_do_wfi_sram)(void);
static void wkup_m3_reinitialize(void);

static void __iomem *am33xx_emif_base;
static struct powerdomain *gfx_pwrdm, *per_pwrdm;
static struct clockdomain *gfx_l4ls_clkdm;
static struct omap_hwmod *usb_oh, *cpsw_oh, *tptc0_oh, *tptc1_oh, *tptc2_oh;
static struct wkup_m3_context *wkup_m3;
static struct pinctrl_dev *pmx_dev;
static u32 gmii_sel;

static DECLARE_COMPLETION(wkup_m3_sync);

#if defined(CONFIG_SUSPEND) || defined(CONFIG_HIBERNATION)
static int am33xx_per_save_context(void)
{
	/*
	 * Erratum 1.0.14 - 'GMII_SEL and CPSW Related Pad Control Registers:
	 * Context of These Registers is Lost During Transitions of PD_PER'
	 *
	 * Save the padconf registers in the PER partition as well as the
	 * GMII_SEL register
	 */

	gmii_sel = readl(AM33XX_CTRL_REGADDR(AM33XX_CONTROL_GMII_SEL_OFFSET));

	return pinmux_save_context(pmx_dev, "am33xx_pmx_per");
}

static void am33xx_per_restore_context(void)
{
	/* Restore for Erratum 1.0.14 */
	writel(gmii_sel, AM33XX_CTRL_REGADDR(AM33XX_CONTROL_GMII_SEL_OFFSET));

	pinmux_restore_context(pmx_dev, "am33xx_pmx_per");
}

static int am33xx_wkup_save_context(void)
{
	int ret;

	ret = pinmux_save_context(pmx_dev, "am33xx_pmx_wkup");
	if (ret < 0)
		return ret;

	omap_intc_save_context();
	am33xx_control_save_context();

	clks_save_context();
	pwrdms_save_context();
	omap_hwmods_save_context();
	clkdm_save_context();
	omap_sram_save_context();

	return 0;
}

static void am33xx_wkup_restore_context(void)
{
	clks_restore_context();
	pwrdms_restore_context();
	clkdm_restore_context();
	omap_hwmods_restore_context();
	am33xx_control_restore_context();
	pinmux_restore_context(pmx_dev, "am33xx_pmx_wkup");
	omap_intc_restore_context();
	wkup_m3_reinitialize();
	omap_sram_restore_context();
}
#endif /* CONFIG_SUSPEND || CONFIG_HIBERNATION */

#ifdef CONFIG_SUSPEND
static int am33xx_do_sram_idle(long unsigned int unused)
{
	am33xx_do_wfi_sram();
	return 0;
}

static int am33xx_pm_suspend(void)
{
	int status, ret = 0;

	/*
	 * By default the following IPs do not have MSTANDBY asserted
	 * which is necessary for PER domain transition. If the drivers
	 * are not compiled into the kernel HWMOD code will not change the
	 * state of the IPs if the IP was not never enabled. To ensure
	 * that there no issues with or without the drivers being compiled
	 * in the kernel, we forcefully put these IPs to idle.
	 */
	omap_hwmod_enable(usb_oh);
	omap_hwmod_enable(tptc0_oh);
	omap_hwmod_enable(tptc1_oh);
	omap_hwmod_enable(tptc2_oh);
	omap_hwmod_enable(cpsw_oh);

	omap_hwmod_idle(usb_oh);
	omap_hwmod_idle(tptc0_oh);
	omap_hwmod_idle(tptc1_oh);
	omap_hwmod_idle(tptc2_oh);
	omap_hwmod_idle(cpsw_oh);

	/* Try to put GFX to sleep */
	pwrdm_set_next_fpwrst(gfx_pwrdm, PWRDM_FUNC_PWRST_OFF);

	ret = cpu_suspend(0, am33xx_do_sram_idle);

	status = pwrdm_read_fpwrst(gfx_pwrdm);
	if (status != PWRDM_FUNC_PWRST_OFF)
		pr_err("GFX domain did not transition\n");
	else
		pr_info("GFX domain entered low power state\n");

	/*
	 * GFX_L4LS clock domain needs to be woken up to
	 * ensure thet L4LS clock domain does not get stuck in transition
	 * If that happens L3 module does not get disabled, thereby leading
	 * to PER power domain transition failing
	 *
	 * The clock framework should take care of ensuring
	 * that the clock domain is in the right state when
	 * GFX driver is active.
	 */
	clkdm_wakeup(gfx_l4ls_clkdm);
	clkdm_sleep(gfx_l4ls_clkdm);

	if (ret) {
		pr_err("Kernel suspend failure\n");
	} else {
		status = omap_ctrl_readl(AM33XX_CONTROL_IPC_MSG_REG1);
		status &= IPC_RESP_MASK;
		status >>= __ffs(IPC_RESP_MASK);

		switch (status) {
		case 0:
			pr_info("Successfully put all powerdomains to target state\n");
			/*
			 * XXX: Leads to loss of logic state in PER power domain
			 * Use SOC specific ops for this?
			 */
			break;
		case 1:
			pr_err("Could not transition all powerdomains to target state\n");
			ret = -1;
			break;
		default:
			pr_err("Something went wrong :(\nStatus = %d\n",
				status);
			ret = -1;
		}
	}

	return ret;
}

static int am33xx_pm_prepare(void)
{
	return am33xx_per_save_context();
}

static void am33xx_pm_finish(void)
{
	am33xx_per_restore_context();
}

static int am33xx_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = am33xx_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int am33xx_pm_begin(suspend_state_t state)
{
	int ret = 0;
	struct mailbox_msg dummy_msg;

	disable_hlt();

	MAILBOX_FILL_HEADER_MSG(dummy_msg, 0xABCDABCD);

	wkup_m3->ipc_data.sleep_mode = IPC_CMD_DS0;
	wkup_m3->ipc_data.param1  = DS_IPC_DEFAULT;
	wkup_m3->ipc_data.param2  = DS_IPC_DEFAULT;

	am33xx_wkup_m3_ipc_cmd(&wkup_m3->ipc_data);

	wkup_m3->state = M3_STATE_MSG_FOR_LP;

	mailbox_enable_irq(wkup_m3->mbox, IRQ_RX);

	ret = mailbox_msg_send(wkup_m3->mbox, &dummy_msg);
	if (ret) {
		pr_err("A8<->CM3 MSG for LP failed\n");
		am33xx_m3_state_machine_reset();
		ret = -1;
	}

	/* Give some time to M3 to respond. 500msec is a random value here */
	if (!wait_for_completion_timeout(&wkup_m3_sync,
					msecs_to_jiffies(500))) {
		pr_err("A8<->CM3 sync failure\n");
		am33xx_m3_state_machine_reset();
		ret = -1;
	} else {
		pr_debug("Message sent for entering DeepSleep mode\n");
		mailbox_disable_irq(wkup_m3->mbox, IRQ_RX);
	}

	return ret;
}

static void am33xx_pm_end(void)
{
	mailbox_enable_irq(wkup_m3->mbox, IRQ_RX);

	am33xx_m3_state_machine_reset();

	enable_hlt();

	return;
}

static const struct platform_suspend_ops am33xx_pm_ops = {
	.begin		= am33xx_pm_begin,
	.end		= am33xx_pm_end,
	.enter		= am33xx_pm_enter,
	.prepare	= am33xx_pm_prepare,
	.finish		= am33xx_pm_finish,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */

#ifdef CONFIG_HIBERNATION
static int am33xx_hibernation_begin(void)
{
	disable_hlt();
	return 0;
}

static int am33xx_hibernation_pre_snapshot(void)
{
	am33xx_per_save_context();
	omap2_gpio_prepare_for_idle(1);
	am33xx_wkup_save_context();

	return 0;
}

static void am33xx_hibernation_leave(void)
{
	pwrdms_lost_power();
	am33xx_wkup_restore_context();
}

static void am33xx_hibernation_finish(void)
{
	omap2_gpio_resume_after_idle();
	am33xx_per_restore_context();
}

static void am33xx_hibernation_end(void)
{
	enable_hlt();
}

static int am33xx_hibernation_prepare(void)
{
	return 0;
}

static int am33xx_hibernation_enter(void)
{
	machine_power_off();
	return 0;
}

static int am33xx_hibernation_pre_restore(void)
{
	omap2_gpio_prepare_for_idle(1);
	return 0;
}

static void am33xx_hibernation_restore_cleanup(void)
{
	omap2_gpio_resume_after_idle();
}

static const struct platform_hibernation_ops am33xx_hibernation_ops = {
	.begin			= am33xx_hibernation_begin,
	.end			= am33xx_hibernation_end,
	.pre_snapshot		= am33xx_hibernation_pre_snapshot,
	.finish			= am33xx_hibernation_finish,
	.prepare		= am33xx_hibernation_prepare,
	.enter			= am33xx_hibernation_enter,
	.leave			= am33xx_hibernation_leave,
	.pre_restore		= am33xx_hibernation_pre_restore,
	.restore_cleanup	= am33xx_hibernation_restore_cleanup,
};
#endif /* CONFIG_HIBERNATION */

#if defined(CONFIG_SUSPEND) || defined(CONFIG_HIBERNATION)
static void wkup_m3_reinitialize(void)
{
	struct platform_device *pdev = to_platform_device(wkup_m3->dev);
	int ret;

	wkup_m3->state = M3_STATE_UNKNOWN;

	ret = omap_device_assert_hardreset(pdev, "wkup_m3");
	if (ret < 0)
		goto err;

	memcpy(wkup_m3->code, wkup_m3->firmware->data, wkup_m3->firmware->size);
	wkup_m3->state = M3_STATE_RESET;

	ret = omap_device_deassert_hardreset(pdev, "wkup_m3");
	if (ret < 0)
		goto err;

	return;

err:
	pr_err("Could not restore M3 context\n");
}
#endif /* CONFIG_SUSPEND || CONFIG_HIBERNATION */

static void am33xx_m3_state_machine_reset(void)
{
	int ret = 0;
	struct mailbox_msg dummy_msg;

	MAILBOX_FILL_HEADER_MSG(dummy_msg, 0xABCDABCD);

	wkup_m3->ipc_data.sleep_mode	= IPC_CMD_RESET;
	wkup_m3->ipc_data.param1	= DS_IPC_DEFAULT;
	wkup_m3->ipc_data.param2	= DS_IPC_DEFAULT;

	am33xx_wkup_m3_ipc_cmd(&wkup_m3->ipc_data);

	wkup_m3->state = M3_STATE_MSG_FOR_RESET;

	ret = mailbox_msg_send(wkup_m3->mbox, &dummy_msg);
	if (!ret) {
		pr_debug("Message sent for resetting M3 state machine\n");
		/* Give some to M3 to respond. 500msec is a random value here */
		if (!wait_for_completion_timeout(&wkup_m3_sync,
						msecs_to_jiffies(500)))
			pr_err("A8<->CM3 sync failure\n");
	} else {
		pr_err("Could not reset M3 state machine!!!\n");
		wkup_m3->state = M3_STATE_UNKNOWN;
	}
}

/*
 * Dummy notifier for the mailbox
 * XXX: Get rid of this requirement once the MBX driver has been finalized
 */
static int wkup_mbox_msg(struct notifier_block *self, unsigned long len,
		void *msg)
{
	return 0;
}

static struct notifier_block wkup_mbox_notifier = {
	.notifier_call = wkup_mbox_msg,
};

static irqreturn_t wkup_m3_txev_handler(int irq, void *unused)
{
	am33xx_txev_eoi();

	switch (wkup_m3->state) {
	case M3_STATE_RESET:
		wkup_m3->state = M3_STATE_INITED;
		break;
	case M3_STATE_MSG_FOR_RESET:
		wkup_m3->state = M3_STATE_INITED;
		mailbox_rx_flush(wkup_m3->mbox);
		complete(&wkup_m3_sync);
		break;
	case M3_STATE_MSG_FOR_LP:
		mailbox_rx_flush(wkup_m3->mbox);
		complete(&wkup_m3_sync);
		break;
	case M3_STATE_UNKNOWN:
		pr_err("IRQ %d with WKUP_M3 in unknown state\n", irq);
		mailbox_rx_flush(wkup_m3->mbox);
		return IRQ_NONE;
	}

	am33xx_txev_enable();
	return IRQ_HANDLED;
}

static void am33xx_pm_firmware_cb(const struct firmware *fw, void *context)
{
	struct wkup_m3_context *wkup_m3_context = context;
	struct platform_device *pdev = to_platform_device(wkup_m3_context->dev);
	int ret = 0;

	/* no firmware found */
	if (!fw) {
		dev_err(wkup_m3_context->dev, "request_firmware failed\n");
		goto err;
	}

	memcpy((void *)wkup_m3_context->code, fw->data, fw->size);
	pr_info("Copied the M3 firmware to UMEM\n");

	wkup_m3->state = M3_STATE_RESET;
	wkup_m3->firmware = fw;

	ret = omap_device_deassert_hardreset(pdev, "wkup_m3");
	if (ret) {
		pr_err("Could not deassert the reset for WKUP_M3\n");
		goto err;
	} else {
#ifdef CONFIG_SUSPEND
		suspend_set_ops(&am33xx_pm_ops);
		/*
		 * Physical resume address to be used by ROM code
		 */
		wkup_m3->ipc_data.resume_addr = (AM33XX_OCMC_END -
				am33xx_do_wfi_sz + am33xx_resume_offset + 0x4);
#endif
		return;
	}

err:
	mailbox_put(wkup_m3_context->mbox, &wkup_mbox_notifier);
}

static int wkup_m3_init(void)
{
	int irq, ret = 0;
	struct resource *mem;
	struct platform_device *pdev = to_platform_device(wkup_m3->dev);

	omap_device_enable_hwmods(to_omap_device(pdev));

	/* Reserve the MBOX for sending messages to M3 */
	wkup_m3->mbox = mailbox_get("wkup_m3", &wkup_mbox_notifier);
	if (IS_ERR(wkup_m3->mbox)) {
		pr_err("Could not reserve mailbox for A8->M3 IPC\n");
		ret = -ENODEV;
		goto exit;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(wkup_m3->dev, "no irq resource\n");
		ret = -ENXIO;
		goto err;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(wkup_m3->dev, "no memory resource\n");
		ret = -ENXIO;
		goto err;
	}

	wkup_m3->code = devm_request_and_ioremap(wkup_m3->dev, mem);
	if (!wkup_m3->code) {
		dev_err(wkup_m3->dev, "could not ioremap\n");
		ret = -EADDRNOTAVAIL;
		goto err;
	}

	ret = devm_request_irq(wkup_m3->dev, irq, wkup_m3_txev_handler,
		  IRQF_DISABLED, "wkup_m3_txev", NULL);
	if (ret) {
		dev_err(wkup_m3->dev, "request_irq failed\n");
		goto err;
	} else {
		am33xx_txev_enable();
	}

	pr_info("Trying to load am335x-pm-firmware.bin");

	/* We don't want to delay boot */
	request_firmware_nowait(THIS_MODULE, 0, "am335x-pm-firmware.bin",
				wkup_m3->dev, GFP_KERNEL, wkup_m3,
				am33xx_pm_firmware_cb);
	return 0;

err:
	mailbox_put(wkup_m3->mbox, &wkup_mbox_notifier);
exit:
	return ret;
}

/*
 * Push the minimal suspend-resume code to SRAM
 */
static void am33xx_push_sram_idle(void)
{
	am33xx_do_wfi_sram = (void *)omap_sram_push
					(am33xx_do_wfi, am33xx_do_wfi_sz);
}

static int __init am33xx_map_emif(void)
{
	am33xx_emif_base = ioremap(AM33XX_EMIF_BASE, SZ_32K);

	if (!am33xx_emif_base)
		return -ENOMEM;

	return 0;
}

void __iomem *am33xx_get_emif_base(void)
{
	return am33xx_emif_base;
}

int __init am33xx_setup_deep_sleep(void)
{
	int ret;

	if (!am33xx_get_emif_base())
		return -ENODEV;

	if (!pmx_dev)
		return -ENODEV;

	/*
	 * By default the following IPs do not have MSTANDBY asserted
	 * which is necessary for PER domain transition. If the drivers
	 * are not compiled into the kernel HWMOD code will not change the
	 * state of the IPs if the IP was not never enabled
	 */
	usb_oh		= omap_hwmod_lookup("usb_otg_hs");
	tptc0_oh	= omap_hwmod_lookup("tptc0");
	tptc1_oh	= omap_hwmod_lookup("tptc1");
	tptc2_oh	= omap_hwmod_lookup("tptc2");
	cpsw_oh		= omap_hwmod_lookup("cpgmac0");

	gfx_pwrdm = pwrdm_lookup("gfx_pwrdm");
	per_pwrdm = pwrdm_lookup("per_pwrdm");

	gfx_l4ls_clkdm = clkdm_lookup("gfx_l4ls_gfx_clkdm");

	if ((!usb_oh) || (!tptc0_oh) || (!tptc1_oh) || (!tptc2_oh) ||
		(!cpsw_oh) || (!gfx_pwrdm) || (!per_pwrdm) ||
		(!gfx_l4ls_clkdm))
		return -ENODEV;

	wkup_m3 = kzalloc(sizeof(struct wkup_m3_context), GFP_KERNEL);
	if (!wkup_m3) {
		pr_err("Memory allocation failed\n");
		return -ENOMEM;
	}

	wkup_m3->dev = omap_device_get_by_hwmod_name("wkup_m3");

	ret = wkup_m3_init();
	if (ret) {
		kfree(wkup_m3);
		pr_err("Could not initialise firmware loading\n");
	}

	return ret;
}

int __init am33xx_pm_init(void)
{
	struct powerdomain *cefuse_pwrdm;
	int ret;

	if (!soc_is_am33xx())
		return -ENODEV;

	pr_info("Power Management for AM33XX family\n");

	am33xx_push_sram_idle();

	ret = am33xx_map_emif();
	if (ret)
		pr_err("Could not ioremap EMIF\n");

	(void) clkdm_for_each(omap_pm_clkdms_setup, NULL);

	/* CEFUSE domain can be turned off post bootup */
	cefuse_pwrdm = pwrdm_lookup("cefuse_pwrdm");
	if (cefuse_pwrdm)
		pwrdm_set_next_fpwrst(cefuse_pwrdm, PWRDM_FUNC_PWRST_OFF);
	else
		pr_err("Failed to get cefuse_pwrdm\n");

	pmx_dev = get_pinctrl_dev_from_devname("44e10800.pinmux");
	if (!pmx_dev)
		pr_err("Could not find pin mux for saving context");

	ret = am33xx_setup_deep_sleep();
	if (ret < 0)
		pr_err("AM33XX deep sleep modes unavailable\n");

#ifdef CONFIG_HIBERNATION
	if (pmx_dev)
		hibernation_set_ops(&am33xx_hibernation_ops);
#endif /* CONFIG_HIBERNATION */

	return 0;
}
