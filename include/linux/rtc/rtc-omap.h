/*
 * RTC-OMAP driver platform header
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _LINUX_RTC_OMAP_H_
#define _LINUX_RTC_OMAP_H_

/**
 * struct omap_rtc_pdata - OMAP rtc Platform Data
 *
 * @pm_off:			Flag to specify whether the RTC is incharge of
 *				controlling the system/device power.
 * @wakeup_capable:		module supports wakeup from suspend with alarm
 *				events
 * @ext_wakeup_enable:		Use ext_wakeup signal to create wake events
 * @ext_wakeup_invert:		0 = Active high, 1 = Active low
 * @ext_wakeup_debounce_en:	Enable debouncing of ext_wakeup signal
 * @ext_wakeup_debounce_taps:	Debounce timer, in units of 30.52 us.
 *				debounce time = 30.52 us * (taps + 1)
 */
struct omap_rtc_pdata {
	bool pm_off;
	unsigned wakeup_capable:1;
	unsigned ext_wakeup_enable:1;
	unsigned ext_wakeup_invert:1;
	unsigned ext_wakeup_debounce_en:1;
	u8 ext_wakeup_debounce_taps;
};
#endif
