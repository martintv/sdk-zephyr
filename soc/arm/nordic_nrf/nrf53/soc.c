/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for Nordic Semiconductor nRF53 family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Nordic Semiconductor nRF53 family processor.
 */

#include <kernel.h>
#include <init.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <soc/nrfx_coredep.h>
#include <logging/log.h>
#include <nrf_erratas.h>
#include <hal/nrf_power.h>
#if defined(CONFIG_SOC_NRF5340_CPUAPP)
#include <hal/nrf_cache.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_oscillators.h>
#include <hal/nrf_regulators.h>
#elif defined(CONFIG_SOC_NRF5340_CPUNET)
#include <hal/nrf_nvmc.h>
#endif
#include <soc_secure.h>

#define PIN_XL1 0
#define PIN_XL2 1

#ifdef CONFIG_RUNTIME_NMI
extern void z_arm_nmi_init(void);
#define NMI_INIT() z_arm_nmi_init()
#else
#define NMI_INIT()
#endif

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
#include <system_nrf5340_application.h>
#elif defined(CONFIG_SOC_NRF5340_CPUNET)
#include <system_nrf5340_network.h>
#else
#error "Unknown nRF53 SoC."
#endif

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

#if defined(CONFIG_SOC_NRF53_ANOMALY_160_WORKAROUND)
static void nrf53_anomaly_160_workaround(void)
{
	/* This part is supposed to be removed once the writes are available
	 * in hal_nordic/nrfx/MDK.
	 */
#if defined(CONFIG_SOC_NRF5340_CPUAPP) && !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
	*((volatile uint32_t *)0x5000470C) = 0x7Eul;
	*((volatile uint32_t *)0x5000493C) = 0x7Eul;
	*((volatile uint32_t *)0x50002118) = 0x7Ful;
	*((volatile uint32_t *)0x50039E04) = 0x0ul;
	*((volatile uint32_t *)0x50039E08) = 0x0ul;
	*((volatile uint32_t *)0x50101110) = 0x0ul;
	*((volatile uint32_t *)0x50002124) = 0x0ul;
	*((volatile uint32_t *)0x5000212C) = 0x0ul;
	*((volatile uint32_t *)0x502012A0) = 0x0ul;
#elif defined(CONFIG_SOC_NRF5340_CPUNET)
	*((volatile uint32_t *)0x41002118) = 0x7Ful;
	*((volatile uint32_t *)0x41080E04) = 0x0ul;
	*((volatile uint32_t *)0x41080E08) = 0x0ul;
	*((volatile uint32_t *)0x41002124) = 0x0ul;
	*((volatile uint32_t *)0x4100212C) = 0x0ul;
	*((volatile uint32_t *)0x41101110) = 0x0ul;
#endif
}

/* This code prevents the CPU from entering sleep again if it already
 * entered sleep 5 times within last 200 us.
 */
static bool nrf53_anomaly_160_check(void)
{
	/* System clock cycles needed to cover 200 us window. */
	const uint32_t window_cycles =
		ceiling_fraction(200 * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
				 1000000);
	static uint32_t timestamps[5];
	static bool timestamps_filled;
	static uint8_t current;
	uint8_t oldest = (current + 1) % ARRAY_SIZE(timestamps);
	uint32_t now = k_cycle_get_32();

	if (timestamps_filled &&
	    /* + 1 because only fully elapsed cycles need to be counted. */
	    (now - timestamps[oldest]) < (window_cycles + 1)) {
		return false;
	}

	/* Check if the CPU actually entered sleep since the last visit here
	 * (WFE/WFI could return immediately if the wake-up event was already
	 * registered).
	 */
	if (nrf_power_event_check(NRF_POWER, NRF_POWER_EVENT_SLEEPENTER)) {
		nrf_power_event_clear(NRF_POWER, NRF_POWER_EVENT_SLEEPENTER);
		/* If so, update the index at which the current timestamp is
		 * to be stored so that it replaces the oldest one, otherwise
		 * (when the CPU did not sleep), the recently stored timestamp
		 * is updated.
		 */
		current = oldest;
		if (current == 0) {
			timestamps_filled = true;
		}
	}

	timestamps[current] = k_cycle_get_32();

	return true;
}

bool z_arm_on_enter_cpu_idle(void)
{
	bool ok_to_sleep = nrf53_anomaly_160_check();

#if (LOG_LEVEL >= LOG_LEVEL_DBG)
	static bool suppress_message;

	if (ok_to_sleep) {
		suppress_message = false;
	} else if (!suppress_message) {
		LOG_DBG("Anomaly 160 trigger conditions detected.");
		suppress_message = true;
	}
#endif
	if (ok_to_sleep) {
		if (IS_ENABLED(CONFIG_SOC_NRF53_ANOMALY_165)) {
			NRF_WDT->TASKS_STOP = 1;
		}
	}

	return ok_to_sleep;
}
#endif /* CONFIG_SOC_NRF53_ANOMALY_160_WORKAROUND */

static int nordicsemi_nrf53_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

#if defined(CONFIG_SOC_NRF5340_CPUAPP) && defined(CONFIG_NRF_ENABLE_CACHE)
#if !defined(CONFIG_BUILD_WITH_TFM)
	/* Enable the instruction & data cache.
	 * This can only be done from secure code.
	 * This is handled by the TF-M platform so we skip it when TF-M is
	 * enabled.
	 */
	nrf_cache_enable(NRF_CACHE);
#endif
#elif defined(CONFIG_SOC_NRF5340_CPUNET) && defined(CONFIG_NRF_ENABLE_CACHE)
	nrf_nvmc_icache_config_set(NRF_NVMC, NRF_NVMC_ICACHE_ENABLE);
#endif

#if defined(CONFIG_SOC_ENABLE_LFXO)
	nrf_oscillators_lfxo_cap_set(NRF_OSCILLATORS,
		IS_ENABLED(CONFIG_SOC_LFXO_CAP_INT_6PF) ?
			NRF_OSCILLATORS_LFXO_CAP_6PF :
		IS_ENABLED(CONFIG_SOC_LFXO_CAP_INT_7PF) ?
			NRF_OSCILLATORS_LFXO_CAP_7PF :
		IS_ENABLED(CONFIG_SOC_LFXO_CAP_INT_9PF) ?
			NRF_OSCILLATORS_LFXO_CAP_9PF :
			NRF_OSCILLATORS_LFXO_CAP_EXTERNAL);
#if !defined(CONFIG_BUILD_WITH_TFM)
	/* This can only be done from secure code.
	 * This is handled by the TF-M platform so we skip it when TF-M is
	 * enabled.
	 */
	nrf_gpio_pin_mcu_select(PIN_XL1, NRF_GPIO_PIN_MCUSEL_PERIPHERAL);
	nrf_gpio_pin_mcu_select(PIN_XL2, NRF_GPIO_PIN_MCUSEL_PERIPHERAL);
#endif /* !defined(CONFIG_BUILD_WITH_TFM) */
#endif /* defined(CONFIG_SOC_ENABLE_LFXO) */
#if defined(CONFIG_SOC_HFXO_CAP_INTERNAL)
	/* This register is only accessible from secure code. */
	uint32_t xosc32mtrim = soc_secure_read_xosc32mtrim();
	/* As specified in the nRF5340 PS:
	 * CAPVALUE = (((FICR->XOSC32MTRIM.SLOPE+56)*(CAPACITANCE*2-14))
	 *            +((FICR->XOSC32MTRIM.OFFSET-8)<<4)+32)>>6;
	 * where CAPACITANCE is the desired capacitor value in pF, holding any
	 * value between 7.0 pF and 20.0 pF in 0.5 pF steps.
	 */
	uint32_t slope = (xosc32mtrim & FICR_XOSC32MTRIM_SLOPE_Msk)
			 >> FICR_XOSC32MTRIM_SLOPE_Pos;
	uint32_t offset = (xosc32mtrim & FICR_XOSC32MTRIM_OFFSET_Msk)
			  >> FICR_XOSC32MTRIM_OFFSET_Pos;
	uint32_t capvalue =
		((slope + 56) * (CONFIG_SOC_HFXO_CAP_INT_VALUE_X2 - 14)
		 + ((offset - 8) << 4) + 32) >> 6;

	nrf_oscillators_hfxo_cap_set(NRF_OSCILLATORS, true, capvalue);
#elif defined(CONFIG_SOC_HFXO_CAP_EXTERNAL)
	nrf_oscillators_hfxo_cap_set(NRF_OSCILLATORS, false, 0);
#endif

#if defined(CONFIG_SOC_NRF53_ANOMALY_160_WORKAROUND)
	/* This needs to be done before DC/DC operation is enabled. */
	nrf53_anomaly_160_workaround();
#endif

#if defined(CONFIG_SOC_DCDC_NRF53X_APP)
	nrf_regulators_dcdcen_set(NRF_REGULATORS, true);
#endif
#if defined(CONFIG_SOC_DCDC_NRF53X_NET)
	nrf_regulators_dcdcen_radio_set(NRF_REGULATORS, true);
#endif
#if defined(CONFIG_SOC_DCDC_NRF53X_HV)
	nrf_regulators_dcdcen_vddh_set(NRF_REGULATORS, true);
#endif

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

void arch_busy_wait(uint32_t time_us)
{
	nrfx_coredep_delay_us(time_us);
}

SYS_INIT(nordicsemi_nrf53_init, PRE_KERNEL_1, 0);
