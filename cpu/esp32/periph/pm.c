/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_esp32
 * @ingroup     drivers_periph_pm
 * @{
 *
 * @file
 * @brief       Implementation of power management functions
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @}
 */

/* RIOT headers have to be included before ESP-IDF headers! */
#include "esp_attr.h"
#include "gpio_arch.h"
#include "periph/rtc.h"
#include "rtt_arch.h"
#include "syscalls.h"

/* ESP-IDF headers */
#include "esp_sleep.h"
#include "rom/rtc.h"
#include "rom/uart.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"

#define ENABLE_DEBUG 0
#include "debug.h"

extern void rtt_save_counter(void);
extern void rtt_restore_counter(void);

static inline esp_sleep_wakeup_cause_t pm_get_wakeup_cause(void)
{
    return esp_sleep_get_wakeup_cause();
}

/* function that is required by pm_set if esp_now and esp_wifi are not used */
esp_err_t __attribute__((weak)) esp_wifi_start(void)
{
    return ESP_OK;
}

/* function that is required by pm_set if esp_now and esp_wifi are not used */
esp_err_t __attribute__((weak)) esp_wifi_stop(void)
{
    return ESP_OK;
}

static inline void pm_set_lowest_normal(void)
{
    /* reset system watchdog timer */
    system_wdt_feed();

#ifndef MODULE_ESP_QEMU
    /* passive wait for interrupt to leave lowest power mode */
    __asm__ volatile ("waiti 0");

    /* reset system watchdog timer */
    system_wdt_feed();
#endif
}

void IRAM_ATTR pm_off(void)
{
    /* disable remaining power domains */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);

    /* enter hibernate mode without any enabled wake-up sources */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_deep_sleep_start();
}

extern void esp_restart_noos(void) __attribute__ ((noreturn));

void pm_reboot(void)
{
    DEBUG ("%s\n", __func__);

    if (IS_USED(MODULE_ESP_WIFI_ANY)) {
        /* stop WiFi if necessary */
        esp_wifi_stop();
    }

    /* flush all UARTs of the SoC, there is no problem to do it for uninitialized UARTs */
    for (int i = 0; i < SOC_UART_NUM; ++i) {
        uart_tx_wait_idle(i);
    }

    /* save RTT counters */
    rtt_save_counter();

    software_reset();
}

#if !IS_USED(MODULE_PM_LAYERED)

void pm_set_lowest(void)
{
    pm_set_lowest_normal();
}

#else /* MODULE_PM_LAYERED */

void pm_set(unsigned mode)
{
    if (mode == ESP_PM_MODEM_SLEEP) {
        pm_set_lowest_normal();
        return;
    }

    DEBUG ("%s enter to power mode %d @%u\n", __func__, mode, system_get_time());

    /* flush stdout */
    fflush(stdout);

    /* Labels for RTC slow memory that are defined in the linker script */
    extern int _rtc_bss_rtc_start;
    extern int _rtc_bss_rtc_end;

    /*
     * Activate the Power Domain for slow RTC memory when the .rtc.bss
     * section is used to retain uninitialized data. The Power Domain for
     * slow RTC memory is automatically activated when the .rtc.data section
     * is used to retain initialized data.
     */
    /* cppcheck-suppress comparePointers */
    if (&_rtc_bss_rtc_end > &_rtc_bss_rtc_start) {
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    }

    /* first disable all wake-up sources */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    /* Prepare the RTC timer if an RTC alarm is set to wake up. */
    rtt_pm_sleep_enter(mode);

    /* Prepare GPIOs as wakeup source */
    gpio_pm_sleep_enter(mode);

    if (mode == ESP_PM_DEEP_SLEEP) {
        esp_deep_sleep_start();
        /* waking up from deep-sleep leads to a DEEPSLEEP_RESET */
        UNREACHABLE();
    }
    else if (mode == ESP_PM_LIGHT_SLEEP) {
        if (IS_USED(MODULE_ESP_WIFI_ANY)) {
            /* stop WiFi if necessary */
            esp_wifi_stop();
        }

        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t wakeup_reason = pm_get_wakeup_cause();
        gpio_pm_sleep_exit(wakeup_reason);
        rtt_pm_sleep_exit(wakeup_reason);

        DEBUG ("%s exit from power mode %d @%u with reason %d\n", __func__,
               mode, system_get_time(), wakeup_reason);

        /* restart WiFi if necessary */
        if (IS_USED(MODULE_ESP_WIFI_ANY) && (esp_wifi_start() != ESP_OK)) {
            LOG_ERROR("esp_wifi_start failed\n");
        }
    }
}

#endif /* MODULE_PM_LAYERED */
