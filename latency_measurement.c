
/******************************************************************************
 * latency_measurement.c
 *
 * MAX32672 TMR3-based latency measurement.
 *
 * Measures:
 *     TMR1B ISR entry -> PWM compare-register update
 *
 * Timer allocation:
 *     TMR1B = 8 kHz acquisition trigger
 *     TMR2  = vibration PWM
 *     TMR3  = free-running latency timer
 ******************************************************************************/
#include "latency_measurement.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mxc_device.h"
#include "mxc_errors.h"
#include "tmr.h"

#include "IIM42352.h"
/* --------------------------------------------------------------------------
 * TMR3 configuration
 * -------------------------------------------------------------------------- */

#define LATENCY_TIMER                 MXC_TMR3

#define LATENCY_TIMER_CLOCK           MXC_TMR_IBRO_CLK

#define LATENCY_TIMER_PRESCALER       TMR_PRES_1

#define LATENCY_TIMER_CLOCK_DIVISOR   UINT32_C(1)

#define LATENCY_TIMER_COMPARE         UINT32_MAX

/*
 * MAX32672 IBRO nominal frequency.
 */
#define LATENCY_IBRO_FREQUENCY_HZ     UINT32_C(7372800)

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */

static volatile uint32_t g_start_ticks;

static volatile bool g_measurement_pending;

static volatile latency_statistics_t g_statistics;

static uint32_t g_timer_frequency_hz;

static bool g_timer_initialized;

static bool g_timer_counts_up;

/* --------------------------------------------------------------------------
 * Critical-section helpers
 * -------------------------------------------------------------------------- */

static uint32_t enter_critical_section(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();
    __DMB();

    return primask;
}

static void exit_critical_section(uint32_t primask)
{
   __DMB();

    if (primask == 0u) {
        __enable_irq();
    }
}

/* --------------------------------------------------------------------------
 * Timestamp access
 * -------------------------------------------------------------------------- */

static inline uint32_t read_timestamp(void)
{
    return MXC_TMR_GetCount(LATENCY_TIMER);
}

/* --------------------------------------------------------------------------
 * Elapsed-tick calculation
 * -------------------------------------------------------------------------- */

static uint32_t calculate_elapsed_ticks(
    uint32_t start_ticks,
    uint32_t end_ticks)
{
    if (g_timer_counts_up) {
        /*
         * Unsigned subtraction handles one upward 32-bit wrap.
         */
        return end_ticks - start_ticks;
    }

    /*
     * Unsigned subtraction handles one downward 32-bit wrap.
     */
    return start_ticks - end_ticks;
}

/* --------------------------------------------------------------------------
 * TMR3 initialization
 * -------------------------------------------------------------------------- */

static int initialize_latency_timer(void)
{
    int status;
    mxc_tmr_cfg_t timer_config;

    uint32_t count_before;
    uint32_t count_after;

    volatile uint32_t delay;

    /*
     * Reset TMR3 before changing its configuration.
     */
    MXC_TMR_Shutdown(LATENCY_TIMER);

    memset(&timer_config,
           0,
           sizeof(timer_config));

    /*
     * These are the mxc_tmr_cfg_t fields used by the MAX32672 MSDK.
     */
    timer_config.pres =
        LATENCY_TIMER_PRESCALER;

    timer_config.mode =
        TMR_MODE_CONTINUOUS;

    timer_config.cmp_cnt =
        LATENCY_TIMER_COMPARE;

    /*
     * No timer output pin is used.
     */
    timer_config.pol = 0u;

    /*
     * The third argument selects full-width timer operation in the
     * MAX32672 MSDK timer API.
     */
#if 0
    status =
        MXC_TMR_Init(
            LATENCY_TIMER,
            &timer_config,
            true);
#else
    status =
           MXC_TMR_Init(
               LATENCY_TIMER,
               &timer_config,
               false);
#endif
    if (status != E_NO_ERROR) {
        return status;
    }

    /*
     * TMR3 is only a timestamp source. Do not enable its interrupt.
     */
    MXC_TMR_DisableInt(LATENCY_TIMER);

    MXC_TMR_ClearFlags(LATENCY_TIMER);

    MXC_TMR_SetCount(
        LATENCY_TIMER,
        0u);

    /*
     * In the MAX32672 MSDK example, MXC_TMR_Start() is used without
     * checking a return value.
     */
    MXC_TMR_Start(LATENCY_TIMER);

    /*
     * IBRO divided by one.
     */
    g_timer_frequency_hz =
        LATENCY_IBRO_FREQUENCY_HZ /
        LATENCY_TIMER_CLOCK_DIVISOR;

    if (g_timer_frequency_hz == 0u) {
        MXC_TMR_Shutdown(LATENCY_TIMER);
        return E_BAD_STATE;
    }

    /*
     * Verify that TMR3 is counting and determine its direction.
     */
    count_before = read_timestamp();

    for (delay = 0u; delay < 256u; delay++) {
        __NOP();
    }

    __DSB();
    __ISB();

    count_after = read_timestamp();

    if (count_after == count_before) {
        MXC_TMR_Shutdown(LATENCY_TIMER);
        return E_BAD_STATE;
    }

    g_timer_counts_up =
        (count_after > count_before);

    g_timer_initialized = true;

    return E_NO_ERROR;
}

/* --------------------------------------------------------------------------
 * Public initialization
 * -------------------------------------------------------------------------- */

int latency_measurement_init(void)
{
    int status;

    g_timer_initialized = false;
    g_timer_frequency_hz = 0u;
    g_measurement_pending = false;

    status = initialize_latency_timer();

    if (status != E_NO_ERROR) {
        return status;
    }

    latency_measurement_reset_statistics();

    return E_NO_ERROR;
}

void latency_measurement_shutdown(void)
{
    uint32_t primask;

    primask = enter_critical_section();

    g_measurement_pending = false;
    g_timer_initialized = false;
    g_timer_frequency_hz = 0u;

    MXC_TMR_Stop(LATENCY_TIMER);
    MXC_TMR_Shutdown(LATENCY_TIMER);

    exit_critical_section(primask);
}

/* --------------------------------------------------------------------------
 * Statistics reset
 * -------------------------------------------------------------------------- */

void latency_measurement_reset_statistics(void)
{
    uint32_t primask;

    primask = enter_critical_section();

    g_start_ticks = 0u;
    g_measurement_pending = false;

    g_statistics.last_ticks = 0u;
    g_statistics.min_ticks = UINT32_MAX;
    g_statistics.max_ticks = 0u;

    g_statistics.total_ticks = 0u;
    g_statistics.sample_count = 0u;

    g_statistics.overwritten_start_count = 0u;
    g_statistics.update_without_start_count = 0u;

    exit_critical_section(primask);
}

/* --------------------------------------------------------------------------
 * Measurement start
 * -------------------------------------------------------------------------- */

void latency_measurement_start(void)
{
    uint32_t timestamp;
    /*
     * Read the timer before performing bookkeeping so the timestamp is as
     * close as possible to TMR1B ISR entry.
     */
    timestamp = read_timestamp();

    /*
     * Another TMR1B event occurred before the preceding acquisition path
     * reached its PWM update.
     */
    if (g_measurement_pending) {
        g_statistics.overwritten_start_count++;
    }

    g_start_ticks = timestamp;

    __DMB();

    g_measurement_pending = true;
}

void latency_measurement_get_statistics(
    latency_statistics_t *statistics)
{
    uint32_t primask;

    if (statistics == NULL) {
        return;
    }

    /*
     * The timing statistics are updated from interrupt context.
     * Protect the copy so the caller receives a consistent snapshot.
     */
    primask = enter_critical_section();

    statistics->last_ticks =
        g_statistics.last_ticks;

    statistics->min_ticks =
        g_statistics.min_ticks;

    statistics->max_ticks =
        g_statistics.max_ticks;

    statistics->total_ticks =
        g_statistics.total_ticks;

    statistics->sample_count =
        g_statistics.sample_count;

    statistics->overwritten_start_count =
        g_statistics.overwritten_start_count;

    statistics->update_without_start_count =
        g_statistics.update_without_start_count;

    exit_critical_section(primask);

    /*
     * Internally, UINT32_MAX indicates that no minimum has been recorded.
     * Return zero to the application when no completed samples exist.
     */
    if (statistics->sample_count == 0u) {
        statistics->min_ticks = 0u;
    }
}

void latency_measurement_pwm_updated(void)
{
    uint32_t end_ticks;
    uint32_t elapsed_ticks;

    /*
     * Capture the timestamp as close as possible to the PWM compare-register
     * update. Call this function immediately after the register write.
     */
    end_ticks = read_timestamp();

    __DMB();

    /*
     * PWM initialization or a console command can update PWM without a
     * preceding TMR1B measurement start. Do not include those updates in
     * the latency statistics.
     */
    if (!g_measurement_pending) {
        g_statistics.update_without_start_count++;
        return;
    }

    /*
     * Account for timer direction and one possible 32-bit timer wrap.
     */
    elapsed_ticks =
        calculate_elapsed_ticks(
            g_start_ticks,
            end_ticks);

    /*
     * Consume the pending measurement before updating statistics.
     */
    g_measurement_pending = false;

    g_statistics.last_ticks =
        elapsed_ticks;

    if (elapsed_ticks < g_statistics.min_ticks) {
        g_statistics.min_ticks =
            elapsed_ticks;
    }

    if (elapsed_ticks > g_statistics.max_ticks) {
        g_statistics.max_ticks =
            elapsed_ticks;
    }

    g_statistics.total_ticks +=
        (uint64_t)elapsed_ticks;

    g_statistics.sample_count++;
}

uint32_t latency_measurement_get_timer_frequency_hz(void)
{
    return g_timer_frequency_hz;
}
uint32_t latency_measurement_ticks_to_ns(
    uint32_t timer_ticks)
{
    uint64_t nanoseconds;

    if (g_timer_frequency_hz == 0u) {
        return 0u;
    }

    /*
     * Use 64-bit arithmetic to avoid overflow during multiplication.
     */
    nanoseconds =
        ((uint64_t)timer_ticks *
         UINT64_C(1000000000)) /
        (uint64_t)g_timer_frequency_hz;

    /*
     * Saturate because the public return type is uint32_t.
     */
    if (nanoseconds >
        (uint64_t)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)nanoseconds;
}

float latency_measurement_ticks_to_us(
    uint32_t timer_ticks)
{
    if (g_timer_frequency_hz == 0u) {
        return 0.0f;
    }

    return
        ((float)timer_ticks *
         1000000.0f) /
        (float)g_timer_frequency_hz;
}


