#ifndef LATENCY_MEASUREMENT_H
#define LATENCY_MEASUREMENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Latency statistics.
 *
 * All timing fields are TMR3 ticks, not CPU cycles.
 */
typedef struct
{
    uint32_t last_ticks;
    uint32_t min_ticks;
    uint32_t max_ticks;

    uint64_t total_ticks;
    uint32_t sample_count;

    /*
     * Number of TMR1B starts that occurred before the preceding
     * measurement reached the PWM update.
     */
    uint32_t overwritten_start_count;

    /*
     * Number of PWM updates that occurred without an active TMR1B
     * measurement. Initialization and console commands can cause these.
     */
    uint32_t update_without_start_count;
} latency_statistics_t;

/*
 * Initialize TMR3 as a free-running timestamp timer.
 *
 * Returns an MSDK error code:
 *
 *     E_NO_ERROR on success
 *     other MSDK error code on failure
 */
int latency_measurement_init(void);

/*
 * Stop and release the TMR3 timestamp timer.
 */
void latency_measurement_shutdown(void);

/*
 * Clear all timing statistics.
 *
 * TMR3 continues running.
 */
void latency_measurement_reset_statistics(void);

/*
 * Start one measurement.
 *
 * Call this as the first operation in the TMR1B interrupt handler.
 */
void latency_measurement_start(void);

/*
 * Complete one measurement.
 *
 * Call this immediately after the actual PWM compare-register update.
 */
void latency_measurement_pwm_updated(void);

/*
 * Copy an atomic snapshot of the latency statistics.
 */
void latency_measurement_get_statistics(
    latency_statistics_t *statistics);

/*
 * Returns true while a TMR1B-to-PWM measurement is active.
 */
bool latency_measurement_is_pending(void);

/*
 * Return the TMR3 timestamp frequency in Hz.
 */
uint32_t latency_measurement_get_timer_frequency_hz(void);

/*
 * Convert TMR3 timestamp ticks into nanoseconds or microseconds.
 */
uint32_t latency_measurement_ticks_to_ns(
    uint32_t timer_ticks);

float latency_measurement_ticks_to_us(
    uint32_t timer_ticks);

#ifdef __cplusplus
}
#endif

#endif /* LATENCY_MEASUREMENT_H */

