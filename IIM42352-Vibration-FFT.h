/*
 * IIM42352-Vibration-FFT.h
 *
 *  Created on: Sep 25, 2026
 *      Author: akguru
 */

#ifndef IIM42352_VIBRATION_FFT_H_
#define IIM42352_VIBRATION_FFT_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * TMR2B PWM output
 * -------------------------------------------------------------------------- */
#define VIB_PWM_TIMER                   MXC_TMR2
/*
 * P0.13 is TMR2B_O on the referenced MAX32672 board schematic.
 *
 * Use a carrier well above the 0 to 4 kHz vibration band.
 *
 * At 100 kHz:
 *     100000 / 8000 = 12.5 PWM periods per sensor sample
 */

#define VIB_PWM_PORT                    MXC_GPIO0
#define VIB_PWM_PIN                     MXC_GPIO_PIN_13
#define VIB_PWM_CARRIER_HZ              100000u
#define VIB_PWM_CLOCK                   MXC_TMR_IBRO_CLK
#define VIB_PWM_PRESCALER               1u
#define VIB_PWM_PRESCALER_ENUM          TMR_PRES_1

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} accel_raw_sample_t;
/*
 * Per-axis calibration:
 *
 * corrected_g = raw_g * scale - bias_g
 *
 * Populate these values from static calibration, self-test, or a controlled
 * six-position calibration procedure.
 */
typedef struct
{
    float bias_x_g;
    float bias_y_g;
    float bias_z_g;

    float scale_x;
    float scale_y;
    float scale_z;
} accel_calibration_t;

typedef struct
{
    uint32_t sequence; // @suppress("Type cannot be resolved")

    float rms_g;
    float peak_g;
    float peak_to_peak_g;
    float crest_factor;
    float dominant_frequency_hz;
    float dominant_amplitude_g;

    float band_10_100_hz_g2;
    float band_100_500_hz_g2;
    float band_500_1000_hz_g2;
    float band_1000_2000_hz_g2;
    float band_2000_4000_hz_g2;

    uint32_t acquisition_overruns; // @suppress("Type cannot be resolved")
} vibration_result_t;

extern volatile uint32_t g_ring_write;
extern volatile uint32_t g_ring_read;
extern volatile uint32_t g_ring_overruns;
extern volatile bool g_vibration_result_ready;
extern volatile vibration_result_t g_vibration_result;
extern volatile uint32_t g_frame_overrun;

extern void publish_vibration_result(const vibration_result_t *result);
extern void vibration_service(void);
extern bool sample_ring_push_from_isr(const accel_raw_sample_t *sample);
extern int vibration_processing_init(void);
extern int vibration_pwm_init(void);

#ifdef __cplusplus
}
#endif

#endif /* IIM42352_VIBRATION_FFT_H_ */
