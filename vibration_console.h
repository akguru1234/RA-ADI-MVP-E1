/*
 * vibration_console.h
 *
 *  Created on: Sep 25, 2026
 *      Author: akguru
 */

#ifndef VIBRATION_CONSOLE_H
#define VIBRATION_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    VIB_PWM_CHANNEL_X = 0,
	VIB_PWM_CHANNEL_Y,
	VIB_PWM_CHANNEL_Z,
    VIB_PWM_CHANNEL_MAGNITUDE,
    VIB_PWM_CHANNEL_OFF
} vibration_channel_t;

extern volatile vibration_channel_t g_selected_channel;
extern volatile float g_pwm_input_range_g;

/*
 * Initializes parser state.
 * Call after uart1_console_init() and vibration_pwm_init().
 */
extern void vibration_console_init(void);

/*
* Call repeatedly from the main loop.
*
* This function never waits for a complete command. It reads only characters
* already present in the UART RX FIFO.
*/
extern void vibration_console_service(void);

/*
 * Selects and conditions one output value from a calibrated XYZ sample.
 * Intended to be called from the SPI DMA completion callback.
 */
extern float vibration_console_process_sample(float x_g,
                                       float y_g,
                                       float z_g);

/*
 * Converts the selected acceleration into PWM duty cycle and calls the
 * application's existing vibration_pwm_set_percent() API.
 */

extern int vibration_console_output_sample(float x_g,
                                    float y_g,
                                    float z_g);
/*
* Called by the vibration/PWM processing path.
*/
extern vibration_channel_t vibration_console_get_channel(void);

float vibration_console_get_range_g(void);
float vibration_console_get_input_range_g(void);
bool vibration_console_dc_removal_enabled(void);

/*
* Processes one calibrated X/Y/Z sample and returns the signal selected
* for PWM output.
*/
float vibration_console_select_signal(float x_g,float y_g,float z_g);

/*
 * Diagnostic counters supplied by the acquisition module.
 */
 extern volatile uint32_t g_dma_complete_count;
 extern volatile uint32_t g_dma_error_count;
 //extern volatile uint32_t g_dma_overrun_count;
 extern volatile float g_pwm_duty_percent;

#ifdef __cplusplus
}
#endif
#endif /* VIBRATION_CONSOLE_H */
