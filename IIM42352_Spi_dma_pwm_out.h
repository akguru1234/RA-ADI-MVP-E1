/*
 * IIM42352_Spi_dma_pwm_out.h
 *
 *  Created on: Sep 25, 2026
 *      Author: akguru
 */

#ifndef IIM42352_SPI_DMA_PWM_OUT_H_
#define IIM42352_SPI_DMA_PWM_OUT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DRDY_INTERRUPT_WIRED 0	//not used on the E0 board

extern volatile uint32_t g_pwm_period_ticks;
extern volatile bool g_spi_dma_busy;
extern volatile int g_spi_dma_status;
extern volatile uint32_t g_dma_error_count;
extern volatile uint32_t g_dma_overrun_count;
extern volatile uint32_t g_PWM_update_error;

extern int iim_spi_dma_init(void);
extern int iim_start_accel_dma(void);
extern int iim_spi_transaction(const uint8_t *tx,
                               uint8_t *rx,
                               uint32_t tx_len, // @suppress("Type cannot be resolved")
                               uint32_t rx_len); // @suppress("Type cannot be resolved")



extern int vibration_pwm_set_percent(float duty_percent);
#ifdef __cplusplus
}
#endif

#endif /* IIM42352_SPI_DMA_PWM_OUT_H_ */
