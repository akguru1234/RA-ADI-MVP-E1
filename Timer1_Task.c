/* --------------------------------------------------------------------------
 * Timer1_Task.c
 *
 *  Created on: Sep 29, 2026
 *  Author: akguru
 *--------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "spi.h"
#include "dma.h"
#include "gpio.h"
#include "led.h"
#include "tmr.h"
#include "arm_math.h"
#include "uart.h"
#include "IIM42352.h"
#include "vibration_console.h"
#include "IIM42352_Spi_dma_pwm_out.h"
#include "IIM42352-Vibration-FFT.h"
#include "latency_measurement.h"


/* --------------------------------------------------------------------------
 * Timer 1 interrupt handlers
 * -------------------------------------------------------------------------- */
// Parameters for Continuous timer interrupt task

#define CLOCK_SOURCE 	MXC_TMR_IBRO_CLK // must be mxc_tmr_clock_t
#define CONT_FREQ 		8000 // 125uS
#define CONT_TIMER 		MXC_TMR1 // Can be MXC_TMR0 through MXC_TMR5


/* --------------------------------------------------------------------------
 * TMR1 isr handler
 * TMR1 is set to raise an interrupt every 125us
 * -------------------------------------------------------------------------- */
void ContinuousTimerHandler(void)
{
	LED_On(0);
	int error;
	bool int_status_val =1;
	/*
	* First operation after normal Cortex-M exception entry.
	*
	* This measures ISR-body entry to PWM update. It does not include the
	* hardware exception-entry stacking cycles that occur before this line.
	*/
	latency_measurement_start();

	MXC_TMR_ClearFlags(CONT_TIMER);

	//check if new sample is ready for transfer from IIM 42352 - read data ready status
	error = iim42352_data_ready(&int_status_val);

	if (error != E_NO_ERROR) {
		return;
	}

	if (int_status_val) //if data is ready in the int status register then start DMA read
	{
		//sets up spi read of 6 bytes from iim accel registers using DMA
		//spi dma callback handles transfer of data from ping-pong buffers to ring buffer
		error = iim_start_accel_dma();
		if (error != E_NO_ERROR) {
			/*
			 * Record the error only. Do not print from interrupt context.
			 * SPI DMA callback copies accel results to ring buffer for FFT
			 * SPI DMA call back updates the PWM register for sending selected accel
			 * register to analog out on P0.13/TMR2B output pin
			 */
			g_spi_dma_status = error;
			return;
		}
	}
	//latency_measurement_pwm_updated();
	LED_Off(0);
return;
}
/* --------------------------------------------------------------------------
 * TMR initialization
 *
 *   Steps for configuring a timer for Continuous Mode:
 *   1. Disable the timer
 *   2. Set the prescale value
 *   3  Configure the timer for Continuous Mode
 *   4. Set polarity, timer parameters
 *   5. Enable Timer
 *
 * -------------------------------------------------------------------------- */
void ContinuousTimerInit(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks; // @suppress("Type cannot be resolved")

    MXC_TMR_Shutdown(CONT_TIMER); // @suppress("Type cannot be resolved")

    // Calculate number of ticks in continuous timer period
    periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, CLOCK_SOURCE, 128, CONT_FREQ); // @suppress("Type cannot be resolved")

    // Configure continuous timer
    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.clock = CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;
    //TMR1A_I and TMR1A_O are multiplexed with SPI0 MISO and MOSI so use TMR1B for 8kHz task
    tmr.bitMode= TMR_BIT_MODE_16B;
    MXC_TMR_Init(CONT_TIMER, &tmr, true);
    MXC_TMR_EnableInt(CONT_TIMER);

    // Start continuous timer
    MXC_TMR_Start(CONT_TIMER);

}

