/******************************************************************************
 * main.c
 *
 *  Created on: Sep 25, 2026
 *      Author: akguru
 *
 ******************************************************************************
 * MAX32672 + IIM-42352
 * SPI0 DMA acceleration acquisition samples streamed over TMR2 PWM output
 *
 * Acquisition:
 *   IIM-42352 INT1 -> MAX32672 GPIO ISR
 *                  -> MXC_SPI_MasterTransactionDMA()
 *                  -> SPI DMA callback
 *                  -> ping-pong acceleration buffers
 *
 * Sensor transfer:
 *   TX: 0x9F                    ACCEL_DATA_X1 | READ
 *   RX: X_MSB X_LSB Y_MSB Y_LSB Z_MSB Z_LSB
 * Interrupt Sequence
 * IIM-42352 updates acceleration registers
 *
 * IIM-42352 asserts INT1
 *
 * MAX32672 enters GPIO0_IRQHandler()
 *
 * MXC_GPIO_Handler() calls iim_drdy_gpio_callback()
 *
 * Callback calls MXC_SPI_MasterTransactionDMA()
 *
 * DMA moves command and acceleration data
 *
 * DMA0_IRQHandler() or DMA1_IRQHandler() runs
 *
 * MXC_DMA_Handler() advances the SPI DMA state
 *
 * MSDK calls iim_spi_dma_callback()
 *
 * Callback publishes the completed ping-pong buffer
 *
 * Main loop parses X, Y, and Z
 *
 *		 IIM-42352 at 8 kHz
 *		       |
 *		       v
 *		SPI0 DMA burst read
 *		       |
 *		       v
 *		Decode X/Y/Z sample
 *		       |
 *		       v
 *		Select axis and remove offset
 *		       |
 *		       v
 *		Map signed acceleration to duty cycle
 *		       |
 *		       v
 *		Update TMR2B PWM compare
 *		       |
 *		       v
 *		P0.13 PWM output
 *		       |
 *		       v
 *		External RC low-pass filter
 *		       |
 *		       v
 *		Reconstructed vibration waveform
  ******************************************************************************/
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
#include "tmr.h"
#include "arm_math.h"
#include "uart.h"
#include "IIM42352.h"
#include "vibration_console.h"
#include "IIM42352_Spi_dma_pwm_out.h"
#include "IIM42352-Vibration-FFT.h"
#include "Timer1_Task.h"
#include "latency_measurement.h"

//select gpio for direct on/off control using LED_ON and LED_OFF macros
const mxc_gpio_cfg_t led_pin[] = {
   // { MXC_GPIO0, MXC_GPIO_PIN_28, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_13, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 }
};


const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

/*
* MAX32672 UART1A target-board pins:
*
* P0.28 = UART1A_RX
* P0.29 = UART1A_TX
*/
#define DEBUG_UART MXC_UART1
#define DEBUG_UART_BAUD 115200u

static const mxc_gpio_cfg_t g_uart1a_pins =
{
    MXC_GPIO0,
    MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29,
    MXC_GPIO_FUNC_ALT1,
    MXC_GPIO_PAD_NONE,
    MXC_GPIO_VSSEL_VDDIO
};


static int uart1_console_init(void);

/***** Functions *****/
int uart1_console_init(void)
{
    int status;

    /*
     * P0.29 must not already be configured as the board status LED.
     */
    status = MXC_GPIO_Config(&g_uart1a_pins);

    if (status != E_NO_ERROR) {
        return status;
    }

    /*
     * Initialize UART1 using the internal baud-rate oscillator.
     */
    status = MXC_UART_Init(DEBUG_UART,
                           DEBUG_UART_BAUD,
                           MXC_UART_IBRO_CLK);

    if (status != E_NO_ERROR) {
        return status;
    }

    return E_NO_ERROR;
}

int main(void)
{
    int status;
    /*
     * Initialize SPI0 and DMA.
     */

    /*
     * Give the board power rails, 25 MHz clock source, accelerometer,
     * and encoder interface time to settle.
     */
    MXC_Delay(MXC_DELAY_MSEC(20));
    /*
     * UART1A:
     *     RX = P0.28
     *     TX = P0.29
     *
     * This must execute before the first printf().
     */
    status = uart1_console_init();

    printf("Hello from Rockwell!\n");

    status = iim_spi_dma_init();
    if (status != E_NO_ERROR) {
    	printf("SPI DMA initialization failed: %d\r\n",
    			status);
    	while (1) {
    	}
    }
    /*
     * Configure IIM-42352:
     *
     *   ACCEL_ODR = 8 kHz
     *   low-noise mode
     *   appropriate full scale
     *   UI data-ready routed to INT1
     */
    status = iim42352_init();
    if (status != E_NO_ERROR) {
    	printf("IIM-42352 initialization failed: %d\r\n",
    			status);
		while (1) {
		}
    }
    /*
     * Configure TMR2B PWM output at 50% initial duty.
     */
    status = vibration_pwm_init();
    if (status != E_NO_ERROR) {
    	printf("PWM initialization failed: %d\r\n",
    			status);
		while (1) {
		}
    }

    /*
    * SystemCoreClock must reflect the current processor-clock frequency.
    */
    SystemCoreClockUpdate();
    status = latency_measurement_init();
    if (status != E_NO_ERROR) {
    	printf("Latency measurement initialization failed: %d\r\n",
    			status);
    	while (1) {
    	}
    }
    /*
    * The command parser uses UART1 already initialized above.
    */
    vibration_console_init();
    /*
     * Configure the IIM-42352 INT1 input and callback last.
     * This avoids receiving sensor interrupts before SPI, DMA, and PWM
     * objects are ready.
     */
    status = vibration_processing_init(); //initializes FFT power spectrum
    if (status != E_NO_ERROR) {
    	printf("FFT initialization failed: %d\r\n",
    			status);
    	while (1) {
    	}
    }
#if DRDY_INTERRUPT_WIRED
    status = iim_drdy_interrupt_init();
    if (status != E_NO_ERROR) {
        while (1) {
        }
    }
#endif
    /*
     * Configure Timer 1 for continuous run. Timer 1 interrupt will be used
     * to poll iim-42352 interrupt status and read from it using non-blocking spi DMA
     */
    MXC_NVIC_SetVector(TMR1_IRQn, ContinuousTimerHandler);
    NVIC_EnableIRQ(TMR1_IRQn);
    ContinuousTimerInit();

    while (1) {
        /*
         * PWM output and SPI acquisition are interrupt driven.
         *
         * Main context can perform:
         *   - diagnostics
         *   - FFT processing
         *   - UART logging
         *   - encoder communication
         */

    	/*
    	 * Nonblocking console command processing.
    	 */
    	vibration_service(); //read and fill ring buffers for FFT
    	vibration_console_service();//process console serial comms

    }
}



