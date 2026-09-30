/*
 * IIM42352-SPI-DMA-PWM_OUT.c
 *
 *  Created on: Sep 25, 2026
 *      Author: akguru
 */
/******************************************************************************
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
 * IIM-42352 asserts INT1 (not on E0 board, use polling via TMR1B task)
 *
 * MAX32672 enters GPIO0_IRQHandler() (not on E0 board, use polling via TMR1B task)
 *
 * MXC_GPIO_Handler() calls iim_drdy_gpio_callback() (not on E0 board, use polling via TMR1B task)
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
#include "led.h"
#include "tmr.h"
#include "arm_math.h"

#include "IIM42352.h"
#include "vibration_console.h"
#include "IIM42352_Spi_dma_pwm_out.h"
#include "IIM42352-Vibration-FFT.h"
#include "latency_measurement.h"

/* --------------------------------------------------------------------------
 * SPI0 configuration
 * -------------------------------------------------------------------------- */

#define IIM_SPI                         MXC_SPI0
#define IIM_SPI_SS_IDX                  0u
#define IIM_SPI_SPEED_HZ           		12000000u

/* --------------------------------------------------------------------------
 * IIM-42352 acquisition
 * -------------------------------------------------------------------------- */
#define IIM_REG_ACCEL_DATA_X1           0x1Fu
#define IIM_SPI_READ_BIT                0x80u
//#define IIM_SAMPLE_RATE_HZ              8000u
#define IIM_ACCEL_BYTES                 6u
#define IIM_DMA_BUFFER_COUNT       		2u
/*
 * ±16 g full scale:
 *
 * 32768 counts / 16 g = 2048 counts/g
 */
#define IIM_ACCEL_LSB_PER_G             1.0f/2048.0f

/*
 * Limit normal duty-cycle operation to 5% through 95%.
 * This preserves headroom and avoids pathological 0%/100% timer behavior.
 */
#define VIB_PWM_MIN_PERCENT             5.0f
#define VIB_PWM_MAX_PERCENT             95.0f
#define VIB_PWM_CENTER_PERCENT          50.0f

#define MINIMUM_RANGE_G                0.05f
#define MAXIMUM_RANGE_G                16.0f

/*
 * Acceleration magnitude mapped to full PWM span.
 *
 * Example:
 *     -4 g -> 5% duty
 *      0 g -> 50% duty
 *     +4 g -> 95% duty
 *
 * Lower this value for increased sensitivity.
 */
#define VIB_PWM_INPUT_RANGE_G           16.0f

/* Select which acceleration signal is placed on the PWM output. */
#define VIB_PWM_AXIS_X 	1
#define	VIB_PWM_AXIS_Y	2
#define	VIB_PWM_AXIS_Z	3
#define	VIB_PWM_VECTOR_MAGNITUDE	4

#define VIB_PWM_SELECTED_AXIS           VIB_PWM_AXIS_X

//static accel_raw_sample_t g_sample_int;

//static void update_vibration_output_to_pwm(void);

typedef struct
{
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
} accel_offset_t;

typedef struct
{
    float dc_estimate_g;
    float alpha;
} dc_blocker_t;

static accel_offset_t g_accel_offset =
{
    .offset_x = 0,
    .offset_y = 0,
    .offset_z = 0
};
/*
 * SPI DMA request and buffers must remain valid until callback completion.
 */
static mxc_spi_req_t g_spi_dma_req;
static void iim_spi_dma_callback(mxc_spi_req_t *req, int error);

static uint8_t g_spi_command =
    IIM_REG_ACCEL_DATA_X1 | IIM_SPI_READ_BIT;

static uint8_t g_rx_buffer[IIM_DMA_BUFFER_COUNT][IIM_ACCEL_BYTES+1];

static volatile uint8_t g_dma_write_index;
volatile bool g_spi_dma_busy;
volatile int g_spi_dma_status;

 volatile uint32_t g_dma_complete_count;
 volatile uint32_t g_dma_error_count;
 volatile uint32_t g_dma_overrun_count;
/*
 * PWM timer period in timer-clock ticks.
 */
volatile uint32_t g_pwm_period_ticks;
//static bool g_sample_ready;
volatile uint32_t g_PWM_update_error;

/*
 * Most recent output value, useful for diagnostics.
 */
//static volatile uint32_t g_pwm_duty_ticks;
 volatile float g_pwm_duty_percent;

/* --------------------------------------------------------------------------
 * Low-level SPI access
 * -------------------------------------------------------------------------- */
int iim_spi_transaction(const uint8_t *tx,
                               uint8_t *rx,
                               uint32_t tx_len, // @suppress("Type cannot be resolved")
                               uint32_t rx_len) // @suppress("Type cannot be resolved")
{
    mxc_spi_req_t req;

    req.spi        = IIM_SPI; // @suppress("Type cannot be resolved")
    req.txData     = (uint8_t *)tx;
    req.rxData     = rx;
    req.txLen      = tx_len;
    req.rxLen      = rx_len;
    req.ssIdx      = IIM_SPI_SS_IDX;
    req.ssDeassert = 1;
    req.completeCB = NULL;
    req.txCnt = 0;
    req.rxCnt = 0;
    return MXC_SPI_MasterTransaction(&req);
}

static inline float clamp_float(float value,
                                float minimum,
                                float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

int vibration_pwm_set_percent(float duty_percent)
{
	int status = E_NO_ERROR;
	uint32_t duty_ticks;

    duty_percent =
        clamp_float(duty_percent,
                    VIB_PWM_MIN_PERCENT,
                    VIB_PWM_MAX_PERCENT);

    duty_ticks =
        (uint32_t)((duty_percent *
                    (float)g_pwm_period_ticks) / 100.0f);

    if (duty_ticks == 0u) {
        duty_ticks = 1u;
    }

    if (duty_ticks >= g_pwm_period_ticks) {
        duty_ticks = g_pwm_period_ticks - 1u;
    }

    /*
     * Update the PWM compare value.
     */

    status = MXC_TMR_SetPWM(VIB_PWM_TIMER, duty_ticks );

     if (status != E_NO_ERROR) {
         MXC_TMR_Shutdown(VIB_PWM_TIMER);
         g_PWM_update_error++;
         return status;
     }
     g_pwm_duty_percent = duty_percent;
    return E_NO_ERROR;
}

static float vibration_accel_to_duty(float acceleration_g)
{
    float range_g;
    float normalized;
    float duty;
    float usable_half_span;

    range_g =
        vibration_console_get_input_range_g();

    if (range_g < MINIMUM_RANGE_G) {
        range_g = MINIMUM_RANGE_G;
    }

    usable_half_span =
        (VIB_PWM_MAX_PERCENT -
         VIB_PWM_MIN_PERCENT) * 0.5f;

    normalized =
        acceleration_g / range_g;

    normalized =
        clamp_float(normalized,
                    -1.0f,
                    1.0f);

    duty =
        VIB_PWM_CENTER_PERCENT +
        normalized * usable_half_span;

    return duty;
}

static inline accel_raw_sample_t decode_accel_sample(
     uint8_t data[IIM_ACCEL_BYTES])
{
    accel_raw_sample_t sample;

    sample.x =
        (int16_t)(((uint16_t)data[0] << 8) |
                   (uint16_t)data[1]);

    sample.y =
        (int16_t)(((uint16_t)data[2] << 8) |
                   (uint16_t)data[3]);

    sample.z =
        (int16_t)(((uint16_t)data[4] << 8) |
                   (uint16_t)data[5]);

    return sample;
}
/*-----------------------------------------------------------
 * SPI callback function is invoked after DMA transfer is
 * completed.
*------------------------------------------------------------*/

void  iim_spi_dma_callback (mxc_spi_req_t *request, int error)
{
	//LED_On(0);
	uint8_t completed_index;
    accel_raw_sample_t raw;

    float x_g;
    float y_g;
    float z_g;

    float selected_signal_g;
    float duty_percent = 50.0f;

    (void)request;

    completed_index = g_dma_write_index;
    g_spi_dma_status = error;
    g_spi_dma_busy = false;

    if (error != E_NO_ERROR) {
        g_dma_error_count++;
        return;
    }
    g_dma_write_index ^= 1u;

    /*
     * store raw samples in a buffer
     */

    raw = decode_accel_sample(g_rx_buffer[completed_index]+1);

    if (!sample_ring_push_from_isr(&raw)) {
    	/*
    	 * Ring-full count is recorded by sample_ring_push_from_isr().
    	 */
    }

    /*
     * Convert raw readings to calibrated acceleration.
     *
     * For ±16 g:
     *     sensitivity = 2048 LSB/g
     */
    x_g =
        ((float)raw.x -
         (float)g_accel_offset.offset_x) * (float) IIM_ACCEL_LSB_PER_G;

    y_g =
        ((float)raw.y -
         (float)g_accel_offset.offset_y) * (float)  IIM_ACCEL_LSB_PER_G;

    z_g =
        ((float)raw.z -
         (float)g_accel_offset.offset_z) * (float)  IIM_ACCEL_LSB_PER_G;

    /*
     * Select X, Y, Z, magnitude, or zero based on the console command.
     */
    selected_signal_g =
        vibration_console_select_signal(
            x_g,
            y_g,
            z_g);

    /*
     * Convert signed acceleration to PWM duty around 50%.
     */
    duty_percent =
        vibration_accel_to_duty(
            selected_signal_g);

    /*
     * On the current target, TMR2A PWM is routed to the TMR2B output pin
     * because native TMR2B PWM mode is returning "not supported."
     */

    vibration_pwm_set_percent(duty_percent);

    g_dma_complete_count++;

    /*
    * Timestamp immediately after the HAL has written the PWM configuration.
    */
    latency_measurement_pwm_updated();
    //LED_Off(0);
    return;
}

int iim_start_accel_dma(void)
{
    int status;
    uint8_t index;

    // there should only be one SPI operation at a time
    if (g_spi_dma_busy) {
        g_dma_overrun_count++;
        return E_BUSY;
    }

    index = g_dma_write_index;

    g_spi_dma_req.rxData     = g_rx_buffer[index];

    /*
     * Set busy before calling the driver. This prevents a new DRDY edge/Tmer1B task from
     * entering the start routine while the transfer is being configured.
     */
    g_spi_dma_busy = true;

    status = MXC_SPI_MasterTransactionDMA(&g_spi_dma_req);

    if (status != E_NO_ERROR) {
    	g_spi_dma_busy = false;
    	g_spi_dma_status = status;
    	g_dma_error_count++;
    }

    return status;
}
void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

void DMA1_IRQHandler(void)
{
    MXC_DMA_Handler();
}


/* --------------------------------------------------------------------------
 * SPI and DMA initialization
 * -------------------------------------------------------------------------- */

int iim_spi_dma_init(void)
{
	int err;

	/*
	 * Initialize the DMA controller.
	 */
	err = MXC_DMA_Init();
	if (err != E_NO_ERROR) {
		return err;
	}

	/*
	 * Enable the DMA channel interrupts.
	 *
	 * MXC_SPI_MasterTransactionDMA() allocates/configures the required
	 * DMA channels through the MSDK peripheral driver.
	 */
	NVIC_ClearPendingIRQ(DMA0_IRQn);
	NVIC_EnableIRQ(DMA0_IRQn);

	NVIC_ClearPendingIRQ(DMA1_IRQn);
	NVIC_EnableIRQ(DMA1_IRQn);

	/*
	 * Initialize SPI0 as:
	 *   master            = 1
	 *   quad mode         = 0
	 *   slave count       = 1
	 *   SS active low     = 0
	 *   SPI clock         = 8 MHz
	 *
	 * Some MSDK releases include a pin-map argument. Use the prototype
	 * from the installed spi.h if this call reports an argument mismatch.
	 */
	err = MXC_SPI_Init(IIM_SPI,
			1,
			0,
			1,
			0,
			IIM_SPI_SPEED_HZ);
	if (err != E_NO_ERROR) {
		return err;
	}

	err = MXC_SPI_SetWidth(IIM_SPI, SPI_WIDTH_STANDARD);
	if (err != E_NO_ERROR) {
		return err;
	}

	err = MXC_SPI_SetDataSize(IIM_SPI, 8);
	if (err != E_NO_ERROR) {
		return err;
	}

	/*
	 * IIM-42352 timing:
	 *   data transitions on falling SCLK
	 *   data is captured on rising SCLK
	 *
	 * This corresponds to SPI mode 0.
	 *
	 * If MXC_SPI_SetMode() is not present in your MSDK release, configure
	 * the equivalent clock polarity and phase using that release's API.
	 */
	err = MXC_SPI_SetMode(IIM_SPI, SPI_MODE_0);
	if (err != E_NO_ERROR) {
		return err;
	}

	memset(&g_spi_dma_req, 0, sizeof(g_spi_dma_req));
	memset(g_rx_buffer, 0, sizeof(g_rx_buffer));

	g_spi_command = IIM_REG_ACCEL_DATA_X1 | IIM_SPI_READ_BIT;
	g_dma_write_index = 0u;
	//do not modify g_spi_dma_req elsewhere
	g_spi_dma_req.spi        = IIM_SPI;
	g_spi_dma_req.txData     = &g_spi_command;
	g_spi_dma_req.rxData     = g_rx_buffer[g_dma_write_index];
	/*
	 * One command byte followed by six received data bytes.
	 */
	g_spi_dma_req.txLen      = IIM_ACCEL_BYTES+1;
	g_spi_dma_req.rxLen      = IIM_ACCEL_BYTES+1;

	g_spi_dma_req.ssIdx      = IIM_SPI_SS_IDX;
	g_spi_dma_req.ssDeassert = 1;

	/*
	 * Initialize driver-maintained transfer counters before each transfer.
	 */
	g_spi_dma_req.txCnt      = 0u;
	g_spi_dma_req.rxCnt      = 0u;

	g_spi_dma_req.completeCB = (void *)iim_spi_dma_callback;


	g_dma_write_index = 0u;
	g_spi_dma_busy = false;
	g_spi_dma_status = E_NO_ERROR;

	return E_NO_ERROR;
}

#if DRDY_INTERRUPT_WIRED
static void iim_drdy_gpio_callback(void *callback_data)
{
    int err;

    (void)callback_data;

    g_drdy_count++;

    /*
     * At most one SPI transaction may be active on SPI0.
     */
    if (g_spi_dma_busy) {
        g_dma_overrun_count++;
        return;
    }

    err = iim_start_accel_dma();

    if (err != E_NO_ERROR) {
        /*
         * Record the error only. Do not print from interrupt context.
         */
        g_spi_dma_status = err;
    }
}
static mxc_gpio_cfg_t g_iim_int_gpio_cfg =
{
    .port  = IIM_INT_PORT,
    .mask  = IIM_INT_MASK,
    .func  = MXC_GPIO_FUNC_IN,
    .pad   = MXC_GPIO_PAD_NONE,
    .vssel = MXC_GPIO_VSSEL_VDDIO
};

int iim_drdy_interrupt_init(void)
{
    int err;

    err = MXC_GPIO_Config(&g_iim_int_gpio_cfg);
    if (err != E_NO_ERROR) {
        return err;
    }

    /*
     * This assumes IIM-42352 INT1 is configured as active-high and pulsed.
     * Use falling-edge or level triggering if the sensor interrupt polarity
     * and drive mode are configured differently.
     */
    MXC_GPIO_RegisterCallback(&g_iim_int_gpio_cfg,
                              iim_drdy_gpio_callback,
                              NULL);

    MXC_GPIO_IntConfig(&g_iim_int_gpio_cfg,
                       MXC_GPIO_INT_RISING);

    MXC_GPIO_EnableInt(IIM_INT_PORT, IIM_INT_MASK);

    /*
     * Verify the appropriate GPIO IRQ name in the MAX32672 device header.
     * Some MSDK releases provide one IRQ per GPIO port.
     */
    NVIC_ClearPendingIRQ(GPIO0_IRQn);
    NVIC_EnableIRQ(GPIO0_IRQn);

    return E_NO_ERROR;
}

void GPIO0_IRQHandler(void)
{
    /*
     * The MSDK handler identifies the asserted GPIO pin, clears its interrupt
     * status, and calls the registered callback.
     */
    MXC_GPIO_Handler(IIM_INT_PORT);
}
#endif



