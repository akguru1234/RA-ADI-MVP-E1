/****************************************************************************************
* Notes:																				*
* The integration below connects the SPI-DMA acquisition path to a practical 			*
* vibration-processing pipeline: DMA sample capture → lock-free sample ring → 			*
* 1024-point frame → DC removal → Hann window → CMSIS-DSP real FFT → RMS, peak, 		*
* crest factor, dominant frequency, and band-energy metrics. It is structured so 		*
* that interrupt handlers only move data and publish events, while all floating-point	*
* and FFT work executes in the main context.											*
*																						*													
****************************************************************************************/
/******************************************************************************
 * MAX32672 + IIM-42352
 * SPI0 DMA acceleration acquisition example
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
 *
 *******************************************************************************
 *
 *					IIM-42352 data-ready at 8 kHz (Tiner interrupt)
 *							  |
 *							  v
 *					TRM1B ISR starts SPI DMA
 *							  |
 *							  v
 *					DMA IRQ calls MXC_DMA_Handler()
 *							  |
 *							  v
 *					SPI callback decodes 6 bytes
 *							  |
 *							  v
 *					Raw XYZ sample enters ring
 *							  |
 * 							  v
 *					Main loop calibrates sample
 *							  |
 *							  v
 *					1024-sample vibration frame
 *							  |
 *							  v
 *					DC removal + Hann window
 *							  |
 *							  v
 *					CMSIS-DSP 1024-point RFFT
 *							  |
 *							  v
 *					RMS, peak, crest factor, dominant frequency, bands
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
 * FIFO Parser definitions
 * -------------------------------------------------------------------------- */
#define TOTAL_PACKETS      			256   	//2048 bytes / 8 bytes per packet = full IIM-42352 fifo size of 2048
#define NUM_OF_PACKETS_PER_SPI_READ	4u		//read 32 bytes at a time
#define FIFO_BUFFER_SIZE 			PACKET_SIZE*TOTAL_PACKETS
#define SAMPLE_RATE_HZ 				8000.0f

/* --------------------------------------------------------------------------
 * iim-42352 interface handlers
 * -------------------------------------------------------------------------- */

/*
 * Ping-pong receive buffers.
 */

/*
 * DMA writes into write_index. After completion, that buffer becomes ready
 * for application processing and write_index advances to the other buffer.
 */

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} iim_accel_sample_t;

typedef struct
{
    float x_g;
    float y_g;
    float z_g;
} accel_sample_g_t;

/*At an 8 kHz sample rate and 1024-point FFT:
Frame duration: 128 ms
Frequency-bin spacing: 7.8125 Hz
One-sided analysis range: 0 to 4 kHz
Frame overlap in this example: 50 percent
New result rate: approximately 15.625 frames/s*/

/*CMSIS-DSP supports floating-point real FFT lengths from 32 through 4096 samples.
Its arm_rfft_fast_f32() function modifies the input buffer, so the implementation
maintains separate frame, FFT-input, and FFT-output arrays*/


/* --------------------------------------------------------------------------
 * Acquisition and FFT configuration
 * -------------------------------------------------------------------------- */

#define FFT_SIZE                   1024u
#define FFT_HOP_SIZE               512u
#define FFT_UNIQUE_BINS            (FFT_SIZE / 2u + 1u)

#define SAMPLE_RING_SIZE           2048u
#define SAMPLE_RING_MASK           (SAMPLE_RING_SIZE - 1u)

#if ((SAMPLE_RING_SIZE & SAMPLE_RING_MASK) != 0)
#error "SAMPLE_RING_SIZE must be a power of two"
#endif

/*
 * Sensitivity for ACCEL_FS_SEL = 000, ±16 g:
 * 32768 counts / 16 g = 2048 counts/g.
 *
 * Change this if ACCEL_CONFIG0 selects a different full-scale range.
 */
#define ACCEL_LSB_PER_G            2048.0f
#define STANDARD_GRAVITY_MPS2      9.80665f

/*
 * Reject the lowest bins when finding the dominant vibration frequency.
 * This avoids reporting residual DC, structural drift, or very slow motion.
 */
#define DOMINANT_FREQ_MIN_HZ       10.0f
#define DOMINANT_FREQ_MAX_HZ       3900.0f

static accel_calibration_t g_accel_cal = // @suppress("Unused variable declaration in file scope")
{
    .bias_x_g = 0.0f,
    .bias_y_g = 0.0f,
    .bias_z_g = -1.0f,

    .scale_x = 1.0f,
    .scale_y = 1.0f,
    .scale_z = 1.0f
};

/* --------------------------------------------------------------------------
 * Single-producer, single-consumer sample ring
 *
 * Producer: SPI DMA completion callback
 * Consumer: main processing loop
 * -------------------------------------------------------------------------- */

volatile accel_raw_sample_t g_sample_ring[SAMPLE_RING_SIZE];

volatile uint32_t g_ring_write;
volatile uint32_t g_ring_read;
volatile uint32_t g_ring_overruns;
volatile bool g_vibration_result_ready; // @suppress("Unused variable declaration in file scope")
volatile vibration_result_t g_vibration_result;

/* --------------------------------------------------------------------------
 * FFT working storage
 * -------------------------------------------------------------------------- */
static float g_frame[FFT_SIZE];
static uint32_t g_frame_samples;
static bool g_frame_ready;
volatile uint32_t g_frame_overrun;
static arm_rfft_fast_instance_f32 g_rfft;

static float g_hann[FFT_SIZE];
static float g_fft_input[FFT_SIZE];
static float g_fft_output[FFT_SIZE];
static float g_power[FFT_UNIQUE_BINS];

static float g_hann_sum;
static float g_hann_power_sum;

static int vibration_pwm_pin_init(void)
{
    mxc_gpio_cfg_t pwm_pin;

    memset(&pwm_pin, 0, sizeof(pwm_pin));

    pwm_pin.port = VIB_PWM_PORT;
    pwm_pin.mask = VIB_PWM_PIN;

    /*
     * P0.13 functions shown in the schematic:
     *
     * GPIO / I2C1_SDA / 32KCAL / TMR2B_O / AIN5
     *
     * Verify ALT3 against the MAX32672 pin header or MSDK pin-map table.
     */
    pwm_pin.func = MXC_GPIO_FUNC_ALT3;
    //pwm_pin.func = MXC_GPIO_FUNC_OUT;
    pwm_pin.pad = MXC_GPIO_PAD_NONE;
    pwm_pin.vssel = MXC_GPIO_VSSEL_VDDIO;
    pwm_pin.drvstr = MXC_GPIO_DRVSTR_0;

    return MXC_GPIO_Config(&pwm_pin);
}
int vibration_pwm_init(void)
{
    int status;
    mxc_tmr_cfg_t tmr_cfg;
    uint32_t initial_duty_ticks;

    g_PWM_update_error = 0;
    status = vibration_pwm_pin_init();

    if (status != E_NO_ERROR) {
    	g_PWM_update_error++;
        return status;
    }

    /*
     * Calculate timer period for the requested PWM carrier frequency.
     */
    g_pwm_period_ticks =
        MXC_TMR_GetPeriod(VIB_PWM_TIMER,
                          VIB_PWM_CLOCK,
                          VIB_PWM_PRESCALER,
                          VIB_PWM_CARRIER_HZ);

    if (g_pwm_period_ticks == 0u) {
        return E_BAD_PARAM;
    }

    initial_duty_ticks = g_pwm_period_ticks / 2u;

    MXC_TMR_Shutdown(VIB_PWM_TIMER);

    memset(&tmr_cfg, 0, sizeof(tmr_cfg));

    tmr_cfg.pres = VIB_PWM_PRESCALER_ENUM;
    tmr_cfg.mode = TMR_MODE_PWM;
    //tmr_cfg.mode = TMR_MODE_COMPARE;

    /*
     * TMR2B uses the B half of Timer 2.
     */
    tmr_cfg.bitMode = TMR_BIT_MODE_16A;//TMR_BIT_MODE_16B
    tmr_cfg.clock = VIB_PWM_CLOCK;
    tmr_cfg.cmp_cnt = g_pwm_period_ticks; //sets up PWM frequency
    tmr_cfg.pol = 0u;

    /*
     * The final MXC_TMR_Init argument selects timer output operation
     * on MSDK timer implementations that expose this option.
     */
    status = MXC_TMR_Init(VIB_PWM_TIMER,
                          &tmr_cfg,
                          false);

    if (status != E_NO_ERROR) {
        return status;
    }

    /*
     * Initialize the output at 50% duty cycle.
     */
    status = MXC_TMR_SetPWM(VIB_PWM_TIMER, initial_duty_ticks );

    if (status != E_NO_ERROR) {
        MXC_TMR_Shutdown(VIB_PWM_TIMER);
        g_PWM_update_error++;
        return status;
    }

    g_pwm_duty_percent = 50.0f;

   MXC_TMR_Start(VIB_PWM_TIMER);
   return status ;
}

bool sample_ring_push_from_isr(const accel_raw_sample_t *sample)
{
    uint32_t write = g_ring_write;
    uint32_t next = (write + 1u) & SAMPLE_RING_MASK;
   // LED_On(0);
    if (next == g_ring_read) {
        g_ring_overruns++;
        return false;
    }

    g_sample_ring[write] = *sample;

    /*
     * Ensure sample data reaches memory before publishing the new index.
     */
    __DMB();
    g_ring_write = next;

    //LED_Off(0);
    return true;
}

static bool sample_ring_pop(accel_raw_sample_t *sample)
{
	uint32_t read = g_ring_read;

	if (read == g_ring_write) {
		return false;
	}
	uint32_t primask;

	primask = __get_PRIMASK();
	__disable_irq();

	*sample = g_sample_ring[read];//pop from ring buffer
	__DMB();
	g_ring_read = (read + 1u) & SAMPLE_RING_MASK;

	if (primask == 0u) {
		__enable_irq();
	}

	return true;
}

/* --------------------------------------------------------------------------
 * FFT Frame assembly
 * -------------------------------------------------------------------------- */
static accel_sample_g_t calibrate_sample(const accel_raw_sample_t *raw)
{
    accel_sample_g_t sample;

    float x = (float)raw->x / ACCEL_LSB_PER_G;
    float y = (float)raw->y / ACCEL_LSB_PER_G;
    float z = (float)raw->z / ACCEL_LSB_PER_G;

    sample.x_g = x * g_accel_cal.scale_x - g_accel_cal.bias_x_g;
    sample.y_g = y * g_accel_cal.scale_y - g_accel_cal.bias_y_g;
    sample.z_g = z * g_accel_cal.scale_z - g_accel_cal.bias_z_g;

    return sample;
}

static float select_vibration_signal(const accel_sample_g_t *sample)
{
	/*
	 * Option A: vector magnitude.
	 */
	/*  return sqrtf(sample->x_g * sample->x_g +
                 sample->y_g * sample->y_g +
                 sample->z_g * sample->z_g);*/

	/*
	 * Option B: analyze one physical axis instead:
	 */
	if (g_selected_channel == VIB_PWM_CHANNEL_X){
		return sample->x_g;
	}
	else if (g_selected_channel == VIB_PWM_CHANNEL_Y){
		return sample->y_g;
	}
	else if (g_selected_channel == VIB_PWM_CHANNEL_Z){
		return sample->z_g;
	}
	else if (g_selected_channel == VIB_PWM_CHANNEL_MAGNITUDE){
		return sqrtf(sample->x_g * sample->x_g +
		                 sample->y_g * sample->y_g +
		                 sample->z_g * sample->z_g);
	}
	return 0.0f;
}

static void vibration_ingest_sample(const accel_raw_sample_t *raw)
{
    accel_sample_g_t calibrated;

    if (g_frame_ready) {
        /*
         * Do not overwrite a complete frame before it is processed.
         * A production version should count this as a processing overrun.
         */
    	g_frame_overrun++;
        return;
    }

    calibrated = calibrate_sample(raw);

    g_frame[g_frame_samples] = select_vibration_signal(&calibrated);
    g_frame_samples++;

    if (g_frame_samples == FFT_SIZE) {
        g_frame_ready = true;
    }
}

int vibration_processing_init(void)
{
	uint32_t n;
    arm_status status;
    status = arm_rfft_fast_init_f32(&g_rfft, FFT_SIZE);

    if (status != ARM_MATH_SUCCESS) {
        return E_BAD_STATE;
    }

    g_hann_sum = 0.0f;
    g_hann_power_sum = 0.0f;

    for (n = 0u; n < FFT_SIZE; n++) {
        float phase =
           (2.0f * PI * (float)n) * (float)(FFT_SIZE - 1u);

        g_hann[n] = 0.5f - 0.5f * cosf(phase);

        g_hann_sum += g_hann[n];
        g_hann_power_sum += g_hann[n] * g_hann[n];
    }

    return E_NO_ERROR;
}
static void calculate_time_metrics(const float *frame,
                                   vibration_result_t *result)
{
    uint32_t n;
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float min_value = frame[0];
    float max_value = frame[0];
    float peak = 0.0f;

    /*
     * Calculate and remove the frame mean. This suppresses gravity and
     * static mounting orientation for acceleration vibration analysis.
     */
    for (n = 0u; n < FFT_SIZE; n++) {
        sum += frame[n];
    }

    {
        float mean = sum / (float)FFT_SIZE;

        for (n = 0u; n < FFT_SIZE; n++) {
            float ac = frame[n] - mean;
            float absolute = fabsf(ac);

            g_fft_input[n] = ac * g_hann[n];

            sum_sq += ac * ac;

            if (ac < min_value) {
                min_value = ac;
            }

            if (ac > max_value) {
                max_value = ac;
            }

            if (absolute > peak) {
                peak = absolute;
            }
        }
    }

    result->rms_g = sqrtf(sum_sq / (float)FFT_SIZE);
    result->peak_g = peak;
    result->peak_to_peak_g = max_value - min_value;

    if (result->rms_g > 1.0e-12f) {
        result->crest_factor = result->peak_g / result->rms_g;
    } else {
        result->crest_factor = 0.0f;
    }
}
static uint32_t frequency_to_bin(float frequency_hz)
{
    float bin = frequency_hz * (float)FFT_SIZE / SAMPLE_RATE_HZ;
    uint32_t result = (uint32_t)(bin + 0.5f);

    if (result > (FFT_SIZE / 2u)) {
        result = FFT_SIZE / 2u;
    }

    return result;
}

static float integrate_power_band(float low_hz, float high_hz)
{
    uint32_t k;
    uint32_t first = frequency_to_bin(low_hz);
    uint32_t last = frequency_to_bin(high_hz);
    float power = 0.0f;

    if (first < 1u) {
        first = 1u;
    }

    if (last > FFT_SIZE / 2u) {
        last = FFT_SIZE / 2u;
    }

    for (k = first; k <= last; k++) {
        power += g_power[k];
    }

    return power;
}

static void calculate_spectrum(vibration_result_t *result)
{
    uint32_t k;
    uint32_t first_dominant_bin;
    uint32_t last_dominant_bin;
    uint32_t dominant_bin = 0u;

    float maximum_power = 0.0f;
    float amplitude_scale;

    /*
     * arm_rfft_fast_f32 modifies g_fft_input.
     */
    arm_rfft_fast_f32(&g_rfft,
                      g_fft_input,
                      g_fft_output,
                      0u);

    /*
     * Coherent-gain correction for sinusoidal amplitude:
     *
     * one-sided peak amplitude =
     *     2 * |FFT bin| / sum(window)
     */
    amplitude_scale = 2.0f / g_hann_sum;

    /*
     * Packed RFFT layout:
     *
     * output[0] = DC real value
     * output[1] = Nyquist real value
     * output[2*k]     = real part of bin k
     * output[2*k + 1] = imaginary part of bin k
     */
    g_power[0] = g_fft_output[0] * g_fft_output[0];

    for (k = 1u; k < FFT_SIZE / 2u; k++) {
        float real = g_fft_output[2u * k];
        float imag = g_fft_output[2u * k + 1u];

        /*
         * Store squared, coherent-gain-corrected peak amplitude.
         */
        g_power[k] =
            (real * real + imag * imag) *
            amplitude_scale * amplitude_scale;
    }

    g_power[FFT_SIZE / 2u] =
        g_fft_output[1] * g_fft_output[1] *
        amplitude_scale * amplitude_scale;

    first_dominant_bin =
        frequency_to_bin(DOMINANT_FREQ_MIN_HZ);

    last_dominant_bin =
        frequency_to_bin(DOMINANT_FREQ_MAX_HZ);

    if (first_dominant_bin < 1u) {
        first_dominant_bin = 1u;
    }

    if (last_dominant_bin >= FFT_SIZE / 2u) {
        last_dominant_bin = FFT_SIZE / 2u - 1u;
    }

    for (k = first_dominant_bin; k <= last_dominant_bin; k++) {
        if (g_power[k] > maximum_power) {
            maximum_power = g_power[k];
            dominant_bin = k;
        }
    }

    result->dominant_frequency_hz =
        ((float)dominant_bin * SAMPLE_RATE_HZ) / (float)FFT_SIZE;

    result->dominant_amplitude_g = sqrtf(maximum_power);

    result->band_10_100_hz_g2 =
        integrate_power_band(10.0f, 100.0f);

    result->band_100_500_hz_g2 =
        integrate_power_band(100.0f, 500.0f);

    result->band_500_1000_hz_g2 =
        integrate_power_band(500.0f, 1000.0f);

    result->band_1000_2000_hz_g2 =
        integrate_power_band(1000.0f, 2000.0f);

    result->band_2000_4000_hz_g2 =
        integrate_power_band(2000.0f, 4000.0f);
}
static void process_vibration_frame(void)
{
    static uint32_t result_sequence;

    vibration_result_t result;

    if (!g_frame_ready) {
        return;
    }

    memset(&result, 0, sizeof(result));

    calculate_time_metrics(g_frame, &result);
    calculate_spectrum(&result);

    result.sequence = ++result_sequence;
    result.acquisition_overruns = g_ring_overruns;

    /*
     * Publish the completed result.
     */
    g_vibration_result = result;
    __DMB();
    g_vibration_result_ready = true;

    /*
     * Retain the second half of the frame to implement 50% overlap.
     */
    memmove(&g_frame[0],
            &g_frame[FFT_HOP_SIZE],
            FFT_HOP_SIZE * sizeof(g_frame[0]));

    g_frame_samples = FFT_HOP_SIZE;
    g_frame_ready = false;
}
void vibration_service(void)
{
    accel_raw_sample_t sample;

    /*
     * Empty the acquisition ring into the current FFT frame.
     */
    while ((!g_frame_ready) && sample_ring_pop(&sample)) {
        vibration_ingest_sample(&sample);
    }

    /*
     * Run the complete frame outside interrupt context.
     */
    if (g_frame_ready) {
        process_vibration_frame();
    }
}

void publish_vibration_result(
    const vibration_result_t *result)
{
    /*
     * Illustrative output. In the real-time build, prefer a binary packet
     * or shared diagnostics structure rather than frequent formatted I/O.
     */
    printf("Frame %lu\r\n",
           (unsigned long)result->sequence);

    printf("RMS       : %.6f g, %.6f m/s^2\r\n",
           (double)result->rms_g,
           (double)(result->rms_g * STANDARD_GRAVITY_MPS2));

    printf("Peak      : %.6f g\r\n",
           (double)result->peak_g);

    printf("Pk-Pk     : %.6f g\r\n",
           (double)result->peak_to_peak_g);

    printf("Crest     : %.3f\r\n",
           (double)result->crest_factor);

    printf("Dominant  : %.2f Hz, %.6f g peak\r\n",
           (double)result->dominant_frequency_hz,
           (double)result->dominant_amplitude_g);

    printf("Overruns  : %lu\r\n\r\n",
           (unsigned long)result->acquisition_overruns);

}

