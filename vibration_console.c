/*
 * vibration_console.c
 *
 *  Created on: Sep 25, 2026
 *      Author: akguru
 *
 *
 *	Console Commands:
 *	--------		 ------------------------
 *	command			  Operation
 *	--------		  -----------------------
 *	x                 Output X-axis vibration
 *	y                 Output Y-axis vibration
 *	z                 Output Z-axis vibration
 *	mag               Output vector magnitude o
 *	Off               Force PWM to 50%, representing zero signal
 *	status            Display current configuration and counters
 *	range <g>         Set full-scale PWM mapping, for example: range 4
 *	dc on             Enable DC-removal filter
 *	dc off            Disable DC-removal filter
 *	zero              Capture current selected-axis value as static zero offset
 *	help              Display commands
 *	clearzero		  Clear captured offsets
 *	clearall		  Clear all records
 *	fft				  Prints last FFT of last 1024 points
 */

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_errors.h"
#include "uart.h"

#include "IIM42352.h"
#include "vibration_console.h"
#include "IIM42352-Vibration-FFT.h"
#include "IIM42352_Spi_dma_pwm_out.h"
#include "latency_measurement.h"
#include "adi_encoder_test.h"

/* --------------------------------------------------------------------------
 * Existing application APIs and diagnostics
 * -------------------------------------------------------------------------- */

/*
 * Existing PWM API from current implementation on E1 board.
 *
 * The implementation uses the working TMR2A PWM configuration while routing
 * the output to the TMR2B pin.
 */

extern volatile float g_pwm_duty_percent;

extern volatile uint32_t g_dma_complete_count;
//extern volatile uint32_t g_dma_error_count;
//extern volatile uint32_t g_dma_overrun_count;

static void print_latency_status(void);
/* --------------------------------------------------------------------------
 * Console configuration
 * -------------------------------------------------------------------------- */
#define CONSOLE_UART1 MXC_UART1

#define CONSOLE_LINE_LENGTH            	48u
#define CONSOLE_COMMAND_LENGTH			16u

/* --------------------------------------------------------------------------
 * PWM mapping
 * -------------------------------------------------------------------------- */

#define PWM_CENTER_PERCENT             50.0f
#define PWM_MIN_PERCENT                 5.0f
#define PWM_MAX_PERCENT                95.0f

#define DEFAULT_PWM_RANGE_G             4.0f
#define MINIMUM_PWM_RANGE_G				0.05f
#define MAXIMUM_PWM_RANGE_G				16.0f


#define DEFAULT_DC_ALPHA				0.0005f
/*
 * DC estimate:
 *
 * dc[n] = dc[n-1] + alpha * (sample[n] - dc[n-1])
 * ac[n] = sample[n] - dc[n]
 */
#define DC_TRACKING_ALPHA               0.0005f

/* --------------------------------------------------------------------------
 * Parser and signal-processing state
 * -------------------------------------------------------------------------- */

static char g_command_buffer[CONSOLE_LINE_LENGTH];
static uint32_t g_command_length;

volatile vibration_channel_t g_selected_channel = VIB_PWM_CHANNEL_X;

volatile float g_pwm_input_range_g =   DEFAULT_PWM_RANGE_G;

static volatile float g_latest_x_g;
static volatile float g_latest_y_g;
static volatile float g_latest_z_g;

static volatile float g_max_x_g;
static volatile float g_max_y_g;
static volatile float g_max_z_g;

static volatile float g_zero_x_g;
static volatile float g_zero_y_g;
static volatile float g_zero_z_g;

static float g_dc_x_g;
static float g_dc_y_g;
static float g_dc_z_g;
static float g_dc_magnitude_g;

static bool g_dc_removal_enabled;

static volatile uint32_t g_console_command_count;
static volatile uint32_t g_console_error_count;
static volatile uint32_t g_console_overflow_count;

/* --------------------------------------------------------------------------
 * Utility functions
 * -------------------------------------------------------------------------- */

static char* skip_spaces(char *text)
{
    while ((*text == ' ') || (*text == '\t')) {
        text++;
    }

    return text;
}

static void trim_trailing_spaces(char *text)
{
    size_t length = strlen(text);

    while (length > 0u) {
        char character = text[length - 1u];

        if ((character != ' ') &&
            (character != '\t')) {
            break;
        }

        text[length - 1u] = '\0';
        length--;
    }
}

static void command_to_lowercase(char *text)
{
    while (*text != '\0') {
        *text = (char)tolower((unsigned char)*text);
        text++;
    }
}

static const char *channel_name(vibration_channel_t channel)
{
    switch (channel) {
    case VIB_PWM_CHANNEL_X:
        return "X";

    case VIB_PWM_CHANNEL_Y:
        return "Y";

    case VIB_PWM_CHANNEL_Z:
        return "Z";

    case VIB_PWM_CHANNEL_MAGNITUDE:
        return "MAGNITUDE";

    case VIB_PWM_CHANNEL_OFF:
        return "OFF";

    default:
        return "INVALID";
    }
}

static void print_help(void)
{
    printf("\r\nVibration PWM commands:\r\n");
    printf("  x              Output X-axis vibration\r\n");
    printf("  y              Output Y-axis vibration\r\n");
    printf("  z              Output Z-axis vibration\r\n");
    printf("  mag            Output vector magnitude\r\n");
    printf("  off            Force PWM to zero-signal level\r\n");
    printf("  status         Display current configuration\r\n");
    printf("  range <g>      Set PWM full-scale range\r\n");
    printf("  dc on          Enable DC-removal filter\r\n");
    printf("  dc off         Disable DC-removal filter\r\n");
    printf("  zero           Capture current sample as offset\r\n");
    printf("  clearzero      Clear captured offsets\r\n");
    printf("  clearall       Clear all records\r\n");
    printf("  fft			 Print FFT results of last 1024 samples");
    printf("  help           Display this command list\r\n");
    printf("  latency        Display TMR1B-to-PWM latency\r\n");
    printf("  latency reset  Clear latency statistics\r\n");
    printf("  mvp   		 Test ADI MVP Encoder Communications\r\n");
    printf("\r\n");
}

static void print_status(void)
{
    printf("\r\nVibration PWM status\r\n");

    printf("  Channel       : %s\r\n",
           channel_name(g_selected_channel));

    printf("  Input range   : +/- %.3f g\r\n",
           (double)g_pwm_input_range_g);

    printf("  DC removal    : %s\r\n",
    		g_dc_removal_enabled ?  "enabled" : "disabled");

    printf("  Zero X        : %.5f g\r\n",
           (double)g_zero_x_g);

    printf("  Zero Y        : %.5f g\r\n",
           (double)g_zero_y_g);

    printf("  Zero Z        : %.5f g\r\n",
           (double)g_zero_z_g);

    printf("  Latest X      : %.5f g\r\n",
           (double)g_latest_x_g);

    printf("  Latest Y      : %.5f g\r\n",
           (double)g_latest_y_g);

    printf("  Latest Z      : %.5f g\r\n",
           (double)g_latest_z_g);

    printf("  Max X      : %.5f g\r\n",
           (double)g_max_x_g);

    printf("  Max Y      : %.5f g\r\n",
           (double)g_max_y_g);

    printf("  Max Z      : %.5f g\r\n",
           (double)g_max_z_g);

    printf("  PWM duty      : %.3f %%\r\n",
           (double)g_pwm_duty_percent);

    printf("  PWM Update Errors      : %lu\r\n",
               (unsigned long)g_PWM_update_error);

    printf("  DMA complete  : %lu\r\n",
           (unsigned long)g_dma_complete_count);

    printf("  DMA errors    : %lu\r\n",
           (unsigned long)g_dma_error_count);

    printf("  DMA overruns  : %lu\r\n",
           (unsigned long)g_dma_overrun_count);

    printf("  Commands      : %lu\r\n",
           (unsigned long)g_console_command_count);

    printf("  Command errors: %lu\r\n",
           (unsigned long)g_console_error_count);

    printf("  RX overflows  : %lu\r\n\r\n",
           (unsigned long)g_console_overflow_count);

    printf("  FFT Frame overflows  : %lu\r\n\r\n",
               (unsigned long)g_frame_overrun);

    print_latency_status(); //used to measure timing of tasks

}
#if 0
void print_latency_status(void)
{
    latency_statistics_t statistics;
    uint32_t average_cycles;

    latency_measurement_get_statistics(&statistics);

    printf("\r\nTMR1B-to-PWM latency\r\n");
    printf("--------------------\r\n");

    printf("Samples         : %lu\r\n",
           (unsigned long)statistics.sample_count);

    if (statistics.sample_count == 0u) {
        printf("No completed measurements\r\n");
        return;
    }

    average_cycles =
        (uint32_t)(
            statistics.total_cycles /
            (uint64_t)statistics.sample_count);

    printf("Last cycles     : %lu\r\n",
           (unsigned long)statistics.last_cycles);

    printf("Minimum cycles  : %lu\r\n",
           (unsigned long)statistics.min_cycles);

    printf("Maximum cycles  : %lu\r\n",
           (unsigned long)statistics.max_cycles);

    printf("Average cycles  : %lu\r\n",
           (unsigned long)average_cycles);

    printf("Last latency    : %.3f us\r\n",
           (double)latency_cycles_to_us(
               statistics.last_cycles));

    printf("Minimum latency : %.3f us\r\n",
           (double)latency_cycles_to_us(
               statistics.min_cycles));

    printf("Maximum latency : %.3f us\r\n",
           (double)latency_cycles_to_us(
               statistics.max_cycles));

    printf("Average latency : %.3f us\r\n",
           (double)latency_cycles_to_us(
               average_cycles));

    printf("Missed starts   : %lu\r\n",
           (unsigned long)
           statistics.overwritten_start_count);

    printf("Orphan updates  : %lu\r\n",
           (unsigned long)
           statistics.update_without_start_count);
}
#else
static void print_latency_status(void)
{
    latency_statistics_t statistics;
    uint32_t average_ticks;

    latency_measurement_get_statistics(
        &statistics);

    printf("\r\nTMR1B-to-PWM latency\r\n");

    printf("Timer frequency : %lu Hz\r\n",
           (unsigned long)
           latency_measurement_get_timer_frequency_hz());

    printf("Samples         : %lu\r\n",
           (unsigned long)
           statistics.sample_count);

    if (statistics.sample_count == 0u) {
        printf("No completed measurements\r\n");
        return;
    }

    average_ticks =
        (uint32_t)(
            statistics.total_ticks /
            (uint64_t)statistics.sample_count);

    printf("Last ticks      : %lu\r\n",
           (unsigned long)
           statistics.last_ticks);

    printf("Minimum ticks   : %lu\r\n",
           (unsigned long)
           statistics.min_ticks);

    printf("Maximum ticks   : %lu\r\n",
           (unsigned long)
           statistics.max_ticks);

    printf("Average ticks   : %lu\r\n",
           (unsigned long)
           average_ticks);

    printf("Last latency    : %lu ns\r\n",
           (unsigned long)
           latency_measurement_ticks_to_ns(
               statistics.last_ticks));

    printf("Minimum latency : %lu ns\r\n",
           (unsigned long)
           latency_measurement_ticks_to_ns(
               statistics.min_ticks));

    printf("Maximum latency : %lu ns\r\n",
           (unsigned long)
           latency_measurement_ticks_to_ns(
               statistics.max_ticks));

}
#endif
static void check_and_publish_FFT_results (void) {
	vibration_result_t result;
	uint32_t primask;

	if (g_vibration_result_ready) {

		primask = __get_PRIMASK();
		__disable_irq();

		result = g_vibration_result;	//last available result
		g_vibration_result_ready = false;//set up for next time

		if (primask == 0u) {
			__enable_irq();
		}


		publish_vibration_result(&result);
	}
}

static void select_channel(
		vibration_channel_t channel)
{
    g_selected_channel = channel;

    /*
     * Reset DC estimators when changing channels so an old channel's
     * operating point does not create a transient.
     */
    g_dc_x_g = 0.0f;
    g_dc_y_g = 0.0f;
    g_dc_z_g = 0.0f;
    g_dc_magnitude_g = 0.0f;

    printf("PWM vibration channel = %s\r\n",
           channel_name(channel));
}

static void process_range_command(char *argument)
{
    char *end_pointer;
    float requested_range;

    argument = skip_spaces(argument);

    if (*argument == '\0') {
        printf("Current PWM range = +/- %.3f g\r\n",
               (double)g_pwm_input_range_g);
        return;
    }

    end_pointer = NULL;

    requested_range =
        strtof(argument, &end_pointer);

    if ((end_pointer == argument)) {
        g_console_error_count++;

        printf("ERROR: range requires a numeric value\r\n");
        printf("Example: range 4\r\n");
        return;
    }

    if ((requested_range < MINIMUM_PWM_RANGE_G) ||
        (requested_range > MAXIMUM_PWM_RANGE_G)) {
        g_console_error_count++;

        printf("ERROR: range must be %.2f to %.2f g\r\n",
               (double)MINIMUM_PWM_RANGE_G,
               (double)MAXIMUM_PWM_RANGE_G);

        return;
    }

    g_pwm_input_range_g = requested_range;

    printf("PWM input range = +/- %.3f g\r\n",
           (double)requested_range);
}

static void process_dc_command(char *argument)
{
    argument = skip_spaces(argument);

    if (strcmp(argument, "on") == 0) {
        g_dc_removal_enabled = true;

        g_dc_x_g = 0.0f;
        g_dc_y_g = 0.0f;
        g_dc_z_g = 0.0f;
        g_dc_magnitude_g = 0.0f;

        printf("DC removal enabled\r\n");
    } else if (strcmp(argument, "off") == 0) {
        g_dc_removal_enabled = false;

        printf("DC removal disabled\r\n");
    } else {
        g_console_error_count++;

        printf("ERROR: use 'dc on' or 'dc off'\r\n");
    }
}

static void capture_zero_offsets(void)
{
    uint32_t primask;
    float x;
    float y;
    float z;

    /*
     * Snapshot the values updated by the DMA callback.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    x = g_latest_x_g;
    y = g_latest_y_g;
    z = g_latest_z_g;

    if (primask == 0u) {
        __enable_irq();
    }

    g_zero_x_g = x;
    g_zero_y_g = y;
    g_zero_z_g = z;

    /*
     * Reset the adaptive DC filters after capturing static offsets.
     */
    g_dc_x_g = 0.0f;
    g_dc_y_g = 0.0f;
    g_dc_z_g = 0.0f;
    g_dc_magnitude_g = 0.0f;

    printf("Captured zero offsets: "
           "X=%.5f g Y=%.5f g Z=%.5f g\r\n",
           (double)x,
           (double)y,
           (double)z);
}

static void clear_zero_offsets(void)
{
    g_zero_x_g = 0.0f;
    g_zero_y_g = 0.0f;
    g_zero_z_g = 0.0f;

    g_dc_x_g = 0.0f;
    g_dc_y_g = 0.0f;
    g_dc_z_g = 0.0f;
    g_dc_magnitude_g = 0.0f;

    printf("Static zero offsets cleared\r\n");
}
static void clear_all_records(void)
{
    g_zero_x_g = 0.0f;
    g_zero_y_g = 0.0f;
    g_zero_z_g = 0.0f;

    g_dc_x_g = 0.0f;
    g_dc_y_g = 0.0f;
    g_dc_z_g = 0.0f;
    g_dc_magnitude_g = 0.0f;

    g_latest_x_g = 0.0f;
    g_latest_y_g = 0.0f;
    g_latest_z_g = 0.0f;

    g_max_x_g = 0.0f;
    g_max_y_g = 0.0f;
    g_max_z_g = 0.0f;

    printf("All stored records cleared\r\n");
}
static void process_command(char *command)
{
    char *argument;

    trim_trailing_spaces(command);

    command = skip_spaces(command);

    if (*command == '\0') {
        return;
    }

    command_to_lowercase(command);
    g_console_command_count++;

    if (strcmp(command, "x")==0) {
        select_channel(VIB_PWM_CHANNEL_X);
    } else if (strcmp(command, "y")==0) {
        select_channel(VIB_PWM_CHANNEL_Y);
    } else if (strcmp(command, "z")==0) {
        select_channel(VIB_PWM_CHANNEL_Z);
    } else if (strcmp(command, "mag")==0 ||
               strcmp(command, "magnitude")==0) {
        select_channel(VIB_PWM_CHANNEL_MAGNITUDE);
    } else if (strcmp(command, "off")==0) {
        select_channel(VIB_PWM_CHANNEL_OFF);

        /*
         * Zero signal is represented by 50% duty.
         */
        (void)vibration_pwm_set_percent(50.0f);
    } else if (strcmp(command, "status")==0 ||
    		strcmp(command, "sta")==0  ||
			strcmp(command, "st")==0 ) {
        print_status();
    } else if (strcmp(command, "help")==0 ||
               strcmp(command, "?")==0) {
        print_help();
    } else if (strcmp(command, "zero")==0) {
        capture_zero_offsets();
    } else if (strcmp(command, "clearzero")==0) {
        clear_zero_offsets();
    } else if (strcmp(command, "clearall")==0) {
           clear_all_records();
    }  else if (strcmp(command, "fft")==0) {
    	check_and_publish_FFT_results();
    }  else if (strncmp(command, "range", 5u) == 0) {
        argument = &command[5];

        process_range_command(argument);
    } else if (strncmp(command, "dc", 2u) == 0) {
        argument = &command[2];

        process_dc_command(argument);
    } else if (strcmp(command, "latency") == 0) {
    	print_latency_status();
    } else if (strcmp(command, "latency reset") == 0) {
    	latency_measurement_reset_statistics();

    	printf("Latency statistics reset\r\n");
    } else if (strcmp(command, "mvp") == 0) {
    	run_encoder_communication_test();
    } else {
    	g_console_error_count++;

    	printf("ERROR: unknown command '%s'\r\n",
    			command);

    	printf("Type 'help' for available commands\r\n");
    }
}

void vibration_console_init(void)
{
    g_command_length = 0u;
    memset(g_command_buffer,
           0,
           sizeof(g_command_buffer));

    g_selected_channel =
        VIB_PWM_CHANNEL_X;

    g_pwm_input_range_g =
        DEFAULT_PWM_RANGE_G;

    g_dc_removal_enabled = true;

    g_zero_x_g = 0.0f;
    g_zero_y_g = 0.0f;
    g_zero_z_g = 0.0f;

    g_latest_x_g = 0.0f;
    g_latest_y_g = 0.0f;
    g_latest_z_g = 0.0f;

    g_max_x_g = 0.0f;
    g_max_y_g = 0.0f;
    g_max_z_g = 0.0f;

    g_dc_x_g = 0.0f;
    g_dc_y_g = 0.0f;
    g_dc_z_g = 0.0f;
    g_dc_magnitude_g = 0.0f;

    printf("\r\nVibration PWM console ready\r\n");
    printf("Default channel: X, range: +/- %.1f g\r\n",
           (double)g_pwm_input_range_g);
    printf("Type 'help' for commands\r\n> ");
}

static void console_accept_character(char character)
{
    /*
     * Enter terminates the command. Accept CR, LF, or CR/LF.
     */
    if ((character == '\r') ||
        (character == '\n')) {
        if (g_command_length > 0u) {
            g_command_buffer[g_command_length] = '\0';

            printf("\r\n");

            process_command(g_command_buffer);

            g_command_length = 0u;
            g_command_buffer[0] = '\0';
        }

        printf("> ");
        return;
    }

    /*
     * Backspace and Delete.
     */
    if ((character == '\b') ||
        ((uint8_t)character == 0x7Fu)) {
        if (g_command_length > 0u) {
            g_command_length--;

            /*
             * Erase the character on a typical terminal.
             */
            printf("\b \b");
        }

        return;
    }

    /*
     * Ignore non-printable control characters.
     */
    if (!isprint((unsigned char)character)) {
        return;
    }

    if (g_command_length >=
        (CONSOLE_COMMAND_LENGTH - 1u)) {
        g_console_overflow_count++;

        g_command_length = 0u;
        g_command_buffer[0] = '\0';

        printf("\r\nERROR: command too long\r\n> ");
        return;
    }

    g_command_buffer[g_command_length] =
        character;

    g_command_length++;

    /*
     * Local echo. Remove this if the USB-UART terminal already echoes input.
     */
    (void)MXC_UART_WriteCharacter(
        CONSOLE_UART1,
        character);
}

void vibration_console_service(void)
{
    int available;

    /*
     * Process only bytes already in the RX FIFO.
     * Never wait here for another character.
     */
    available =
        MXC_UART_GetRXFIFOAvailable(
            CONSOLE_UART1);

    while (available > 0) {
        int character;

        character =
            MXC_UART_ReadCharacterRaw(
                CONSOLE_UART1);

        if (character >= 0) {
            console_accept_character(
                (char)character);
        }

        available =
            MXC_UART_GetRXFIFOAvailable(
                CONSOLE_UART1);
    }
}

vibration_channel_t vibration_console_get_channel(void)
{
    return g_selected_channel;
}

float vibration_console_get_input_range_g(void)
{
    return g_pwm_input_range_g;
}

bool vibration_console_dc_removal_enabled(void)
{
    return g_dc_removal_enabled;
}

static float remove_dc(float input,
                       float *dc_estimate)
{
    *dc_estimate +=
    		DEFAULT_DC_ALPHA *
        (input - *dc_estimate);

    return input - *dc_estimate;
}

float vibration_console_select_signal(float x_g,
                                      float y_g,
                                      float z_g)
{
	vibration_channel_t channel;
    float x_corrected;
    float y_corrected;
    float z_corrected;
    float selected;

    /*
     * Save the latest calibrated sample for the "zero" command.
     */
    g_latest_x_g = x_g;
    g_latest_y_g = y_g;
    g_latest_z_g = z_g;

    x_corrected = x_g - g_zero_x_g;
    y_corrected = y_g - g_zero_y_g;
    z_corrected = z_g - g_zero_z_g;

    if (x_corrected  > g_max_x_g){
    	g_max_x_g = x_corrected;
    }
    if (y_corrected  > g_max_y_g){
    	g_max_y_g = y_corrected;
    }
    if (z_corrected  > g_max_z_g){
    	g_max_z_g = z_corrected;
    }
    channel = g_selected_channel;

    switch (channel) {
    case VIB_PWM_CHANNEL_X:
        if (g_dc_removal_enabled) {
            selected =
                remove_dc(x_corrected,
                          &g_dc_x_g);
        } else {
            selected = x_corrected;
        }
        break;

    case VIB_PWM_CHANNEL_Y:
        if (g_dc_removal_enabled) {
            selected =
                remove_dc(y_corrected,
                          &g_dc_y_g);
        } else {
            selected = y_corrected;
        }
        break;

    case VIB_PWM_CHANNEL_Z:
        if (g_dc_removal_enabled) {
            selected =
                remove_dc(z_corrected,
                          &g_dc_z_g);
        } else {
            selected = z_corrected;
        }
        break;

    case VIB_PWM_CHANNEL_MAGNITUDE:
        selected =
            sqrtf((x_corrected * x_corrected) +
                  (y_corrected * y_corrected) +
                  (z_corrected * z_corrected));

        /*
         * Magnitude is always positive and includes approximately 1 g of
         * gravity when stationary. DC removal centers it around zero.
         */
        if (g_dc_removal_enabled) {
            selected =
                remove_dc(selected,
                          &g_dc_magnitude_g);
        }
        break;

    case VIB_PWM_CHANNEL_OFF:
    default:
        selected = 0.0f;
        break;
    }

    return selected;
}
