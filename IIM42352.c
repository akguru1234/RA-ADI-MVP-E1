/*
 * IIM42352.c
 *
 *  Created on: Sep 29, 2026
 *      Author: akguru
 */
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

#include "IIM42352.h"
#include "vibration_console.h"
#include "IIM42352_Spi_dma_pwm_out.h"
#include "IIM42352-Vibration-FFT.h"
#include "latency_measurement.h"

//locals
static uint8_t discard[6];

/* --------------------------------------------------------------------------
 * low level drivers to read from IIM42352 over SPI0 Interface
 * -------------------------------------------------------------------------- */
int iim_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = reg | IIM_SPI_WRITE;
    tx[1] = value;

    return iim_spi_transaction(tx, NULL, sizeof(tx), 0);
}

int iim_read_regs(uint8_t start_reg, uint8_t *data, uint32_t length)
{
    uint8_t command = start_reg | IIM_SPI_READ;

    /*
     * MSDK performs the command phase followed by the receive phase while
     * keeping chip select asserted for the complete request.
     */
    return iim_spi_transaction(&command, data, 2, length);
}
int iim_read_multi_regs(uint8_t start_reg, uint8_t *data, uint32_t length)
{

    uint8_t command = start_reg | IIM_SPI_READ;
    return iim_spi_transaction(&command, data, length, length);

}
int iim_read_multi_reg(uint8_t reg, uint8_t *value, uint8_t num)
{
	 uint8_t rd_reg_val[num+1];
	 int err;

	 err = iim_read_multi_regs(reg, rd_reg_val, num+1);
	 if (err == E_NO_ERROR) {
		 for (uint8_t i=0; i<num; i++){
			 *value++ = rd_reg_val[i+1];
		 }

	 }
	return err;
}

int iim_read_reg(uint8_t reg, uint8_t *value)
{

	 int err;
	 uint8_t rd_reg_val[2];

	 err = iim_read_regs(reg, rd_reg_val, 2);
	 if (err == E_NO_ERROR) {
		 *value = rd_reg_val[1];
	 }

	return err;
}

int iim_select_bank(uint8_t bank)
{
    return iim_write_reg(IIM_REG_BANK_SEL, bank & 0x07u);
}

int iim_read_FIFO_record(uint8_t *data, uint32_t num_bytes_per_record) // @suppress("Type cannot be resolved")
{
//reads from IIM_42352 FIFO
// FIFO is stores 8 bytes per record
// num_records indicates number of records to read
	int err;
	uint8_t rx_data[num_bytes_per_record+1];
	uint8_t tx_data[num_bytes_per_record+1];

	tx_data[0]=IIM_REG_FIFO_DATA | IIM_SPI_READ;
	err = iim_spi_transaction(tx_data, rx_data, num_bytes_per_record+1,num_bytes_per_record+1);
	if (err == E_NO_ERROR) {
		for (uint8_t i=0; i<num_bytes_per_record; i++){
			*data++ = rx_data[i+1];
		}

	}
	return err;

}
int iim42352_data_ready(bool *ready)
{
    int err;
    uint8_t status;

    //only one active spi read at a time
    if (g_spi_dma_busy) {
    	g_dma_overrun_count++;
    	return E_BUSY;
    }
    err = iim_read_reg(IIM_REG_INT_STATUS, &status);
    if (err != E_NO_ERROR) {
        *ready = false;
        return err;
    }

    *ready = ((status & IIM_INT_STATUS_DRDY) != 0u);

    return E_NO_ERROR;
}

/* --------------------------------------------------------------------------
 * Vibration Sensor initialization
 * -------------------------------------------------------------------------- */

int iim42352_init(void)
{
    int err;
    uint8_t who_am_i = 0u;

    /* Force bank 0 before accessing the normal register map. */
    err = iim_select_bank(0);
    if (err != E_NO_ERROR) {
        return err;
    }

    /* Software reset. */
    err = iim_write_reg(IIM_REG_DEVICE_CONFIG, IIM_DEVICE_SOFT_RESET);
    if (err != E_NO_ERROR) {
        return err;
    }

    MXC_Delay(MXC_DELAY_MSEC(2));

    /* Reset returns the selected register bank to bank 0. */
    err = iim_read_reg(IIM_REG_WHO_AM_I, &who_am_i);
    if (err != E_NO_ERROR) {
        return err;
    }

    //printf("IIM-42352 WHO_AM_I = 0x%02X\n", who_am_i);

    if ((IIM42352_WHO_AM_I_VALUE != 0u) &&
        (who_am_i != IIM42352_WHO_AM_I_VALUE)) {
        return E_BAD_STATE;
    }

    uint8_t data[PACKET_SIZE];
    err = iim_read_FIFO_record(data, sizeof(data));
    if (err != E_NO_ERROR) return err;

    /*
     * Keep the accelerometer off while its ODR, range, and filters
     * are programmed.
     */
    err = iim_write_reg(IIM_REG_PWR_MGMT0, IIM_ACCEL_MODE_OFF);
    if (err != E_NO_ERROR) {
        return err;
    }

    /*
     * ±16 g full scale and 8 kHz ODR:
     *
     * ACCEL_FS_SEL = 000b
     * ACCEL_ODR    = 0011b
     */
    err = iim_write_reg(IIM_REG_ACCEL_CONFIG0,
                        IIM_ACCEL_FS_16G | IIM_ACCEL_ODR_8KHZ);
    if (err != E_NO_ERROR) {
        return err;
    }

    /*
     * UI low-pass filter bandwidth = ODR/2.
     * Use a narrower setting here if the application bandwidth is lower.
     */
    err = iim_write_reg(IIM_REG_ACCEL_FILT_CONFIG,
                        IIM_ACCEL_UI_BW_ODR_DIV_2);
    if (err != E_NO_ERROR) {
        return err;
    }

    err = iim_write_reg(IIM_REG_ACCEL_CONFIG1,
                        IIM_ACCEL_FILTER_ORDER);
    if (err != E_NO_ERROR) {
        return err;
    }

    /*
     * Optional interrupt configuration:
     * INT1 push-pull, active high, pulsed.
     *
     * This write may be omitted when polling INT_STATUS only.
     * ignore for now by polling INT_STATUS
     */
#if DRDY_INTERRUPT_WIRED
    err = iim_write_reg(IIM_REG_INT_CONFIG, 0x03u);
    if (err != E_NO_ERROR) {
        return err;
    }

    /*
     * Route UI data-ready to INT1. This is useful when the INT1 pin is
     * connected to a MAX32672 GPIO, but INT_STATUS polling also works.
     */
    err = iim_write_reg(IIM_REG_INT_SOURCE0,
                        IIM_INT_SOURCE0_UI_DRDY_EN);
    if (err != E_NO_ERROR) {
        return err;
    }
#endif
    /* for E0 comms board evaluation use polling of INT_STATUS register to see
     *
     * if FIFO reached water mark threshold
     */
    err = iim_write_reg(IIM_REG_FIFO_CONFIG1, IIM_FIFO_CONFIG1_REG_VAL);
    if (err != E_NO_ERROR) {
    	return err;
    }
    err = iim_write_reg(IIM_REG_FIFO_CONFIG2, IIM_FIFO_CONFIG2_REG_VAL);
    if (err != E_NO_ERROR) {
    	return err;
    }
    err = iim_write_reg(IIM_REG_FIFO_CONFIG3, IIM_FIFO_CONFIG3_REG_VAL);
    if (err != E_NO_ERROR) {
    	return err;
    }
    err = iim_write_reg(IIM_REG_FIFO_CONFIG, IIM_FIFO_CONFIG_REG_VAL);
    if (err != E_NO_ERROR) {
    	return err;
    }
    uint8_t read_reg_val;
    err = iim_read_reg (IIM_REG_FIFO_CONFIG,&read_reg_val );
    if (err != E_NO_ERROR) {
    	return err;
    }

    /*
     * Enable low-noise mode. The datasheet requires no further register
     * writes for at least 200 microseconds after the OFF-to-ON transition.
     */
    err = iim_write_reg(IIM_REG_PWR_MGMT0, IIM_ACCEL_MODE_LN);
    if (err != E_NO_ERROR) {
        return err;
    }

    MXC_Delay(MXC_DELAY_USEC(1000));

    /*
     * Discard the first sample following startup because the first output
     * can be invalid while the signal path settles.
     */

    err = iim_read_multi_reg(IIM_REG_ACCEL_DATA_X1,
    		discard,
			sizeof(discard));
    if (err != E_NO_ERROR) return err;

    return E_NO_ERROR;
}
