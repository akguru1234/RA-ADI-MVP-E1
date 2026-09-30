/*
 * IIM42352.h
 *
 *  Created on: Sep 29, 2026
 *      Author: akguru
 */

#ifndef IIM42352_H_
#define IIM42352_H_

//IIM42352 Packet 1 has predefined 8 byte length
#define PACKET_SIZE        			8u
/* --------------------------------------------------------------------------
 * IIM-42352 bank 0 registers
 * -------------------------------------------------------------------------- */

#define IIM_REG_DEVICE_CONFIG       0x11u
#define IIM_REG_FIFO_CONFIG      	0x16u
#define IIM_REG_TEMP_DATA1        	0x1Du
#define IIM_REG_TEMP_DATA0	        0x1Eu
#define IIM_REG_ACCEL_DATA_X1       0x1Fu
#define IIM_REG_ACCEL_DATA_X0       0x20u
#define IIM_REG_ACCEL_DATA_Y1       0x21u
#define IIM_REG_ACCEL_DATA_Y0       0x22u
#define IIM_REG_ACCEL_DATA_Z1       0x23u
#define IIM_REG_ACCEL_DATA_Z0       0x24u

#define IIM_REG_INT_STATUS          0x2Du
#define IIM_REG_FIFO_DATA      		0x30u
#define IIM_REG_SIGNAL_PATH_RESET   0x4Bu
#define IIM_REG_INTF_CONFIG0        0x4Cu
#define IIM_REG_INTF_CONFIG1        0x4Du
#define IIM_REG_PWR_MGMT0           0x4Eu
#define IIM_REG_ACCEL_CONFIG0       0x50u
#define IIM_REG_ACCEL_FILT_CONFIG   0x52u
#define IIM_REG_ACCEL_CONFIG1       0x53u
#define IIM_REG_FIFO_CONFIG1        0x5Fu
#define IIM_REG_FIFO_CONFIG2        0x60u
#define IIM_REG_FIFO_CONFIG3        0x61u
#define IIM_REG_INT_CONFIG          0x14u
#define IIM_REG_INT_CONFIG0         0x63u
#define IIM_REG_INT_CONFIG1         0x64u
#define IIM_REG_INT_SOURCE0         0x65u
#define IIM_REG_WHO_AM_I            0x75u
#define IIM_REG_BANK_SEL            0x76u

#define IIM_ACCEL_FILTER_ORDER      ((2u << 3) | (2u << 1))
/* SIGNAL_PATH_RESET */
#define IIM_ABORT_AND_RESET         0x08u

/* PWR_MGMT0 */
#define IIM_ACCEL_MODE_OFF          0x00u
#define IIM_ACCEL_MODE_LN           0x13u
/* SPI protocol */
#define IIM_SPI_READ                0x80u
#define IIM_SPI_WRITE               0x00u

/* DEVICE_CONFIG */
#define IIM_DEVICE_SOFT_RESET       0x01u

/* FIFO CONFIG */
#define IIM_FIFO_CONFIG_REG_VAL		0x00u //Disable FIFO
#define IIM_FIFO_CONFIG1_REG_VAL	0x00u //Disable FIFO
#define IIM_FIFO_CONFIG2_REG_VAL	0x00u //Disable FIFO
#define IIM_FIFO_CONFIG3_REG_VAL	0x00u //Disable FIFO


/* SIGNAL_PATH_RESET */
#define IIM_ABORT_AND_RESET         0x08u

/* PWR_MGMT0 */
#define IIM_ACCEL_MODE_OFF          0x00u
#define IIM_ACCEL_MODE_LN           0x13u
/* ACCEL_CONFIG0, bits 7:5 select full scale */
#define IIM_ACCEL_FS_16G            (0u << 5)
#define IIM_ACCEL_FS_8G             (1u << 5)
#define IIM_ACCEL_FS_4G             (2u << 5)
#define IIM_ACCEL_FS_2G             (3u << 5)

/* ACCEL_CONFIG0, bits 3:0 select ODR */
#define IIM_ACCEL_ODR_8KHZ          0x03u

/*
 * ACCEL_FILT_CONFIG:
 * bits 7:4 = 0 selects UI filter bandwidth of ODR/2 in LN mode.
 * At 8 kHz ODR this corresponds to nominal 4 kHz UI bandwidth.
 */
#define IIM_ACCEL_UI_BW_ODR_DIV_2   0x00u

/*
 * ACCEL_CONFIG1:
 * bits 4:3 = 10b selects third-order UI filter.
 * bits 2:1 = 10b selects the documented DEC2_M2 setting.
 */
#define IIM_ACCEL_FILTER_ORDER      ((2u << 3) | (2u << 1))

/*
 * INT_STATUS data-ready bit.
 * Verify the bit definition against the exact IIM-42352 datasheet revision.
 */
#define IIM_INT_STATUS_DRDY         0x08u //data is ready
#define IIM_INT_STATUS_FIFO_THS     0x04u //FIFO is at Threshold
#define IIM_INT_STATUS_FIFO_FULL    0x02u //FIFO is full
/*
 * INT_SOURCE0 UI data-ready routing bit.
 * Used only if the physical INT pin is connected and enabled.
 */
#define IIM_INT_SOURCE0_UI_DRDY_EN  0x08u	//not used in E0 board as interrupt is not wired

/*
 * Replace with the value specified for your device revision.
 * Setting this to zero disables the strict value comparison.
 */
#define IIM42352_WHO_AM_I_VALUE     0x6Du
 /***** Functions *****/
/* --------------------------------------------------------------------------
 * iim-42352 interface handlers
 * -------------------------------------------------------------------------- */

extern int iim_read_multi_reg(uint8_t reg, uint8_t *value, uint8_t num);
extern int iim_read_multi_regs(uint8_t start_reg, uint8_t *data, uint32_t length);
extern int iim_write_reg(uint8_t reg, uint8_t value);
extern int iim_read_regs(uint8_t start_reg, uint8_t *data, uint32_t length);
extern int iim_read_reg(uint8_t reg, uint8_t *value);
extern int iim_read_FIFO_record(uint8_t *data, uint32_t num_bytes_per_record);
extern int iim_select_bank(uint8_t bank);

extern int iim42352_init(void);
extern int iim42352_data_ready(bool *ready);

#endif /* IIM42352_H_ */
