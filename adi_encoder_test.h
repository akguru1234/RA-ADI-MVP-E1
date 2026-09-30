#ifndef ADI_ENCODER_TEST_H
#define ADI_ENCODER_TEST_H

#include <stdint.h>

#include "maulin_proto.h"

typedef enum
{
    ADI_ENCODER_TEST_PASS = 0,
    ADI_ENCODER_TEST_BAD_ARGUMENT,
    ADI_ENCODER_TEST_SPI_INIT_FAILED,
    ADI_ENCODER_TEST_SPI_TRANSFER_FAILED,
    ADI_ENCODER_TEST_LOOPBACK_MISMATCH,
    ADI_ENCODER_TEST_NO_RESPONSE,
    ADI_ENCODER_TEST_CRC_FAILED,
    ADI_ENCODER_TEST_ALARM_ACTIVE,
    ADI_ENCODER_TEST_INVALID_POSITION
} adi_encoder_test_status_t;

typedef struct
{
    adi_encoder_test_status_t status;
    uint32_t transactions;
    uint32_t transfer_errors;
    uint32_t data_mismatches;
    uint8_t expected[MAULIN_FRAME_SIZE];
    uint8_t received[MAULIN_FRAME_SIZE];
} adi_encoder_loopback_result_t;

typedef struct
{
    adi_encoder_test_status_t status;
    uint32_t transactions;
    uint32_t transfer_errors;
    uint32_t crc_errors;
    uint32_t first_position;
    uint32_t last_position;
    int16_t turn_count;
    uint16_t register_data;
    uint8_t alarm;
    uint8_t raw_response[MAULIN_FRAME_SIZE];
} adi_encoder_communication_result_t;

adi_encoder_test_status_t adi_encoder_spi1_loopback_test(
    uint32_t spi_frequency_hz,
    uint32_t iteration_count,
    adi_encoder_loopback_result_t *result);

adi_encoder_test_status_t adi_encoder_communication_test(
    uint32_t spi_frequency_hz,
    uint8_t test_register,
    uint32_t position_samples,
    adi_encoder_communication_result_t *result);

extern void run_encoder_communication_test(void);
const char *adi_encoder_test_status_string(adi_encoder_test_status_t status);

#endif /* ADI_ENCODER_TEST_H */
