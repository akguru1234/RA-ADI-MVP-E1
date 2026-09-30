#include "adi_encoder_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mxc_errors.h"
#include "adi_encoder_spi1_max32672.h"

#define ADI_ENCODER_DEFAULT_SPI_HZ UINT32_C(1000000)
#define ADI_ENCODER_MAX_POSITION   UINT32_C(0x001FFFFF)

static bool buffer_has_constant_value(const uint8_t *buffer,
                                      size_t length,
                                      uint8_t value)
{
    size_t index;

    for (index = 0u; index < length; index++) {
        if (buffer[index] != value) {
            return false;
        }
    }

    return true;
}

static void copy_frame(uint8_t destination[MAULIN_FRAME_SIZE],
                       const uint8_t source[MAULIN_FRAME_SIZE])
{
    memcpy(destination, source, MAULIN_FRAME_SIZE);
}

const char *adi_encoder_test_status_string(adi_encoder_test_status_t status)
{
    switch (status) {
    case ADI_ENCODER_TEST_PASS: return "PASS";
    case ADI_ENCODER_TEST_BAD_ARGUMENT: return "BAD ARGUMENT";
    case ADI_ENCODER_TEST_SPI_INIT_FAILED: return "SPI INITIALIZATION FAILED";
    case ADI_ENCODER_TEST_SPI_TRANSFER_FAILED: return "SPI TRANSFER FAILED";
    case ADI_ENCODER_TEST_LOOPBACK_MISMATCH: return "LOOPBACK DATA MISMATCH";
    case ADI_ENCODER_TEST_NO_RESPONSE: return "NO ENCODER RESPONSE";
    case ADI_ENCODER_TEST_CRC_FAILED: return "CRC CHECK FAILED";
    case ADI_ENCODER_TEST_ALARM_ACTIVE: return "ENCODER ALARM ACTIVE";
    case ADI_ENCODER_TEST_INVALID_POSITION: return "INVALID POSITION";
    default: return "UNKNOWN STATUS";
    }
}

adi_encoder_test_status_t adi_encoder_spi1_loopback_test(
    uint32_t spi_frequency_hz,
    uint32_t iteration_count,
    adi_encoder_loopback_result_t *result)
{
    static const uint8_t pattern[MAULIN_FRAME_SIZE] =
        { 0x00u, 0xFFu, 0x55u, 0xAAu, 0x12u, 0x34u, 0xA5u, 0x5Au };
    uint8_t transmit_frame[MAULIN_FRAME_SIZE];
    uint8_t receive_frame[MAULIN_FRAME_SIZE];
    uint32_t iteration;
    uint32_t index;
    int status;

    if ((result == NULL) || (iteration_count == 0u)) {
        return ADI_ENCODER_TEST_BAD_ARGUMENT;
    }

    memset(result, 0, sizeof(*result));

    if (spi_frequency_hz == 0u) {
        spi_frequency_hz = ADI_ENCODER_DEFAULT_SPI_HZ;
    }

    status = adi_encoder_spi1_init(spi_frequency_hz);
    if (status != E_NO_ERROR) {
        result->status = ADI_ENCODER_TEST_SPI_INIT_FAILED;
        return result->status;
    }

    for (iteration = 0u; iteration < iteration_count; iteration++) {
        for (index = 0u; index < MAULIN_FRAME_SIZE; index++) {
            transmit_frame[index] = pattern[index] ^ (uint8_t)iteration;
        }

        memset(receive_frame, 0, sizeof(receive_frame));
        status = adi_encoder_spi1_transfer(transmit_frame,
                                           receive_frame,
                                           sizeof(transmit_frame),
                                           NULL);
        result->transactions++;

        if (status != E_NO_ERROR) {
            result->transfer_errors++;
            copy_frame(result->expected, transmit_frame);
            copy_frame(result->received, receive_frame);
            result->status = ADI_ENCODER_TEST_SPI_TRANSFER_FAILED;
            adi_encoder_spi1_shutdown();
            return result->status;
        }

        if (memcmp(transmit_frame, receive_frame, sizeof(transmit_frame)) != 0) {
            result->data_mismatches++;
            copy_frame(result->expected, transmit_frame);
            copy_frame(result->received, receive_frame);
            result->status = ADI_ENCODER_TEST_LOOPBACK_MISMATCH;
            adi_encoder_spi1_shutdown();
            return result->status;
        }
    }

    result->status = ADI_ENCODER_TEST_PASS;
    adi_encoder_spi1_shutdown();
    return result->status;
}

static adi_encoder_test_status_t read_encoder_register(
    uint8_t register_address,
    uint16_t *register_value,
    maulin_response_t *response,
    uint8_t raw_response[MAULIN_FRAME_SIZE],
    adi_encoder_communication_result_t *result)
{
    uint8_t transmit_frame[MAULIN_FRAME_SIZE];
    uint8_t receive_frame[MAULIN_FRAME_SIZE];
    int status;

    maulin_build_read(transmit_frame, register_address);
    memset(receive_frame, 0, sizeof(receive_frame));

    status = adi_encoder_spi1_transfer(transmit_frame,
                                       receive_frame,
                                       sizeof(transmit_frame),
                                       NULL);
    result->transactions++;
    if (status != E_NO_ERROR) {
        result->transfer_errors++;
        return ADI_ENCODER_TEST_SPI_TRANSFER_FAILED;
    }

    maulin_build_nop(transmit_frame);
    memset(receive_frame, 0, sizeof(receive_frame));

    status = adi_encoder_spi1_transfer(transmit_frame,
                                       receive_frame,
                                       sizeof(transmit_frame),
                                       NULL);
    result->transactions++;
    copy_frame(raw_response, receive_frame);

    if (status != E_NO_ERROR) {
        result->transfer_errors++;
        return ADI_ENCODER_TEST_SPI_TRANSFER_FAILED;
    }

    if (buffer_has_constant_value(receive_frame, sizeof(receive_frame), 0x00u) ||
        buffer_has_constant_value(receive_frame, sizeof(receive_frame), 0xFFu)) {
        return ADI_ENCODER_TEST_NO_RESPONSE;
    }

    maulin_parse_response(receive_frame, response);
    if (!response->crc_valid) {
        result->crc_errors++;
        return ADI_ENCODER_TEST_CRC_FAILED;
    }

    *register_value = response->register_data;
    return ADI_ENCODER_TEST_PASS;
}

static adi_encoder_test_status_t read_encoder_position(
    maulin_response_t *response,
    uint8_t raw_response[MAULIN_FRAME_SIZE],
    adi_encoder_communication_result_t *result)
{
    uint8_t transmit_frame[MAULIN_FRAME_SIZE];
    uint8_t receive_frame[MAULIN_FRAME_SIZE];
    int status;

    maulin_build_nop(transmit_frame);
    memset(receive_frame, 0, sizeof(receive_frame));

    status = adi_encoder_spi1_transfer(transmit_frame,
                                       receive_frame,
                                       sizeof(transmit_frame),
                                       NULL);
    result->transactions++;
    copy_frame(raw_response, receive_frame);

    if (status != E_NO_ERROR) {
        result->transfer_errors++;
        return ADI_ENCODER_TEST_SPI_TRANSFER_FAILED;
    }

    if (buffer_has_constant_value(receive_frame, sizeof(receive_frame), 0x00u) ||
        buffer_has_constant_value(receive_frame, sizeof(receive_frame), 0xFFu)) {
        return ADI_ENCODER_TEST_NO_RESPONSE;
    }

    maulin_parse_response(receive_frame, response);
    if (!response->crc_valid) {
        result->crc_errors++;
        return ADI_ENCODER_TEST_CRC_FAILED;
    }

    if (response->single_turn_position > ADI_ENCODER_MAX_POSITION) {
        return ADI_ENCODER_TEST_INVALID_POSITION;
    }

    return ADI_ENCODER_TEST_PASS;
}

adi_encoder_test_status_t adi_encoder_communication_test(
    uint32_t spi_frequency_hz,
    uint8_t test_register,
    uint32_t position_samples,
    adi_encoder_communication_result_t *result)
{
    adi_encoder_test_status_t test_status;
    maulin_response_t response;
    uint16_t register_value;
    uint32_t sample_index;
    int status;

    if ((result == NULL) || (position_samples == 0u)) {
        return ADI_ENCODER_TEST_BAD_ARGUMENT;
    }

    memset(result, 0, sizeof(*result));

    if (spi_frequency_hz == 0u) {
        spi_frequency_hz = ADI_ENCODER_DEFAULT_SPI_HZ;
    }

    status = adi_encoder_spi1_init(spi_frequency_hz);
    if (status != E_NO_ERROR) {
        result->status = ADI_ENCODER_TEST_SPI_INIT_FAILED;
        return result->status;
    }

    test_status = read_encoder_register(test_register,
                                        &register_value,
                                        &response,
                                        result->raw_response,
                                        result);
    if (test_status != ADI_ENCODER_TEST_PASS) {
        result->status = test_status;
        adi_encoder_spi1_shutdown();
        return result->status;
    }

    result->register_data = register_value;
    result->alarm = response.alarm;

    for (sample_index = 0u; sample_index < position_samples; sample_index++) {
        test_status = read_encoder_position(&response,
                                            result->raw_response,
                                            result);
        if (test_status != ADI_ENCODER_TEST_PASS) {
            result->status = test_status;
            adi_encoder_spi1_shutdown();
            return result->status;
        }

        if (sample_index == 0u) {
            result->first_position = response.single_turn_position;
        }

        result->last_position = response.single_turn_position;
        result->turn_count = response.turn_count;
        result->alarm = response.alarm;
    }

    result->status = (result->alarm != 0u) ?
                     ADI_ENCODER_TEST_ALARM_ACTIVE :
                     ADI_ENCODER_TEST_PASS;

    adi_encoder_spi1_shutdown();
    return result->status;
}
