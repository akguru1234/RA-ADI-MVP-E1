#include "maulin_proto.h"

#include <stddef.h>
#include <string.h>

#define MAULIN_CRC_INITIAL_VALUE 0xA5u
#define MAULIN_CRC_POLYNOMIAL    0x07u

uint8_t maulin_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = MAULIN_CRC_INITIAL_VALUE;
    uint8_t byte_index;
    uint8_t bit_index;

    if (data == NULL) {
        return 0u;
    }

    for (byte_index = 0u; byte_index < length; byte_index++) {
        crc ^= data[byte_index];

        for (bit_index = 0u; bit_index < 8u; bit_index++) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((crc << 1) ^ MAULIN_CRC_POLYNOMIAL);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void maulin_build_nop(uint8_t frame[MAULIN_FRAME_SIZE])
{
    if (frame == NULL) {
        return;
    }

    memset(frame, 0, MAULIN_FRAME_SIZE);
    frame[0] = 0x80u;
    frame[7] = maulin_crc8(frame, 7u);
}

void maulin_build_read(uint8_t frame[MAULIN_FRAME_SIZE], uint8_t address)
{
    if (frame == NULL) {
        return;
    }

    memset(frame, 0, MAULIN_FRAME_SIZE);
    frame[0] = (uint8_t)(0x80u | ((address >> 2) & 0x3Fu));
    frame[1] = (uint8_t)((address & 0x03u) << 6);
    frame[7] = maulin_crc8(frame, 7u);
}

void maulin_build_write(uint8_t frame[MAULIN_FRAME_SIZE], uint8_t address, uint16_t data)
{
    if (frame == NULL) {
        return;
    }

    memset(frame, 0, MAULIN_FRAME_SIZE);
    frame[0] = (uint8_t)((address >> 2) & 0x3Fu);
    frame[1] = (uint8_t)((address & 0x03u) << 6);
    frame[5] = (uint8_t)(data >> 8);
    frame[6] = (uint8_t)data;
    frame[7] = maulin_crc8(frame, 7u);
}

void maulin_parse_response(const uint8_t frame[MAULIN_FRAME_SIZE], maulin_response_t *response)
{
    uint16_t turn_count_raw;

    if ((frame == NULL) || (response == NULL)) {
        return;
    }

    response->alarm = (uint8_t)((frame[0] >> 5) & 0x01u);
    response->register_data =
        (uint16_t)(((uint16_t)(frame[0] & 0x1Fu) << 11) |
                   ((uint16_t)frame[1] << 3) |
                   ((uint16_t)(frame[2] >> 5) & 0x07u));

    turn_count_raw =
        (uint16_t)(((uint16_t)(frame[2] & 0x1Fu) << 11) |
                   ((uint16_t)frame[3] << 3) |
                   ((uint16_t)(frame[4] >> 5) & 0x07u));

    response->turn_count = (int16_t)turn_count_raw;
    response->single_turn_position =
        ((uint32_t)(frame[4] & 0x1Fu) << 16) |
        ((uint32_t)frame[5] << 8) |
        (uint32_t)frame[6];

    response->received_crc = frame[7];
    response->crc_valid = (frame[7] == maulin_crc8(frame, 7u));
}
