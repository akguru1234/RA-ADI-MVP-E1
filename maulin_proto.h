#ifndef MAULIN_PROTO_H
#define MAULIN_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#define MAULIN_FRAME_SIZE 8u

typedef struct
{
    uint16_t register_data;
    int16_t turn_count;
    uint32_t single_turn_position;
    uint8_t alarm;
    uint8_t received_crc;
    bool crc_valid;
} maulin_response_t;

uint8_t maulin_crc8(const uint8_t *data, uint8_t length);
void maulin_build_nop(uint8_t frame[MAULIN_FRAME_SIZE]);
void maulin_build_read(uint8_t frame[MAULIN_FRAME_SIZE], uint8_t address);
void maulin_build_write(uint8_t frame[MAULIN_FRAME_SIZE], uint8_t address, uint16_t data);
void maulin_parse_response(const uint8_t frame[MAULIN_FRAME_SIZE], maulin_response_t *response);

#endif /* MAULIN_PROTO_H */
