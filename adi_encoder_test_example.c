#include <stdio.h>

#include "adi_encoder_test.h"

#define TEST_SPI_FREQUENCY_HZ UINT32_C(1000000)
#define TEST_REGISTER_ADDRESS 0x24u /* Verify against the authoritative register map. */

static void print_frame(const uint8_t frame[MAULIN_FRAME_SIZE])
{
    uint32_t index;

    for (index = 0u; index < MAULIN_FRAME_SIZE; index++) {
        printf(" %02X", frame[index]);
    }
}

void run_spi1_loopback_test(void)
{
    adi_encoder_loopback_result_t result;
    adi_encoder_test_status_t status;

    status = adi_encoder_spi1_loopback_test(TEST_SPI_FREQUENCY_HZ, 256u, &result);

    printf("\r\nSPI1 loopback test: %s\r\n",
           adi_encoder_test_status_string(status));
    printf("Transactions: %lu\r\n", (unsigned long)result.transactions);
    printf("Transfer errors: %lu\r\n", (unsigned long)result.transfer_errors);
    printf("Data mismatches: %lu\r\n", (unsigned long)result.data_mismatches);

    if (status == ADI_ENCODER_TEST_LOOPBACK_MISMATCH) {
        printf("Expected:");
        print_frame(result.expected);
        printf("\r\nReceived:");
        print_frame(result.received);
        printf("\r\n");
    }
}

void run_encoder_communication_test(void)
{
    adi_encoder_communication_result_t result;
    adi_encoder_test_status_t status;

    status = adi_encoder_communication_test(TEST_SPI_FREQUENCY_HZ,
                                            TEST_REGISTER_ADDRESS,
                                            32u,
                                            &result);

    printf("\r\nEncoder communication test: %s\r\n",
           adi_encoder_test_status_string(status));
    printf("Transactions: %lu\r\n", (unsigned long)result.transactions);
    printf("Transfer errors: %lu\r\n", (unsigned long)result.transfer_errors);
    printf("CRC errors: %lu\r\n", (unsigned long)result.crc_errors);
    printf("Register data: 0x%04X\r\n", result.register_data);
    printf("First position: %lu\r\n", (unsigned long)result.first_position);
    printf("Last position: %lu\r\n", (unsigned long)result.last_position);
    printf("Turn count: %d\r\n", result.turn_count);
    printf("Alarm: %u\r\n", result.alarm);
    printf("Last response:");
    print_frame(result.raw_response);
    printf("\r\n");
}
