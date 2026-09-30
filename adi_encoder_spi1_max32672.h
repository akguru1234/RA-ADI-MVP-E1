#ifndef ADI_ENCODER_SPI1_MAX32672_H
#define ADI_ENCODER_SPI1_MAX32672_H

#include <stddef.h>
#include <stdint.h>

int adi_encoder_spi1_init(uint32_t frequency_hz);
void adi_encoder_spi1_shutdown(void);
int adi_encoder_spi1_transfer(const uint8_t *transmit_data,
                              uint8_t *receive_data,
                              size_t length,
                              void *context);
uint32_t adi_encoder_spi1_get_frequency_hz(void);

#endif /* ADI_ENCODER_SPI1_MAX32672_H */
