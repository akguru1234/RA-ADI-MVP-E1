#include "adi_encoder_spi1_max32672.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mxc_device.h"
#include "mxc_errors.h"
#include "spi.h"

/* MAX32672 SPI1 pin group used by the P4 encoder interface:
 * P0.14 = MISO, P0.15 = MOSI, P0.16 = SCK, P0.17 = SS0.
 */
#define ADI_ENCODER_SPI              MXC_SPI1
#define ADI_ENCODER_SPI_SS_INDEX     0u
#define ADI_ENCODER_SPI_NUM_SLAVES   1
#define ADI_ENCODER_SPI_SS_POLARITY  0
#define ADI_ENCODER_SPI_DEFAULT_HZ   UINT32_C(10000000)

static volatile bool g_spi1_transaction_active;
static uint32_t g_spi1_frequency_hz;

int adi_encoder_spi1_init(uint32_t frequency_hz)
{
    int status;

    if (frequency_hz == 0u) {
        frequency_hz = ADI_ENCODER_SPI_DEFAULT_HZ;
    }

    g_spi1_transaction_active = false;
    g_spi1_frequency_hz = 0u;
    MXC_SPI_Shutdown(ADI_ENCODER_SPI);

    status = MXC_SPI_Init(ADI_ENCODER_SPI,
                          1,
                          0,
                          ADI_ENCODER_SPI_NUM_SLAVES,
                          ADI_ENCODER_SPI_SS_POLARITY,
                          frequency_hz);
    if (status != E_NO_ERROR) {
        return status;
    }

    status = MXC_SPI_SetMode(ADI_ENCODER_SPI, SPI_MODE_0);
    if (status != E_NO_ERROR) {
        MXC_SPI_Shutdown(ADI_ENCODER_SPI);
        return status;
    }

    status = MXC_SPI_SetWidth(ADI_ENCODER_SPI, SPI_WIDTH_STANDARD);
    if (status != E_NO_ERROR) {
        MXC_SPI_Shutdown(ADI_ENCODER_SPI);
        return status;
    }

    status = MXC_SPI_SetDataSize(ADI_ENCODER_SPI, 8);
    if (status != E_NO_ERROR) {
        MXC_SPI_Shutdown(ADI_ENCODER_SPI);
        return status;
    }

    status = MXC_SPI_SetFrequency(ADI_ENCODER_SPI, frequency_hz);
    if (status != E_NO_ERROR) {
        MXC_SPI_Shutdown(ADI_ENCODER_SPI);
        return status;
    }

    g_spi1_frequency_hz = frequency_hz;
    return E_NO_ERROR;
}

void adi_encoder_spi1_shutdown(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_spi1_transaction_active = false;
    g_spi1_frequency_hz = 0u;
    __DMB();

    if (primask == 0u) {
        __enable_irq();
    }

    MXC_SPI_Shutdown(ADI_ENCODER_SPI);
}

int adi_encoder_spi1_transfer(const uint8_t *transmit_data,
                              uint8_t *receive_data,
                              size_t length,
                              void *context)
{
    int status;
    uint32_t primask;
    mxc_spi_req_t request;

    (void)context;

    if ((transmit_data == NULL) || (receive_data == NULL) ||
        (length == 0u) || (length > (size_t)INT_MAX)) {
        return E_BAD_PARAM;
    }

    if (g_spi1_frequency_hz == 0u) {
        return E_BAD_STATE;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (g_spi1_transaction_active) {
        if (primask == 0u) {
            __enable_irq();
        }
        return E_BUSY;
    }

    g_spi1_transaction_active = true;
    __DMB();

    if (primask == 0u) {
        __enable_irq();
    }

    memset(&request, 0, sizeof(request));
    request.spi = ADI_ENCODER_SPI;
    request.txData = (uint8_t *)(uintptr_t)transmit_data;
    request.rxData = receive_data;
    request.txLen = (int)length;
    request.rxLen = (int)length;
    request.ssIdx = ADI_ENCODER_SPI_SS_INDEX;
    request.ssDeassert = 1;

    status = MXC_SPI_MasterTransaction(&request);

    __DMB();
    g_spi1_transaction_active = false;

    return status;
}

uint32_t adi_encoder_spi1_get_frequency_hz(void)
{
    return g_spi1_frequency_hz;
}
