# MAX32672 SPI1 encoder integration

## Files
Copy `include/*.h` into the project's `include` directory and `src/*.c` into `src`.
The example file may be copied into `src` for initial bring-up.

## Hardware mapping
- P0.14: SPI1 MISO
- P0.15: SPI1 MOSI
- P0.16: SPI1 SCK
- P0.17: SPI1 SS0

Verify the exact P4 connector pin numbers from the graphical schematic or by unpowered continuity testing before connecting the encoder.

## Build configuration
Add these sources if the project does not automatically discover `src`:

```makefile
SRCS += src/adi_encoder_spi1_max32672.c
SRCS += src/adi_encoder_test.c
SRCS += src/maulin_proto.c
SRCS += examples/adi_encoder_test_example.c
IPATH += include
PERIPH_DRIVER += SPI
```

## Loopback test
1. Disconnect the encoder from P4.
2. Connect P0.15 MOSI to P0.14 MISO.
3. Call `run_spi1_loopback_test()` after console initialization.
4. Start at 1 MHz.

## Encoder communication test
1. Remove the loopback connection.
2. Connect the encoder to P4.
3. Verify `TEST_REGISTER_ADDRESS` against the authoritative register map.
4. Call `run_encoder_communication_test()` after console initialization.

## Protocol configuration
- SPI Mode 0
- MSB first
- 8-bit SPI characters
- 8-byte protocol frames
- Active-low SS0
- CRC-8 initial value 0xA5, polynomial 0x07
- Register reads are pipelined: send READ, then send NOP to retrieve the response

## Important
The implementation uses hardware SS0. If the final encoder timing requires explicit chip-select setup, hold-low, or inactive delays, change P0.17 to software-controlled GPIO and implement those timing requirements around each transfer.
