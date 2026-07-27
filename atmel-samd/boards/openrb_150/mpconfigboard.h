#ifndef __MICROPY_INCLUDED_ATMEL_SAMD_BOARDS_OPENRB_150_MPCONFIGBOARD_H__
#define __MICROPY_INCLUDED_ATMEL_SAMD_BOARDS_OPENRB_150_MPCONFIGBOARD_H__

// Hardware Identity Declarations
#define MICROPY_HW_BOARD_NAME "OpenRB-150"
#define MICROPY_HW_MCU_NAME   "samd21g18"

// Clock Configuration (OpenRB-150 uses internal calibration instead of an external crystal)
#define BOARD_HAS_CRYSTAL 0

// Hardware Onboard Status Indicators
// FIXED: was &pin_PA17, which is actually D9/SCK on this board (see
// variant.cpp's SPI table) -- the real LED_BUILTIN is PB08 (Arduino pin 32
// per variant.cpp / BDPIN LED define in the e-manual). Must match the LED
// mapping in pins.c.
#define MICROPY_HW_LED_STATUS   (&pin_PB08)

// Standard Peripheral Bus Defaults (Mapped to the Arduino MKR form-factor footprint edge headers)
#define DEFAULT_I2C_BUS_SCL (&pin_PA09)
#define DEFAULT_I2C_BUS_SDA (&pin_PA08)

// FIXED: SPI defaults were PB10/PB11/PA12 (D4/D5, plus PA12 which is now
// the DXL Serial1 TX pin -- a direct collision). variant.cpp's own SPI
// section labels PA16/PA17/PA19 (D8/D9/D10) as MOSI/SCK/MISO, so the
// default SPI bus should use those, matching the physical silkscreen.
#define DEFAULT_SPI_BUS_SCK  (&pin_PA17)
#define DEFAULT_SPI_BUS_MOSI (&pin_PA16)
#define DEFAULT_SPI_BUS_MISO (&pin_PA19)

#define DEFAULT_UART_BUS_TX (&pin_PA22)
#define DEFAULT_UART_BUS_RX (&pin_PA23)

// ==============================================================================
// MEMORY PARTITION FORCING: flash configuration
// ==============================================================================
// Shrink internal flash drive layout from 38KB down to 64KB.
// This instantly grants an extra 38KB of flash storage space to FLASH_FIRMWARE.
// We DO NOT define START_ADDR here; mpconfigport.h will compute it automatically.
#define CIRCUITPY_INTERNAL_FLASH_FILESYSTEM_SIZE (38* 1024)

#endif // __MICROPY_INCLUDED_ATMEL_SAMD_BOARDS_OPENRB_150_MPCONFIGBOARD_H__

//#ifndef CIRCUITPY_DEFAULT_STACK_SIZE
//#define CIRCUITPY_DEFAULT_STACK_SIZE 5632   // default is 3584 on SAMD21
//#endif

