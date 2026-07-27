#include "py/obj.h"
#include "shared-bindings/board/__init__.h"

static const mp_rom_map_elem_t board_global_dict_table[] = {
    // Standard Breadboard Edge Footprint Tracks (Arduino MKR Mirror)
    { MP_ROM_QSTR(MP_QSTR_D0), MP_ROM_PTR(&pin_PA22) },
    { MP_ROM_QSTR(MP_QSTR_D1), MP_ROM_PTR(&pin_PA23) },
    { MP_ROM_QSTR(MP_QSTR_D2), MP_ROM_PTR(&pin_PA10) },
    { MP_ROM_QSTR(MP_QSTR_D3), MP_ROM_PTR(&pin_PA11) },
    { MP_ROM_QSTR(MP_QSTR_D4), MP_ROM_PTR(&pin_PB10) },
    { MP_ROM_QSTR(MP_QSTR_D5), MP_ROM_PTR(&pin_PB11) },
    { MP_ROM_QSTR(MP_QSTR_D6), MP_ROM_PTR(&pin_PA20) },
    { MP_ROM_QSTR(MP_QSTR_D7), MP_ROM_PTR(&pin_PA21) },
    { MP_ROM_QSTR(MP_QSTR_D8), MP_ROM_PTR(&pin_PA16) },
    { MP_ROM_QSTR(MP_QSTR_D9), MP_ROM_PTR(&pin_PA17) },
    { MP_ROM_QSTR(MP_QSTR_D10), MP_ROM_PTR(&pin_PA19) },
    { MP_ROM_QSTR(MP_QSTR_D11), MP_ROM_PTR(&pin_PA08) },
    { MP_ROM_QSTR(MP_QSTR_D12), MP_ROM_PTR(&pin_PA09) },

    // Standard Analog
    { MP_ROM_QSTR(MP_QSTR_A0), MP_ROM_PTR(&pin_PA02) },
    { MP_ROM_QSTR(MP_QSTR_A1), MP_ROM_PTR(&pin_PB02) },
    { MP_ROM_QSTR(MP_QSTR_A2), MP_ROM_PTR(&pin_PB03) },
    { MP_ROM_QSTR(MP_QSTR_A3), MP_ROM_PTR(&pin_PA04) },
    { MP_ROM_QSTR(MP_QSTR_A4), MP_ROM_PTR(&pin_PA05) },
    { MP_ROM_QSTR(MP_QSTR_A5), MP_ROM_PTR(&pin_PA06) },
    { MP_ROM_QSTR(MP_QSTR_A6), MP_ROM_PTR(&pin_PA07) },

    // FIXED: DXL bus UART. ROBOTIS' own Dynamixel2Arduino examples define
    // DXL_SERIAL as Serial1 for ARDUINO_OpenRB, and the real variant.cpp
    // shows Serial1 is SERCOM2 on PA12 (TX) / PA13 (RX) -- NOT PB22/PB23
    // (that pair is Serial3 on this board) and NOT PA24/PA25 (native USB).
    { MP_ROM_QSTR(MP_QSTR_TX1), MP_ROM_PTR(&pin_PA12) },
    { MP_ROM_QSTR(MP_QSTR_RX1), MP_ROM_PTR(&pin_PA13) },

    // Hardware Features
    // FIXED: LED_BUILTIN is PB08 (Arduino pin 32 per variant.cpp/e-manual),
    // not PA17 -- PA17 is already D9/SCK above, so the old mapping silently
    // aliased two different board pins onto the same physical pad.
    { MP_ROM_QSTR(MP_QSTR_LED), MP_ROM_PTR(&pin_PB08) },
    // FIXED: DXL_PWR_EN is PA28 (Arduino pin 31, "DXL_PWR_SW" in variant.cpp
    // and BDPIN_DXL_PWR_EN=31u in the e-manual), not PA18 -- PA18 is
    // actually the USB ID pin on this hardware.
    { MP_ROM_QSTR(MP_QSTR_DXL_PWR_EN), MP_ROM_PTR(&pin_PA28) },
};
MP_DEFINE_CONST_DICT(board_module_globals, board_global_dict_table);
