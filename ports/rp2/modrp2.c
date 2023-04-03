/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/runtime.h"
#include "drivers/dht/dht.h"
#include "modrp2.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/adc.h"

#if MICROPY_PY_NETWORK_CYW43
#include "extmod/modnetwork.h"
#endif

#if CYW43_USES_VSYS_PIN || CYW43_WL_GPIO_VBUS_PIN
#include "lib/cyw43-driver/src/cyw43.h"
#endif

#if MICROPY_PY_NETWORK_CYW43
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(mod_network_country_obj);
#endif

// Improved version of
// https://github.com/raspberrypi/pico-examples/blob/master/picoboard/button/button.c
STATIC bool __no_inline_not_in_flash_func(bootsel_button)(void) {
    const uint CS_PIN_INDEX = 1;

    // Disable interrupts and the other core since they might be
    // executing code from flash and we are about to temporarily
    // disable flash access.
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();

    // Set the CS pin to high impedance.
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
        (GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB),
        IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Delay without calling any functions in flash.
    uint32_t start = timer_hw->timerawl;
    while ((uint32_t)(timer_hw->timerawl - start) <= MICROPY_HW_BOOTSEL_DELAY_US) {
        ;
    }

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // The button pulls the QSPI_SS pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1 << CS_PIN_INDEX));

    // Restore the QSPI_SS pin so we can use flash again.
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
        (GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB),
        IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    MICROPY_END_ATOMIC_SECTION(atomic_state);

    return button_state;
}

STATIC mp_obj_t rp2_bootsel_button(void) {
    return MP_OBJ_NEW_SMALL_INT(bootsel_button());
}
MP_DEFINE_CONST_FUN_OBJ_0(rp2_bootsel_button_obj, rp2_bootsel_button);

STATIC mp_obj_t rp2_power_source_battery(void) {
    bool vbus;
#if defined CYW43_WL_GPIO_VBUS_PIN
    cyw43_gpio_get(&cyw43_state, CYW43_WL_GPIO_VBUS_PIN, &vbus);
#elif defined PICO_VBUS_GPIO_PIN
    gpio_set_function(PICO_VBUS_GPIO_PIN, GPIO_FUNC_SIO);
    vbus = gpio_get(PICO_VBUS_GPIO_PIN);
#else
    mp_raise_NotImplementedError(MP_ERROR_TEXT("power_source not supported"));
#endif
    return MP_OBJ_NEW_SMALL_INT(!vbus);
}

MP_DEFINE_CONST_FUN_OBJ_0(rp2_power_source_battery_obj, rp2_power_source_battery);

#ifndef PICO_POWER_SAMPLE_COUNT
#define PICO_POWER_SAMPLE_COUNT 3
#endif

// Pin used for ADC 0
#define PICO_FIRST_ADC_PIN 26

STATIC mp_obj_t rp2_power_voltage(void) {
    float voltage_result;
#ifndef PICO_VSYS_PIN
    mp_raise_NotImplementedError(MP_ERROR_TEXT("power_voltage not supported"));
#endif
#if CYW43_USES_VSYS_PIN
    CYW43_THREAD_ENTER;
    // Make sure cyw43 is awake
    bool battery_powered;
    cyw43_gpio_get(&cyw43_state, CYW43_WL_GPIO_VBUS_PIN, &battery_powered);
    (void)battery_powered;
#endif

    // Initialise the ADC peripheral if it's not already running.
    if (!(adc_hw->cs & ADC_CS_EN_BITS)) {
        adc_init();
    }

    // setup adc
    adc_gpio_init(PICO_VSYS_PIN);
    adc_select_input(PICO_VSYS_PIN - PICO_FIRST_ADC_PIN);
 
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);

#if CYW43_USES_VSYS_PIN
    // We seem to read low values from cyw43 sometimes - this seems to fix it
    int ignore_count = PICO_POWER_SAMPLE_COUNT;
    while (!adc_fifo_is_empty() || ignore_count-- > 0) {
        (void)adc_fifo_get_blocking();
    }
#endif

    // read vsys
    uint32_t vsys = 0;
    for(int i = 0; i < PICO_POWER_SAMPLE_COUNT; i++) {
        uint16_t val = adc_fifo_get_blocking();
        vsys += val;
    }

    adc_run(false);
    adc_fifo_drain();

    vsys /= PICO_POWER_SAMPLE_COUNT;
#if CYW43_USES_VSYS_PIN
    CYW43_THREAD_EXIT;
#endif
    // Generate voltage
    const float conversion_factor = 3.3f / (1 << 12);
    voltage_result = vsys * 3 * conversion_factor;
    return mp_obj_new_float(voltage_result);
}

MP_DEFINE_CONST_FUN_OBJ_0(rp2_power_voltage_obj, rp2_power_voltage);

STATIC const mp_rom_map_elem_t rp2_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_rp2) },
    { MP_ROM_QSTR(MP_QSTR_Flash),               MP_ROM_PTR(&rp2_flash_type) },
    { MP_ROM_QSTR(MP_QSTR_PIO),                 MP_ROM_PTR(&rp2_pio_type) },
    { MP_ROM_QSTR(MP_QSTR_StateMachine),        MP_ROM_PTR(&rp2_state_machine_type) },
    { MP_ROM_QSTR(MP_QSTR_bootsel_button),      MP_ROM_PTR(&rp2_bootsel_button_obj) },
    { MP_ROM_QSTR(MP_QSTR_power_source_battery),MP_ROM_PTR(&rp2_power_source_battery_obj) },
    { MP_ROM_QSTR(MP_QSTR_power_voltage),       MP_ROM_PTR(&rp2_power_voltage_obj) },

    #if MICROPY_PY_NETWORK_CYW43
    // Deprecated (use network.country instead).
    { MP_ROM_QSTR(MP_QSTR_country),             MP_ROM_PTR(&mod_network_country_obj) },
    #endif
};
STATIC MP_DEFINE_CONST_DICT(rp2_module_globals, rp2_module_globals_table);

const mp_obj_module_t mp_module_rp2 = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&rp2_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__rp2, mp_module_rp2);
