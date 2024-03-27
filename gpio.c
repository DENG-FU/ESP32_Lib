//----------------------------------------------------------------------------
// Description: ESP32 gpio control self abstract layer
//
// Author     : dengfu
// Date       : 2024/03/03
// Filename   : ESP32_Lib/gpio.c
//----------------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>

#include "gpio.h"

#include "driver/gpio.h"




void mcu_gpio_init(void)
{
    gpio_reset_pin(4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(4, GPIO_MODE_OUTPUT);
}

uint8_t mcu_get_pin_value(uint8_t port, uint8_t pin)
{
    return gpio_get_level(pin);
}


void mcu_set_pin_value(uint8_t port, uint8_t pin, uint8_t level)
{
    gpio_set_level(pin, level);
}