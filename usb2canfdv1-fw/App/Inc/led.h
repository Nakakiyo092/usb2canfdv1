///////////////////////////////////////////////////////////////////////////////
// GNU General Public License v3.0
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Full license text: https://www.gnu.org/licenses/gpl-3.0.txt
// See also: LICENSE.md in the root of this repository
///////////////////////////////////////////////////////////////////////////////

#ifndef USB2CANFDV1_LED_H
#define USB2CANFDV1_LED_H

#include "main.h"

// LED state (active-low: LED_ON = GPIO_PIN_RESET drives pin LOW to light the LED)
enum LedState
{
    LED_ON  = GPIO_PIN_RESET,
    LED_OFF = GPIO_PIN_SET
};

// GPIO definitions
#define LED_RXD LED_RXD_GPIO_Port , LED_RXD_Pin
#define LED_TXD LED_TXD_GPIO_Port , LED_TXD_Pin

// Prototypes
void led_init(void);
void led_blink_txd(void);
void led_blink_rxd(void);
void led_notify_error(void);
void led_notify_loss(void);
void led_process(void);     // Writes the LEDs; the functions above only record the request (except led_init)

#endif // USB2CANFDV1_LED_H
