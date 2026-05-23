///////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
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
void led_turn_txd(enum LedState state); // No led_turn_rxd: RX LED is managed exclusively via led_blink_rxd/led_process
void led_blink_sequence(uint8_t numblinks);
void led_blink_txd(void);
void led_blink_rxd(void);
void led_process(void);

#endif // USB2CANFDV1_LED_H
