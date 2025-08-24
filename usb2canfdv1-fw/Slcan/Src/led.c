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

// Handles blinking of rx and tx status lights

#include "stm32g0xx_hal.h"
#include "led.h"
#include "slcan.h"

// Duration in ms
#define LED_BLINK_DURATION    (25)

// Private variables
static uint32_t led_rxd_last_time = 0;
static uint32_t led_txd_last_time = 0;
static enum led_state led_rxd_last_state = LED_OFF;
static enum led_state led_txd_last_state = LED_OFF;
static uint8_t led_error_was_indicating = 0;

// Initialize LED GPIOs
void led_init()
{
    HAL_GPIO_WritePin(LED_RXD, LED_ON);
    HAL_GPIO_WritePin(LED_TXD, LED_ON);
}

// Turn TX LED on/off
void led_turn_txd(enum led_state state)
{
    HAL_GPIO_WritePin(LED_TXD, state);
}

// Blink two LEDs (blocking)
void led_blink_sequence(uint8_t numblinks)
{
    uint8_t i;
    for (i = 0; i < numblinks; i++)
    {
        HAL_GPIO_WritePin(LED_RXD, LED_ON);
        HAL_GPIO_WritePin(LED_TXD, LED_OFF);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED_RXD, LED_OFF);
        HAL_GPIO_WritePin(LED_TXD, LED_ON);
        HAL_Delay(100);
    }
}

// Turn TX LED on for a short duration
void led_blink_txd(void)
{
    // Make sure the LED has been off for at least LED_BLINK_DURATION before turning on again
    // This prevents a solid status LED on a busy can bus
    if (led_txd_last_state == LED_OFF && (uint32_t)(HAL_GetTick() - led_txd_last_time) > LED_BLINK_DURATION)
    {
        HAL_GPIO_WritePin(LED_TXD, LED_ON);
        led_txd_last_time = HAL_GetTick();
        led_txd_last_state = LED_ON;
    }
}

// Turn RX LED on for a short duration
void led_blink_rxd(void)
{
    // Make sure the LED has been off for at least LED_BLINK_DURATION before turning on again
    // This prevents a solid status LED on a busy canbus
    if (led_rxd_last_state == LED_OFF && (uint32_t)(HAL_GetTick() - led_rxd_last_time) > LED_BLINK_DURATION)
    {
        HAL_GPIO_WritePin(LED_RXD, LED_ON);
        led_rxd_last_time = HAL_GetTick();
        led_rxd_last_state = LED_ON;
    }
}

// Process time-based LED events
void led_process(void)
{
    // If an error is stored, override LEDs with constant on
    if (slcan_get_status_flags())
    {
        HAL_GPIO_WritePin(LED_RXD, LED_ON);
        HAL_GPIO_WritePin(LED_TXD, LED_ON);
        led_error_was_indicating = 1;
    }
    // Otherwise, normal LED operation
    else
    {
        // If an error was stored but no longer is stored, turn the LEDs back off.
        if (led_error_was_indicating)
        {
            HAL_GPIO_WritePin(LED_RXD, LED_OFF);
            HAL_GPIO_WritePin(LED_TXD, LED_OFF);
            led_error_was_indicating = 0;
        }

        // If LED has been on for long enough, turn it off
        if (led_rxd_last_state == LED_ON && (uint32_t)(HAL_GetTick() - led_rxd_last_time) > LED_BLINK_DURATION)
        {
            HAL_GPIO_WritePin(LED_RXD, LED_OFF);
            led_rxd_last_time = HAL_GetTick();
            led_rxd_last_state = LED_OFF;
        }

        // If LED has been on for long enough, turn it off
        if (led_txd_last_state == LED_ON && (uint32_t)(HAL_GetTick() - led_txd_last_time) > LED_BLINK_DURATION)
        {
            // Invert LED
            HAL_GPIO_WritePin(LED_TXD, LED_OFF);
            led_txd_last_time = HAL_GetTick();
            led_txd_last_state = LED_OFF;
        }
    }
}
