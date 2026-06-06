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

// Handles blinking of rx and tx status lights

#include "stm32g0xx_hal.h"
#include "led.h"
#include "generator.h"

// Duration in ms
#define LED_BLINK_DURATION          25U

// Opening animation: 5 blinks, each half-period 100 ms (matches the old blocking sequence)
#define LED_ANIM_BLINKS             5U
#define LED_ANIM_HALF_PERIOD_MS     100U
#define LED_ANIM_TOTAL_MS           (LED_ANIM_BLINKS * 2 * LED_ANIM_HALF_PERIOD_MS)

// Private variables
static uint32_t led_rxd_last_time = 0;
static uint32_t led_txd_last_time = 0;
static enum LedState led_rxd_last_state = LED_OFF;
static enum LedState led_txd_last_state = LED_OFF;
static uint8_t led_error_was_indicating = 0;
static uint32_t led_anim_start_tick = 0;
static uint8_t led_anim_active = 0;

// Initialize LED GPIOs and start the non-blocking opening animation
void led_init(void)
{
    HAL_GPIO_WritePin(LED_RXD, LED_ON);
    HAL_GPIO_WritePin(LED_TXD, LED_ON);
    led_anim_start_tick = HAL_GetTick();
    led_anim_active = 1;
}

// Turn TX LED on/off; requests are ignored while the opening animation is running
void led_turn_txd(enum LedState state)
{
    if (led_anim_active)
        return;
    HAL_GPIO_WritePin(LED_TXD, state);
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
    // Drive the non-blocking opening animation started in led_init()
    if (led_anim_active)
    {
        uint32_t elapsed = (uint32_t)(HAL_GetTick() - led_anim_start_tick);
        if (elapsed < LED_ANIM_TOTAL_MS)
        {
            // Alternate every half-period: even -> RXD=ON, TXD=OFF; odd -> RXD=OFF, TXD=ON
            uint32_t half = elapsed / LED_ANIM_HALF_PERIOD_MS;
            if ((half % 2) == 0)
            {
                HAL_GPIO_WritePin(LED_RXD, LED_ON);
                HAL_GPIO_WritePin(LED_TXD, LED_OFF);
            }
            else
            {
                HAL_GPIO_WritePin(LED_RXD, LED_OFF);
                HAL_GPIO_WritePin(LED_TXD, LED_ON);
            }
            return;
        }
        // Animation complete: let normal LED logic take over from a clean state
        led_anim_active = 0;
        HAL_GPIO_WritePin(LED_RXD, LED_OFF);
        HAL_GPIO_WritePin(LED_TXD, LED_OFF);
        led_rxd_last_state = LED_OFF;
        led_txd_last_state = LED_OFF;
        led_rxd_last_time = HAL_GetTick();
        led_txd_last_time = HAL_GetTick();
    }

    // If an error is stored, override LEDs with constant on
    if (gen_get_status_flags())
    {
        HAL_GPIO_WritePin(LED_RXD, LED_ON);
        HAL_GPIO_WritePin(LED_TXD, LED_ON);
        led_rxd_last_state = LED_OFF;
        led_txd_last_state = LED_OFF;
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
            HAL_GPIO_WritePin(LED_TXD, LED_OFF);
            led_txd_last_time = HAL_GetTick();
            led_txd_last_state = LED_OFF;
        }
    }
}
