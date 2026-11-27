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

// Handles the rx and tx status lights
//
// led_process() decides the indication by the rules below, applied from top:
//   1. Startup: RX and TX blink in turn (after a reset)
//   2. Bus off: RX and TX blink together while the channel is open
//   3. CAN error: RX and TX on while the node is error passive,
//      and for LED_EVENT_HOLD_MS after a CAN protocol error
//   4. Data loss: RX on and TX off for LED_EVENT_HOLD_MS after any data is discarded
//   5. Normal: TX on while the channel is closed, RX and TX blink by the activity
// The LEDs do not use the status flags of the F command, so reading them does not change the LEDs.
// See: https://github.com/Nakakiyo092/usb2canfdv1/discussions/214

#include "stm32g0xx_hal.h"
#include "can.h"
#include "led.h"

// Durations in ms
#define LED_BLINK_DURATION_MS       25U     // Activity blink: on time, and the minimum off time before the next one
#define LED_EVENT_HOLD_MS           500U    // CAN error and data loss indication after the event
#define LED_BUS_OFF_HALF_PERIOD_MS  250U    // Bus off blink

// Opening animation: 5 blinks, each half-period 100 ms (matches the old blocking sequence)
#define LED_ANIM_BLINKS             5U
#define LED_ANIM_HALF_PERIOD_MS     100U
#define LED_ANIM_TOTAL_MS           (LED_ANIM_BLINKS * 2 * LED_ANIM_HALF_PERIOD_MS)

// Private variables
// Activity blink state, applied to the LEDs by led_process()
static uint32_t led_rxd_last_time = 0;
static uint32_t led_txd_last_time = 0;
static enum LedState led_rxd_last_state = LED_OFF;
static enum LedState led_txd_last_state = LED_OFF;
// Time of the last event, valid while the held flag is set
static uint8_t led_error_held = 0;
static uint32_t led_error_time = 0;
static uint8_t led_loss_held = 0;
static uint32_t led_loss_time = 0;
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

// Turn TX LED on for a short duration
void led_blink_txd(void)
{
    // Make sure the LED has been off for at least LED_BLINK_DURATION_MS before turning on again
    // This prevents a solid status LED on a busy can bus
    if (led_txd_last_state == LED_OFF && (uint32_t)(HAL_GetTick() - led_txd_last_time) > LED_BLINK_DURATION_MS)
    {
        led_txd_last_time = HAL_GetTick();
        led_txd_last_state = LED_ON;
    }
}

// Turn RX LED on for a short duration
void led_blink_rxd(void)
{
    // Make sure the LED has been off for at least LED_BLINK_DURATION_MS before turning on again
    // This prevents a solid status LED on a busy can bus
    if (led_rxd_last_state == LED_OFF && (uint32_t)(HAL_GetTick() - led_rxd_last_time) > LED_BLINK_DURATION_MS)
    {
        led_rxd_last_time = HAL_GetTick();
        led_rxd_last_state = LED_ON;
    }
}

// Indicate a CAN protocol error for LED_EVENT_HOLD_MS
void led_notify_error(void)
{
    led_error_time = HAL_GetTick();
    led_error_held = 1;
}

// Indicate a data loss for LED_EVENT_HOLD_MS
void led_notify_loss(void)
{
    led_loss_time = HAL_GetTick();
    led_loss_held = 1;
}

// Process time-based LED events
void led_process(void)
{
    uint32_t now = HAL_GetTick();

    // Drive the non-blocking opening animation started in led_init()
    if (led_anim_active)
    {
        uint32_t elapsed = (uint32_t)(now - led_anim_start_tick);
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
        led_rxd_last_state = LED_OFF;
        led_txd_last_state = LED_OFF;
        led_rxd_last_time = now;
        led_txd_last_time = now;
    }

    // If an activity blink has been on for long enough, turn it off
    if (led_rxd_last_state == LED_ON && (uint32_t)(now - led_rxd_last_time) > LED_BLINK_DURATION_MS)
    {
        led_rxd_last_time = now;
        led_rxd_last_state = LED_OFF;
    }
    if (led_txd_last_state == LED_ON && (uint32_t)(now - led_txd_last_time) > LED_BLINK_DURATION_MS)
    {
        led_txd_last_time = now;
        led_txd_last_state = LED_OFF;
    }

    // End the event indications after the hold time
    if (led_error_held && (uint32_t)(now - led_error_time) >= LED_EVENT_HOLD_MS)
        led_error_held = 0;
    if (led_loss_held && (uint32_t)(now - led_loss_time) >= LED_EVENT_HOLD_MS)
        led_loss_held = 0;

    // The error state is reset on open and not updated while closed, so use it only while open
    uint8_t opened = (can_get_bus_state() == BUS_OPENED);
    struct CanErrorState error_state = can_get_error_state();

    enum LedState rxd;
    enum LedState txd;
    if (opened && error_state.bus_off)
    {
        // Bus off: blink together
        // The tick wraps at 2^32 ms, which is not a multiple of the blink period, so one half-period
        // just before the wrap is shorter. It happens once every 49.7 days, and is accepted.
        rxd = (((now / LED_BUS_OFF_HALF_PERIOD_MS) % 2) == 0) ? LED_ON : LED_OFF;
        txd = rxd;
    }
    else if ((opened && error_state.err_pssv) || led_error_held)
    {
        // CAN error: both on
        rxd = LED_ON;
        txd = LED_ON;
    }
    else if (led_loss_held)
    {
        // Data loss: RX only (TX off, to differ from the error indication while closed)
        rxd = LED_ON;
        txd = LED_OFF;
    }
    else
    {
        // Normal: TX on while closed, activity blink otherwise
        rxd = led_rxd_last_state;
        txd = opened ? led_txd_last_state : LED_ON;
    }

    HAL_GPIO_WritePin(LED_RXD, rxd);
    HAL_GPIO_WritePin(LED_TXD, txd);
}
