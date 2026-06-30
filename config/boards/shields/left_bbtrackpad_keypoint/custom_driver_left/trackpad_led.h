/*
 * Copyright (c) 2025 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get current indicator led brightness
 *
 * @return uint8_t valid LED brightness
 */
uint8_t indicator_tp_get_last_valid_brightness(void);

/**
 * @brief Run pulse LED effect
 *
 * Pre-computed 70-step sequence: ease-out 0→80 (100ms) → hold 80 (200ms) →
 *           ease-in 80→0 (50ms) → repeat
 *
 * @param count number of times to repeat the pulse
 */
void trackpad_led_pulse(uint8_t count);

#ifdef __cplusplus
}
#endif
