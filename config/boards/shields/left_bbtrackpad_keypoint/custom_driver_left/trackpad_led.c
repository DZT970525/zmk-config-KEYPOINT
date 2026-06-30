/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>
#include <zmk/backlight.h>
#include <zmk/activity.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>
#include "trackpad_led.h"
#include "a320.h"

#define HID_INDICATORS_CAPS_LOCK (1 << 1)

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_trackpad_led),
             "CONFIG_ZMK_TRACKPAD_LED enabled but no zmk,trackpad_led chosen node found");

static const struct device *const led_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_trackpad_led));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define INDICATOR_LED_NUM_LEDS (DT_NUM_CHILD(DT_CHOSEN(zmk_trackpad_led)))

#define BRT_MIN 10
#define BRT_MAX 100
#define BRT_LOW 20
#define BRT_STEP 5

#define ANIMATION_INTERVAL_MS 20
#define POLLING_INTERVAL_MS 5
#define AUTO_OFF_DELAY_MS 5000

#define FLASH_ON_MS 100   /* USB mode ON duration */
#define FLASH_PERIOD 1000 /* Total USB flash period */

#define PULSE_STEP_MS 10
#define PULSE_SEQ_LEN 41 /* 10 fade-in + 20 hold + 6 fade-out + 5 rest */
#define PULSE_REPEAT_MS 2000

static struct k_work_delayable polling_work;
static struct k_work_delayable animation_work;
static struct k_work_delayable auto_off_work;
static struct k_work_delayable usb_flash_work;
static struct k_work_delayable pulse_work;
static struct k_work_delayable pulse_repeat_work;

static bool capslock_on = false;
static bool touch_active = false;
static bool animation_increasing = true;
static uint8_t brightness = BRT_MIN;

static uint8_t last_valid_brt = BRT_MAX;
static uint8_t last_backlight_brt = 0;
static bool manual_override = false;
static bool keyboard_active = false;

static bool usb_flash_state = false;
static bool usb_mode = false;

static const uint8_t pulse_indicator_layers[] = {1, 2};

#define PULSE_INDICATOR_LAYER_COUNT ARRAY_SIZE(pulse_indicator_layers)

static const uint8_t pulse_seq[PULSE_SEQ_LEN] = {
    /* fade-in: ease-out 0→80, 10 steps */
    0, 8, 15, 22, 29, 35, 41, 46, 51, 56,
    /* hold: 80, 20 steps */
    80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
    80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
    /* fade-out: ease-in 80→0, 6 steps */
    80, 77, 67, 51, 29, 0,
    /* rest: 0, 5 steps */
    0, 0, 0, 0, 0,
};

static uint8_t pulse_remaining = 0;
static uint8_t pulse_step = 0;
static bool pulse_active = false;
static uint8_t pulse_saved_brt = 0;
static uint8_t led_hw_brt = 0;
static uint8_t last_layer = 0;

static void pulse_work_handler(struct k_work *work);
static void pulse_repeat_work_handler(struct k_work *work);
static bool pulse_layer_indicate(uint8_t layer);
void trackpad_led_pulse(uint8_t count);

static void pulse_stop(void) {
    k_work_cancel_delayable(&pulse_work);
    k_work_cancel_delayable(&pulse_repeat_work);
    pulse_active = false;
}

static void set_led_brightness(uint8_t level) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED device not ready");
        return;
    }
    for (int i = 0; i < INDICATOR_LED_NUM_LEDS; i++) {
        int err = led_set_brightness(led_dev, i, level);
        if (err < 0) {
            LOG_ERR("Failed to set LED[%d] brightness: %d", i, err);
        }
    }
    led_hw_brt = level;
}

static bool pulse_layer_indicate(uint8_t layer) {
    for (uint8_t i = 0; i < PULSE_INDICATOR_LAYER_COUNT; i++) {
        if (pulse_indicator_layers[i] == layer) {
            return true;
        }
    }
    return false;
}

/* USB flashing handler */
static void usb_flash_work_handler(struct k_work *work) {
    if (!usb_mode) {
        set_led_brightness(0);
        return;
    }

    usb_flash_state = !usb_flash_state;
    set_led_brightness(usb_flash_state ? BRT_MAX : 0);

    k_work_reschedule(&usb_flash_work,
                      K_MSEC(usb_flash_state ? FLASH_ON_MS : (FLASH_PERIOD - FLASH_ON_MS)));
}

static void auto_off_work_handler(struct k_work *work) {
    if (!capslock_on && !touch_active && !pulse_active) {
        manual_override = false;
        set_led_brightness(0);
        LOG_DBG("Auto-off triggered after inactivity");
    }
}

static void animation_work_handler(struct k_work *work) {
    if (!capslock_on)
        return;

    if (animation_increasing) {
        brightness += BRT_STEP;
        if (brightness >= BRT_MAX) {
            brightness = BRT_MAX;
            animation_increasing = false;
        }
    } else {
        brightness -= BRT_STEP;
        if (brightness <= BRT_LOW) {
            brightness = BRT_LOW;
            animation_increasing = true;
        }
    }

    set_led_brightness(brightness);
    k_work_reschedule(&animation_work, K_MSEC(ANIMATION_INTERVAL_MS));
}

static void pulse_work_handler(struct k_work *work) {
    set_led_brightness(pulse_seq[pulse_step]);
    pulse_step++;

    if (pulse_step >= PULSE_SEQ_LEN) {
        pulse_remaining--;
        if (pulse_remaining > 0) {
            pulse_step = 0;
            k_work_reschedule(&pulse_work, K_NO_WAIT);
        } else {
            pulse_active = false;
            if (pulse_layer_indicate(zmk_keymap_highest_layer_active())) {
                set_led_brightness(pulse_saved_brt > 0 ? pulse_saved_brt : 0);
            } else {
                set_led_brightness(0);
            }
        }
        return;
    }

    k_work_reschedule(&pulse_work, K_MSEC(PULSE_STEP_MS));
}

void trackpad_led_pulse(uint8_t count) {
    k_work_cancel_delayable(&pulse_work);
    if (!pulse_active) {
        pulse_saved_brt = led_hw_brt;
    }
    pulse_active = true;
    pulse_remaining = count;
    pulse_step = 0;
    if (count > 0) {
        k_work_reschedule(&pulse_work, K_NO_WAIT);
    }
}

static void polling_work_handler(struct k_work *work) {
    enum zmk_transport transport = zmk_endpoints_selected().transport;
    bool current_capslock = (zmk_hid_indicators_get_current_profile() & HID_INDICATORS_CAPS_LOCK);
    bool current_touch = tp_is_touched();
    bool current_active = (zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    uint8_t current_brt = zmk_backlight_get_brt();

    /* ---------------- USB mode ---------------- */
    if (transport == ZMK_TRANSPORT_USB) {
        if (!usb_mode) {
            usb_mode = true;
            usb_flash_state = false;
            pulse_stop();
            k_work_reschedule(&usb_flash_work, K_NO_WAIT);
            LOG_INF("Entered USB flash mode");
        }
        k_work_reschedule(&polling_work, K_MSEC(POLLING_INTERVAL_MS));
        return;
    }

    /* ---------------- BLE mode ---------------- */
    if (usb_mode) {
        usb_mode = false;
        k_work_cancel_delayable(&usb_flash_work);
        set_led_brightness(0);
        LOG_INF("Exited USB flash mode");
    }

    if (current_active != keyboard_active) {
        keyboard_active = current_active;
        if (keyboard_active) {
            last_backlight_brt = current_brt;
        }
    }

    /* CapsLock animation */
    if (current_capslock != capslock_on) {
        capslock_on = current_capslock;
        if (capslock_on) {
            pulse_stop();
            brightness = BRT_MIN;
            animation_increasing = true;
            k_work_reschedule(&animation_work, K_NO_WAIT);
        } else {
            k_work_cancel_delayable(&animation_work);
            manual_override = false;

            if (current_touch) {
                touch_active = true;
                manual_override = true;
                if (keyboard_active) {
                    last_valid_brt = MAX(BRT_MIN, current_brt);
                }
                if (!pulse_active) {
                    set_led_brightness(last_valid_brt);
                }
                k_work_cancel_delayable(&auto_off_work);
            } else if (!pulse_active) {
                set_led_brightness(0);
            }
        }
    }

    /* Touch event handling */
    if (!capslock_on && current_touch != touch_active) {
        touch_active = current_touch;
        if (touch_active) {
            manual_override = true;
            if (keyboard_active) {
                last_valid_brt = MAX(BRT_MIN, current_brt);
            }
            if (!pulse_active) {
                set_led_brightness(last_valid_brt);
            }
            k_work_cancel_delayable(&auto_off_work);
        } else {
            k_work_reschedule(&auto_off_work, K_MSEC(AUTO_OFF_DELAY_MS));
        }
    }

    /* Backlight brightness change */
    if (!capslock_on && !touch_active && current_brt != last_backlight_brt && keyboard_active) {
        last_backlight_brt = current_brt;
        if (current_brt > 0) {
            manual_override = true;
            last_valid_brt = MAX(BRT_MIN, current_brt);
            if (!pulse_active) {
                set_led_brightness(last_valid_brt);
            }
            k_work_reschedule(&auto_off_work, K_MSEC(AUTO_OFF_DELAY_MS));
        }
    }

    k_work_reschedule(&polling_work, K_MSEC(POLLING_INTERVAL_MS));
}

static int layer_change_listener(const zmk_event_t *eh) {
    uint8_t current_layer = zmk_keymap_highest_layer_active();
    if (current_layer != last_layer) {
        last_layer = current_layer;
        k_work_cancel_delayable(&pulse_repeat_work);
        if (!capslock_on && pulse_layer_indicate(current_layer)) {
            if (!pulse_active) {
                pulse_saved_brt = led_hw_brt;
            }
            trackpad_led_pulse(current_layer);
            k_work_reschedule(&pulse_repeat_work, K_MSEC(PULSE_REPEAT_MS));
            LOG_INF("Layer %d pulse %d times", current_layer, current_layer);
        }
    }
    return 0;
}

ZMK_LISTENER(layer_change_led_listener, layer_change_listener);
ZMK_SUBSCRIPTION(layer_change_led_listener, zmk_layer_state_changed);

uint8_t indicator_tp_get_last_valid_brightness(void) { return last_valid_brt; }

static void pulse_repeat_work_handler(struct k_work *work) {
    uint8_t current_layer = zmk_keymap_highest_layer_active();
    if (!capslock_on && pulse_layer_indicate(current_layer) && !pulse_active) {
        trackpad_led_pulse(current_layer);
    }
    if (!capslock_on && pulse_layer_indicate(current_layer)) {
        k_work_reschedule(&pulse_repeat_work, K_MSEC(PULSE_REPEAT_MS));
    }
}

static int indicator_tp_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED indicator_tp device not ready");
        return -ENODEV;
    }

    set_led_brightness(0);
    usb_mode = false;
    usb_flash_state = false;
    last_backlight_brt = zmk_backlight_get_brt();
    capslock_on = touch_active = manual_override = keyboard_active = false;
    last_layer = zmk_keymap_highest_layer_active();

    k_work_init_delayable(&polling_work, polling_work_handler);
    k_work_init_delayable(&animation_work, animation_work_handler);
    k_work_init_delayable(&auto_off_work, auto_off_work_handler);
    k_work_init_delayable(&usb_flash_work, usb_flash_work_handler);
    k_work_init_delayable(&pulse_work, pulse_work_handler);
    k_work_init_delayable(&pulse_repeat_work, pulse_repeat_work_handler);

    k_work_reschedule(&polling_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(indicator_tp_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
