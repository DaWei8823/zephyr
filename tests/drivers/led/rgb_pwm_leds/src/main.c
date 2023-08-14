
/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "zephyr/drivers/led.h"
#include "zephyr/kernel.h"
#include <zephyr/ztest.h>
#include <stdint.h>

static const uint8_t RED[] = {0xFF, 0, 0, 0x7F};
static const uint8_t GREEN[] = {0, 0xFF, 0, 0x7F};
static const uint8_t BLUE[] = {0, 0, 0xFF, 0x7F};
const uint8_t *COLORS[] = {RED, GREEN, BLUE};
#define NUM_COLORS  ARRAY_SIZE(COLORS)
#define NUM_RGBA_CHANNELS  4
#define IGNORE_LED_INDEX   0

ZTEST_SUITE(rgb_pwm_led_tests, NULL, NULL, NULL, NULL, NULL);

/**
 * @brief Test all colors
 *
 * This test goes through red, green and blue leds and turns them on/off
 *
 */
ZTEST(rgb_pwm_led_tests, test_all_colors)
{
	const struct device *leds = DEVICE_DT_GET_ONE(rgb_pwm_leds);
	int ret;

	for (int color = 0; color < NUM_COLORS; color++) {
		ret = led_set_color(leds, IGNORE_LED_INDEX, NUM_RGBA_CHANNELS, COLORS[color]);
		zassert_ok(ret);
		k_sleep(K_MSEC(500));
		ret = led_off(leds, color);
		zassert_ok(ret);
	}
}
