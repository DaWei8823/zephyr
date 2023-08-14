/*
 * (c) Meta Platforms, Inc. and affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT rgb_pwm_leds

/**
 * @file
 * @brief PWM driven RGB LEDs
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rgb_led_pwm, CONFIG_LED_LOG_LEVEL);

#define ALPHA_MAX                            0xFF
#define MIN_BLINK_TIMER_PERIOD_MS            10
#define CURRENT_BLINK_PERIOD_TO_RETRY_PERIOD 5
#define SET_LED_MUTEX_TIMEOUT_MS             200

typedef union __packed {
	struct {
		uint8_t red;
		uint8_t green;
		uint8_t blue;
		uint8_t alpha;
	} rgba;
	uint8_t raw[4];
} color_t;

struct rgb_led_pwm_config {
	const struct device *leds_base;
	uint32_t red_led_idx;
	uint32_t blue_led_idx;
	uint32_t green_led_idx;
	color_t init_color;
};

struct rgb_led_pwm_data {
	color_t current_color;
	struct k_timer blink_timer;
	struct k_mutex mutex;
	bool blink_state;
	uint32_t blink_off_ms;
	uint32_t blink_on_ms;
};

const color_t LED_OFF_COLOR = {
	.rgba = {
			.alpha = 0,
			.red = 0,
			.blue = 0,
			.green = 0,
		},
};

static int _rgb_led_pwm_set(const struct device *dev, color_t color)
{
	const struct rgb_led_pwm_config *config = dev->config;
	uint8_t red = (color.rgba.red * color.rgba.alpha) / ALPHA_MAX;
	uint8_t blue = (color.rgba.blue * color.rgba.alpha) / ALPHA_MAX;
	uint8_t green = (color.rgba.green * color.rgba.alpha) / ALPHA_MAX;
	int ret = 0;

	ret = led_set_brightness(config->leds_base, config->red_led_idx, red);
	if (ret != 0) {
		LOG_ERR("could not set red to %u", red);
		return ret;
	}

	ret = led_set_brightness(config->leds_base, config->blue_led_idx, blue);
	if (ret != 0) {
		LOG_ERR("could not set blue to %u", blue);
		return ret;
	}

	ret = led_set_brightness(config->leds_base, config->green_led_idx, green);
	if (ret != 0) {
		LOG_ERR("could not set green to %u", green);
		return ret;
	}

	return ret;
}

static void _launch_retry_timer(struct k_timer *timer, uint32_t current_period_ms)
{
	uint32_t retry_ms = current_period_ms / CURRENT_BLINK_PERIOD_TO_RETRY_PERIOD;

	retry_ms = MAX(retry_ms, MIN_BLINK_TIMER_PERIOD_MS);
	k_timer_start(timer, K_MSEC(retry_ms), K_FOREVER);
}

static void _rgb_led_pwm_blink_timer_fn(struct k_timer *timer)
{
	const struct device *dev = k_timer_user_data_get(timer);
	struct rgb_led_pwm_data *data = dev->data;
	uint32_t next_expiry_ms = 0;
	int ret;

	ret = k_mutex_lock(&data->mutex, K_NO_WAIT);
	if (ret != 0) {
		/* interrupted some led function so we'll just try again a little later */
		_launch_retry_timer(timer,
				    data->blink_state ? data->blink_on_ms : data->blink_off_ms);
		return;
	}

	data->blink_state = !data->blink_state;
	if (data->blink_state) {
		ret = _rgb_led_pwm_set(dev, data->current_color);
		next_expiry_ms = data->blink_on_ms;
	} else {
		ret = _rgb_led_pwm_set(dev, LED_OFF_COLOR);
		next_expiry_ms = data->blink_off_ms;
	}

	if (ret) {
		LOG_ERR("failed to set blink that %s", data->blink_state ? "on" : "off");
	}

	k_timer_start(timer, K_MSEC(next_expiry_ms), K_FOREVER);
	k_mutex_unlock(&data->mutex);
}

static int rgb_led_pwm_on(const struct device *dev, uint32_t _led)
{
	struct rgb_led_pwm_data *data = dev->data;
	int ret;

	ret = k_mutex_lock(&data->mutex, K_MSEC(SET_LED_MUTEX_TIMEOUT_MS));
	if (ret != 0) {
		LOG_ERR("could not lock mutex: %d", ret);
		return ret;
	}

	ret = _rgb_led_pwm_set(dev, data->current_color);
	if (ret != 0) {
		LOG_ERR("failed to turn led on");
	}

	k_mutex_unlock(&data->mutex);
	return ret;
}

static int rgb_led_pwm_off(const struct device *dev, uint32_t _led)
{
	struct rgb_led_pwm_data *data = dev->data;
	int ret;

	ret = k_mutex_lock(&data->mutex, K_MSEC(SET_LED_MUTEX_TIMEOUT_MS));

	if (ret != 0) {
		LOG_ERR("could not lock mutex: %d", ret);
		return ret;
	}
	k_timer_stop(&data->blink_timer);

	ret = _rgb_led_pwm_set(dev, LED_OFF_COLOR);
	if (ret != 0) {
		LOG_ERR("failed to turn led off");
	}

	k_mutex_unlock(&data->mutex);
	return ret;
}

static int rgb_led_pwm_blink(const struct device *dev, uint32_t led, uint32_t delay_on,
			     uint32_t delay_off)
{
	struct rgb_led_pwm_data *data = dev->data;
	int ret = 0;

	if (!delay_on || !delay_off) {
		ret = -EINVAL;
	}

	ret = k_mutex_lock(&data->mutex, K_MSEC(SET_LED_MUTEX_TIMEOUT_MS));
	if (ret != 0) {
		LOG_ERR("could not lock mutex: %d", ret);
		return ret;
	}

	k_timer_stop(&data->blink_timer);

	data->blink_on_ms = delay_on;
	data->blink_off_ms = delay_off;
	data->blink_state = false;

	_rgb_led_pwm_set(dev, LED_OFF_COLOR);
	k_timer_start(&data->blink_timer, K_MSEC(data->blink_off_ms), K_FOREVER);

	k_mutex_unlock(&data->mutex);
	return ret;
}

int rgb_led_pwm_set_brightness(const struct device *dev, uint32_t _led, uint8_t value)
{
	struct rgb_led_pwm_data *data = dev->data;

	data->current_color.rgba.alpha = value;
	return _rgb_led_pwm_set(dev, data->current_color);
}

int rgb_led_pwm_set_color(const struct device *dev, uint32_t led, uint8_t num_colors,
			  const uint8_t *color)
{
	struct rgb_led_pwm_data *data = dev->data;

	if (num_colors < 3 || num_colors > 4) {
		LOG_ERR("invalid num_colors: %u. must be 3 for rgb or 4 for rgba", num_colors);
		return -EINVAL;
	}

	memcpy(data->current_color.raw, color, num_colors * sizeof(color[0]));
	return _rgb_led_pwm_set(dev, data->current_color);
}

static int rgb_led_pwm_init(const struct device *dev)
{
	const struct rgb_led_pwm_config *config = dev->config;
	struct rgb_led_pwm_data *data = dev->data;

	if (!device_is_ready(config->leds_base)) {
		LOG_ERR("%s: pwm device not ready", config->leds_base->name);
		return -ENODEV;
	}

	k_mutex_init(&data->mutex);
	k_timer_init(&data->blink_timer, _rgb_led_pwm_blink_timer_fn, NULL);
	k_timer_user_data_set(&data->blink_timer, (void *)dev);

	data->current_color = config->init_color;
	return 0;
}

static const struct led_driver_api rgb_led_pwm_api = {
	.on = rgb_led_pwm_on,
	.off = rgb_led_pwm_off,
	.blink = rgb_led_pwm_blink,
	.set_brightness = rgb_led_pwm_set_brightness,
	.set_color = rgb_led_pwm_set_color,

};

#define RGB_LED_PWM_DEVICE(id)                                                                     \
	static const struct rgb_led_pwm_config rgb_led_pwm_config_##id = {                         \
		.leds_base = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(DT_DRV_INST(id), pwm_leds, 0)),       \
		.red_led_idx = DT_NODE_CHILD_IDX(DT_NODELABEL(red_pwm_led)),                       \
		.green_led_idx = DT_NODE_CHILD_IDX(DT_NODELABEL(green_pwm_led)),                   \
		.blue_led_idx = DT_NODE_CHILD_IDX(DT_NODELABEL(blue_pwm_led)),                     \
		.init_color = {.raw = DT_INST_PROP(id, init_rgba)},                                \
	};                                                                                         \
	static struct rgb_led_pwm_data rgb_led_pwm_data_##id;                                      \
	DEVICE_DT_INST_DEFINE(id, &rgb_led_pwm_init, NULL, &rgb_led_pwm_data_##id,                 \
			      &rgb_led_pwm_config_##id, APPLICATION, CONFIG_LED_INIT_PRIORITY,     \
			      &rgb_led_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(RGB_LED_PWM_DEVICE)
