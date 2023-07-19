/*
 * (c) Meta Platforms, Inc. and affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT nxp_ctimer_pwm

#include <errno.h>
#include <fsl_ctimer.h>
#include <fsl_clock.h>
#include <zephyr/drivers/pwm.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_mcux_ctimer, CONFIG_PWM_LOG_LEVEL);

#define CHANNEL_COUNT kCTIMER_Match_3 + 1

#define SECURE_NONSECURE_MSK            (0xF0000000U)
#define SECURE_NONSECURE_SHIFT          (28U)
#define PERIPHERAL_BASE_SECURE_ALIAS    (0x05U << SECURE_NONSECURE_SHIFT)
#define PERIPHERAL_BASE_NONSECURE_ALIAS (0x04U << SECURE_NONSECURE_SHIFT)
#define PERIPHERAL_BASE		(DT_RANGES_PARENT_BUS_ADDRESS_BY_IDX(DT_NODELABEL(peripheral), 0))

#if (PERIPHERAL_BASE & SECURE_NONSECURE_MSK) == PERIPHERAL_BASE_SECURE_ALIAS
#define SECURITY_ALIAS PERIPHERAL_BASE_SECURE_ALIAS
#elif (PERIPHERAL_BASE & SECURE_NONSECURE_MSK) == PERIPHERAL_BASE_NONSECURE_ALIAS
#define SECURITY_ALIAS PERIPHERAL_BASE_NONSECURE_ALIAS
#else
#error "cannot determine secure or nonsecure access to peripherals"
#endif

#define ADD_SECURITY_ALIAS(addr)     ((addr & (~SECURE_NONSECURE_MSK)) | SECURITY_ALIAS)

struct pwm_mcux_ctimer_config {
	CTIMER_Type *base;
	uint32_t prescale;
	uint32_t period_channel;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif
};

static int _mcux_ctimer_get_ctimer_id(CTIMER_Type *base, uint8_t *ret_id)
{
	static uint32_t const ctimers[] = CTIMER_BASE_ADDRS;
	uint8_t id = 0;

	while (id < ARRAY_SIZE(ctimers)) {
		if (ADD_SECURITY_ALIAS(ctimers[id]) == (uint32_t)base) {
			*ret_id = id;
			return 0;
		}
		id++;
	}
	return -EINVAL;
}

static int mcux_ctimer_pwm_set_cycles(const struct device *dev,
				       uint32_t pulse_channel, uint32_t period_cycles,
				       uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_mcux_ctimer_config *config = dev->config;
	status_t status;

	if (pulse_channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel %u. muse be less than %u", pulse_channel, CHANNEL_COUNT);
		return -EINVAL;
	}

	if (period_cycles == 0) {
		LOG_ERR("Channel can not be set to zero");
		return -ENOTSUP;
	}

	if (config->period_channel == pulse_channel) {
		LOG_ERR("pulse_channel %u cannot be same as period_channel", pulse_channel);
		return -EINVAL;
	}

	if (flags & PWM_POLARITY_INVERTED) {
		if (pulse_cycles == 0) {
			/* make pulse cycles greater than period so event never occurs*/
			pulse_cycles = period_cycles + 1;
		} else {
			pulse_cycles = period_cycles - pulse_cycles;
		}
	}

	status = CTIMER_SetupPwmPeriod(config->base, config->period_channel,
		pulse_channel, period_cycles, pulse_cycles, false);
	if (kStatus_Success != status) {
		LOG_ERR("failed setup pwm period. status=%d", status);
		return -EIO;
	}

	CTIMER_StartTimer(config->base);
	return 0;
}

static int mcux_ctimer_pwm_get_cycles_per_sec(const struct device *dev,
					       uint32_t channel,
					       uint64_t *cycles)
{
	const struct pwm_mcux_ctimer_config *config = dev->config;
	uint8_t ctimer_id = 0;
	int ret = 0;

	ret = _mcux_ctimer_get_ctimer_id(config->base, &ctimer_id);
	if (ret != 0) {
		LOG_ERR("could not get ctimer id for %p", config->base);
		return ret;
	}
	*cycles = CLOCK_GetCtimerClkFreq(ctimer_id)/(config->prescale > 0 ? config->prescale : 1);

	return ret;
}

static int mcux_ctimer_pwm_init(const struct device *dev)
{
	const struct pwm_mcux_ctimer_config *config = dev->config;
	ctimer_config_t pwm_config;
#ifdef CONFIG_PINCTRL
	int err;

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}
#endif

	if (config->period_channel >= CHANNEL_COUNT) {
		LOG_ERR("invalid period_channel: %d. must be less than %d",
			config->period_channel,
			CHANNEL_COUNT);
		return -EINVAL;
	}

	CTIMER_GetDefaultConfig(&pwm_config);
	pwm_config.prescale = config->prescale;

	CTIMER_Init(config->base, &pwm_config);
	return 0;
}

static const struct pwm_driver_api pwm_mcux_ctimer_driver_api = {
	.set_cycles = mcux_ctimer_pwm_set_cycles,
	.get_cycles_per_sec = mcux_ctimer_pwm_get_cycles_per_sec,
};

#ifdef CONFIG_PINCTRL
#define PWM_MCUX_CTIMER_PINCTRL_DEFINE(n) PINCTRL_DT_INST_DEFINE(n);
#define PWM_MCUX_CTIMER_PINCTRL_INIT(n) .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),
#else
#define PWM_MCUX_CTIMER_PINCTRL_DEFINE(n)
#define PWM_MCUX_CTIMER_PINCTRL_INIT(n)
#endif

#define PWM_MCUX_CTIMER_DEVICE_INIT_MCUX(n)						\
	PWM_MCUX_CTIMER_PINCTRL_DEFINE(n)						\
	static const struct pwm_mcux_ctimer_config pwm_mcux_ctimer_config_##n = {	\
		.base = (CTIMER_Type *)DT_INST_REG_ADDR(n),				\
		.prescale = DT_INST_PROP(n, prescaler),					\
		.period_channel = DT_INST_PROP(n, period_channel),      \
		PWM_MCUX_CTIMER_PINCTRL_INIT(n)					\
	};										\
											\
	DEVICE_DT_INST_DEFINE(n,							\
			      mcux_ctimer_pwm_init,					\
			      NULL,							\
			      NULL,				\
			      &pwm_mcux_ctimer_config_##n,				\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &pwm_mcux_ctimer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_MCUX_CTIMER_DEVICE_INIT_MCUX)
