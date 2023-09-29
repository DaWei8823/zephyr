/*
 * Copyright (C) 2023 David Ullmann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT reyax_rylrxxx

#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>

LOG_MODULE_REGISTER(LYLR, CONFIG_LORA_LOG_LEVEL);

struct rylr_config {
	const struct device *uart;
};

struct rylr_data {
	struct k_msgq *const rx_queue;
};

static int rylr_init(const struct device *dev)
{
    return 0;
}

static const struct lora_driver_api rylr_lora_api = {
	.config = NULL,
	.send = NULL,
	.send_async = NULL,
	.recv = NULL,
	.recv_async = NULL,
	.test_cw = NULL,
};

#define RYLR_DEFINE(inst)                                                                          \
	K_MSGQ_DEFINE(rx_message_queue##inst, CONFIG_LORA_RYLRXX_RX_QUEUE_WIDTH,                   \
		      CONFIG_LORA_RYLRXX_RX_QUEUE_DEPTH, 1);                                       \
	static struct rylr_data rylr_data_##inst = {.rx_queue = &rx_message_queue##inst};          \
                                                                                                   \
	static const struct rylr_config rylr_config_##inst = {                                     \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &rylr_init, NULL, &rylr_data_##inst, &rylr_config_##inst,      \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY, &rylr_lora_api);

DT_INST_FOREACH_STATUS_OKAY(RYLR_DEFINE)
