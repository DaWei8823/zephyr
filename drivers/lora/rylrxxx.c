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
#include <errno.h>

LOG_MODULE_REGISTER(LYLR, CONFIG_LORA_LOG_LEVEL);

#define RYLR_CMD_PREFIX          "AT"
#define RYLR_CMD_TERMINATOR      "\r\n"
#define RYLR_RESPONSE_TERMINATOR "\r\n"
#define RYLR_CMD_TEST            (RYLR_CMD_PREFIX "\r\n")
#define RYLR_CMD_TEST_LEN        sizeof(RYLR_CMD_TEST) - 1
#define RYLR_CMD_TEST_RESP       "+OK"

#define RYLR_MAX_REPONSE 256

struct rylr_rx_msgq_element {
	char data[CONFIG_LORA_RYLRXX_RX_QUEUE_WIDTH];
	uint32_t size;
};

struct rylr_config {
	const struct device *uart;
};

struct rylr_data {
	struct k_msgq *const rx_queue;
	struct rylr_rx_msgq_element rx_curr;
	uint32_t uart_callback_last_status;
	struct k_mutex *rylr_mutex;
};

static int rylr_trx_sync(const struct device *uart, char *tx, uint32_t tx_size, char *rx,
			 uint32_t *rx_size)
{
	int err = 0;
	while (tx_size--) {
		uart_poll_out(uart, *tx++);
	}

	*rx_size = 0;
	while (*rx_size < RYLR_MAX_REPONSE) {
		err = uart_poll_in(uart, rx + (*rx_size)++);
		if (err != 0) {
			LOG_ERR("error polling rylr %d", err);
			return err;
		}

		if (*rx_size > 1 && rx[*rx_size - 1] == RYLR_RESPONSE_TERMINATOR[1] &&
		    rx[*rx_size - 2] == RYLR_RESPONSE_TERMINATOR[0]) {
			if (*rx_size != RYLR_MAX_REPONSE) {
				rx[*rx_size] = '\0';
			}
			return err;
		}
	}

	// TODO: add null term and make buffers big enough for max response case
	return err;
}

static int rylr_init(const struct device *dev)
{
	const struct rylr_config *config = dev->config;
	struct rylr_data *data = dev->data;
	int err = 0;

	if (!device_is_ready(config->uart)) {
		LOG_ERR("uart bus not ready");
		return -ENODEV;
	}

	err = rylr_trx_sync(config->uart, RYLR_CMD_TEST, RYLR_CMD_TEST_LEN, data->rx_curr.data,
			    &data->rx_curr.size);

	if (err != 0) {
		LOG_ERR("error trx test cmd. err=%d", err);
		return err;
	}

	return err;
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
	K_MSGQ_DEFINE(rx_message_queue##inst, sizeof(struct rylr_rx_msgq_element),                 \
		      CONFIG_LORA_RYLRXX_RX_QUEUE_DEPTH, 1);                                       \
	K_MUTEX_DEFINE(rylr_mutex_##inst);                                                         \
	static struct rylr_data rylr_data_##inst = {                                               \
		.rx_queue = &rx_message_queue##inst,                                               \
		.rylr_mutex = &rylr_mutex_##inst,                                                  \
	};                                                                                         \
                                                                                                   \
	static const struct rylr_config rylr_config_##inst = {                                     \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &rylr_init, NULL, &rylr_data_##inst, &rylr_config_##inst,      \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY, &rylr_lora_api);

DT_INST_FOREACH_STATUS_OKAY(RYLR_DEFINE)
