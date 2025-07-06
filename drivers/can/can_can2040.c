/*
 * This file contains glue code licensed under the terms described below.
 * The original implementation (can_loopback.c from Zephyr project) is
 * licensed under the Apache License 2.0.
 *
 * For the license and authorship of other components like can2040,
 * please refer to their respective files.
 *
 * Please note that when this file is linked with the can2040 library,
 * the resulting binary will be licensed under GPL-3.0-only.
 * Apache-2.0 license applies to this code only, for example, when
 * it is copied or used as a reference in other projects.
 *
 * Copyright (c) 2025 Dmitrii Sharshakov <d3dx12.xx@gmail.com>
 *
 * Parts based on can_loopback.c, which is Apache-2.0 licensed:
 * Copyright (c) 2021 Vestas Wind Systems A/S
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: GPL-3.0-only OR Apache-2.0
 */

#define DT_DRV_COMPAT can2040_can2040

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include "can2040_ll.h"

LOG_MODULE_REGISTER(can_can2040, CONFIG_CAN_LOG_LEVEL);

struct can_can2040_filter {
	can_rx_callback_t rx_cb;
	void *cb_arg;
	struct can_filter filter;
};

struct can_can2040_config {
	const struct can_driver_config common;
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	const uint32_t tx_gpio;
	const uint32_t rx_gpio;
	const uint32_t bitrate;
	const uint32_t clk_freq;
	void (*irq_connect_func)(const struct device *dev);
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
};

struct can_can2040_data {
	struct can2040 can2040;

	struct can_driver_data common;
	struct can_can2040_filter filters[CONFIG_CAN_MAX_FILTER];
	struct k_mutex filters_mtx;

	can_tx_callback_t tx_cb;
	void *tx_cb_arg;
	struct k_sem tx_idle;
};

static int can_can2040_send(const struct device *dev, const struct can_frame *frame,
			    k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	struct can_can2040_data *data = dev->data;
	int ret;

	if (k_sem_take(&data->tx_idle, timeout) != 0) {
		return -EAGAIN;
	}

	LOG_DBG("Sending %d bytes on %s. Id: 0x%x, ID type: %s %s", frame->dlc, dev->name,
		frame->id, (frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
		return -EINVAL;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	struct can2040_msg tx;
	tx.dlc = frame->dlc;
	tx.id = frame->id;
	tx.id |= (frame->flags & CAN_FRAME_IDE) ? CAN2040_ID_EFF : 0;
	tx.id |= (frame->flags & CAN_FRAME_RTR) ? CAN2040_ID_RTR : 0;
	LOG_DBG("TX frame: id=0x%x, dlc=%d, flags=0x%02x", tx.id, tx.dlc, frame->flags);
	memcpy(tx.data, frame->data, frame->dlc);

	data->tx_cb = callback;
	data->tx_cb_arg = user_data;

	ret = can2040_transmit(&data->can2040, &tx);
	if (ret < 0) {
		LOG_DBG("TX queue full (err %d)", ret);
		return -EAGAIN;
	}

	return 0;
}

static inline int get_free_filter(struct can_can2040_filter *filters)
{
	for (int i = 0; i < CONFIG_CAN_MAX_FILTER; i++) {
		if (filters[i].rx_cb == NULL) {
			return i;
		}
	}

	return -ENOSPC;
}

static int can_can2040_add_rx_filter(const struct device *dev, can_rx_callback_t cb, void *cb_arg,
				     const struct can_filter *filter)
{
	struct can_can2040_data *data = dev->data;
	struct can_can2040_filter *filter_entry;
	int filter_id;

	LOG_DBG("Adding filter ID: 0x%x, mask: 0x%x", filter->id, filter->mask);

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->filters_mtx, K_FOREVER);
	filter_id = get_free_filter(data->filters);

	if (filter_id < 0) {
		LOG_ERR("No free filter left");
		k_mutex_unlock(&data->filters_mtx);
		return filter_id;
	}

	filter_entry = &data->filters[filter_id];

	filter_entry->cb_arg = cb_arg;
	filter_entry->filter = *filter;
	// Add callback the last, so we don't call it before the filter is fully set up
	// Do this because we cannot lock mutex in ISR
	filter_entry->rx_cb = cb;
	k_mutex_unlock(&data->filters_mtx);

	LOG_DBG("Filter added. ID: %d", filter_id);

	return filter_id;
}

static void can_can2040_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_can2040_data *data = dev->data;

	if (filter_id < 0 || filter_id >= ARRAY_SIZE(data->filters)) {
		LOG_ERR("filter ID %d out-of-bounds", filter_id);
		return;
	}

	LOG_DBG("Remove filter ID: %d", filter_id);
	k_mutex_lock(&data->filters_mtx, K_FOREVER);
	data->filters[filter_id].rx_cb = NULL;
	k_mutex_unlock(&data->filters_mtx);
}

static int can_can2040_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	// TODO: CAN_MODE_LISTENONLY, CAN_MODE_ONE_SHOT, CAN_MODE_3_SAMPLES?
	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK;

	return 0;
}

static void can_can2040_tx_done(const struct device *dev, int status)
{
	struct can_can2040_data *data = dev->data;
	can_tx_callback_t callback = data->tx_cb;
	void *user_data = data->tx_cb_arg;

	if (callback != NULL) {
		data->tx_cb = NULL;
		callback(dev, status, user_data);
	}
	k_sem_give(&data->tx_idle);
}

static void can_can2040_handle_rx(const struct device *dev, struct can2040_msg *msg)
{
	struct can_can2040_data *data = dev->data;

	struct can_frame frame = {
		.id = msg->id & CAN_EXT_ID_MASK,
		.dlc = msg->dlc,
		.flags = ((msg->id & CAN2040_ID_EFF) ? CAN_FRAME_IDE : 0) |
			 ((msg->id & CAN2040_ID_RTR) ? CAN_FRAME_RTR : 0),
	};
	memcpy(frame.data, msg->data, frame.dlc);

	LOG_DBG("Received frame: id=0x%x, dlc=%d, flags=0x%02x", frame.id, frame.dlc, frame.flags);

	struct can_can2040_filter *filter;
	// SAFETY: cannot lock mutex in ISR
	for (int i = 0; i < CONFIG_CAN_MAX_FILTER; i++) {
		filter = &data->filters[i];
		if (filter->rx_cb != NULL && can_frame_matches_filter(&frame, &filter->filter)) {
			LOG_DBG("rx->receive");
			filter->rx_cb(dev, &frame, filter->cb_arg);
		}
	}
}

// Called from an ISR
static void can_can2040_callback(struct can2040 *can2040_dev, uint32_t notify,
				 struct can2040_msg *msg)
{
	const struct device *dev = can2040_dev->rx_cb_user_data;
	struct can_can2040_data *data = dev->data;

	if (notify & CAN2040_NOTIFY_RX) {
		can_can2040_handle_rx(dev, msg);
	} else if (notify & CAN2040_NOTIFY_TX) {
		LOG_DBG("TX complete: id=0x%x, dlc=%d", msg->id, msg->dlc);
		can_can2040_tx_done(dev, 0);

		if (data->common.mode & CAN_MODE_LOOPBACK) {
			can_can2040_handle_rx(dev, msg);
		}
	} else if (notify & CAN2040_NOTIFY_ERROR) {
		LOG_WRN("Bus error notification received, some received frames may be lost");
	}
}

static void can_can2040_pio_irq_handler(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;
	can2040_pio_irq_handler(&data->can2040);
}

static int can_can2040_start(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;
	const struct can_can2040_config *config = dev->config;

	if (data->common.started) {
		return -EALREADY;
	}

	memset(&data->can2040, 0, sizeof(data->can2040));
	data->can2040.pio_hw = pio_rpi_pico_get_pio(config->piodev);
	data->can2040.rx_cb_user_data = (void *)dev;
	can2040_callback_config(&data->can2040, can_can2040_callback);

	CAN_STATS_RESET(dev);

	config->irq_enable_func(dev);

	LOG_DBG("Enable device %s, clock: %u", dev->name, config->clk_freq);

	can2040_ll_data_state_clear_bits(&data->can2040);
	data->can2040.gpio_rx = config->rx_gpio;
	data->can2040.gpio_tx = config->tx_gpio;
	can2040_ll_pio_set_clkdiv(&data->can2040, config->clk_freq, config->bitrate);
	can2040_ll_pio_sm_setup(&data->can2040);
	can2040_ll_data_state_go_discard(&data->can2040);

	data->common.started = true;

	return pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
}

static int can_can2040_stop(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;
	const struct can_can2040_config *config = dev->config;

	if (!data->common.started) {
		return -EALREADY;
	}

	data->common.started = false;

	can_can2040_tx_done(dev, -ENETDOWN);
	config->irq_disable_func(dev);
	can2040_ll_pio_sm_setup(&data->can2040);

	return 0;
}

static int can_can2040_set_mode(const struct device *dev, can_mode_t mode)
{
	struct can_can2040_data *data = dev->data;

	if (data->common.started) {
		LOG_ERR("Cannot set mode while CAN is running");
		return -EBUSY;
	}

	if (mode & ~(CAN_MODE_NORMAL | CAN_MODE_LOOPBACK)) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	data->common.mode = mode;

	return 0;
}

static int can_can2040_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct can_can2040_data *data = dev->data;

	ARG_UNUSED(timing);

	if (data->common.started) {
		return -EBUSY;
	}

	return 0;
}

static int can_can2040_get_state(const struct device *dev, enum can_state *state,
				 struct can_bus_err_cnt *err_cnt)
{
	struct can_can2040_data *data = dev->data;

	if (state != NULL) {
		if (data->common.started) {
			*state = CAN_STATE_ERROR_ACTIVE;
		} else {
			*state = CAN_STATE_STOPPED;
		}
	}

	if (err_cnt) {
		err_cnt->tx_err_cnt = 0;
		err_cnt->rx_err_cnt = 0;
	}

	return 0;
}

static void can_can2040_set_state_change_callback(const struct device *dev,
						  can_state_change_callback_t cb, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
}

static int can_can2040_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

	/* Recommended CAN clock from CiA 601-3 */
	*rate = MHZ(80);

	return 0;
}

static int can_can2040_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	return CONFIG_CAN_MAX_FILTER;
}

static DEVICE_API(can, can_can2040_driver_api) = {
	.get_capabilities = can_can2040_get_capabilities,
	.start = can_can2040_start,
	.stop = can_can2040_stop,
	.set_mode = can_can2040_set_mode,
	.set_timing = can_can2040_set_timing,
	.send = can_can2040_send,
	.add_rx_filter = can_can2040_add_rx_filter,
	.remove_rx_filter = can_can2040_remove_rx_filter,
	.get_state = can_can2040_get_state,
	.set_state_change_callback = can_can2040_set_state_change_callback,
	.get_core_clock = can_can2040_get_core_clock,
	.get_max_filters = can_can2040_get_max_filters,
	/* Recommended configuration ranges from CiA 601-2 */
	.timing_min = {.sjw = 1, .prop_seg = 0, .phase_seg1 = 2, .phase_seg2 = 2, .prescaler = 1},
	.timing_max =
		{.sjw = 128, .prop_seg = 0, .phase_seg1 = 256, .phase_seg2 = 128, .prescaler = 32},
};

static int can_can2040_init(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;
	const struct can_can2040_config *config = dev->config;

	k_mutex_init(&data->filters_mtx);
	k_sem_init(&data->tx_idle, 1, 1);

	for (int i = 0; i < CONFIG_CAN_MAX_FILTER; i++) {
		data->filters[i].rx_cb = NULL;
	}

	config->irq_connect_func(dev);

	return 0;
}

#define CAN_CAN2040_INIT(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void can_can2040_irq_connect_func_##n(const struct device *dev)                     \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST_PARENT(inst), irq0, irq),                       \
			    DT_IRQ_BY_NAME(DT_INST_PARENT(inst), irq0, priority),                  \
			    can_can2040_pio_irq_handler, DEVICE_DT_INST_GET(inst), 0);             \
	}                                                                                          \
                                                                                                   \
	static void can_can2040_irq_enable_func_##n(const struct device *dev)                      \
	{                                                                                          \
		irq_enable(DT_IRQ_BY_NAME(DT_INST_PARENT(inst), irq0, irq));                       \
	}                                                                                          \
                                                                                                   \
	static void can_can2040_irq_disable_func_##n(const struct device *dev)                     \
	{                                                                                          \
		irq_disable(DT_IRQ_BY_NAME(DT_INST_PARENT(inst), irq0, irq));                      \
	}                                                                                          \
                                                                                                   \
	static const struct can_can2040_config can_can2040_config_##inst = {                       \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, DT_INST_PROP(inst, bitrate),         \
							DT_INST_PROP(inst, bitrate)),              \
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.tx_gpio = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, tx_pins, 0),         \
		.rx_gpio = DT_INST_RPI_PICO_PIO_PIN_BY_NAME(inst, default, 0, rx_pins, 0),         \
		.bitrate = DT_INST_PROP(inst, bitrate),                                            \
		.clk_freq = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, sys), clock_frequency),         \
		.irq_connect_func = can_can2040_irq_connect_func_##n,                              \
		.irq_enable_func = can_can2040_irq_enable_func_##n,                                \
		.irq_disable_func = can_can2040_irq_disable_func_##n,                              \
	};                                                                                         \
                                                                                                   \
	static struct can_can2040_data can_can2040_data_##inst;                                    \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_can2040_init, NULL, &can_can2040_data_##inst,          \
				  &can_can2040_config_##inst, POST_KERNEL,                         \
				  CONFIG_CAN_INIT_PRIORITY, &can_can2040_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_CAN2040_INIT)
