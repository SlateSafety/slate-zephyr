/**
 * @file
 * @brief SiWx91x NCP NWP driver over SPI.
 *
 * This file implements the NWP (Network Wireless Processor) driver for the
 * SiWx91x in NCP (Network Co-Processor) mode. The SiWx91x module is
 * connected to the host MCU via SPI with GPIO signals for reset and
 * interrupt handshaking.
 *
 * This driver:
 * 1. Implements sl_si91x_host_interface.h (platform abstraction for SPI/GPIO)
 * 2. Performs NWP device initialization (reset, boot, sl_wifi_init)
 * 3. Exports siwx91x_nwp_mode_switch() and country code helpers for the
 *    bus-agnostic WiFi driver (siwx91x_wifi.c)
 *
 * Copyright (c) 2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_siwx91x_nwp_spi

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/wifi.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "siwx91x_nwp.h"
#include "nwp_fw_version.h"
#include "sl_wifi_callback_framework.h"
#include "sl_si91x_host_interface.h"
#include "sl_si91x_driver.h"

#if defined(CONFIG_BT_SILABS_SIWX91X) || defined(CONFIG_SIWX91X_NCP_BLE_RF_TEST)
#include "rsi_ble_common_config.h"
#include "rsi_ble_apis.h"
#include "rsi_bt_common_apis.h"
#include "sl_si91x_ble.h"
#endif

LOG_MODULE_REGISTER(siwx91x_nwp_ncp, 4);

/* ========================================================================== */
/* DT-driven hardware configuration                                           */
/* ========================================================================== */

#define SIWX91X_NWP_NCP_INST(inst)  DT_DRV_INST(inst)

struct siwx91x_ncp_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec irq_gpio;
	struct gpio_dt_spec cs_gpio;
#if DT_INST_NODE_HAS_PROP(0, sleep_request_gpios)
	struct gpio_dt_spec sleep_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, wake_indicator_gpios)
	struct gpio_dt_spec wake_gpio;
#endif
};

static const struct device *gpiog_dev = DEVICE_DT_GET(DT_NODELABEL(gpiog));
static const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define LOAD_SW_PIN 11

struct siwx91x_ncp_data {
	char current_country_code[WIFI_COUNTRY_CODE_LEN];
	sl_si91x_host_rx_irq_handler rx_irq_handler;
	struct gpio_callback irq_cb_data;
	struct spi_config spi_cfg;
	volatile bool bus_irq_enabled;
	sl_wifi_transmitter_test_info_t wifi_tx_test_cfg;
	bool wifi_tx_test_running;
#if defined(CONFIG_BT_SILABS_SIWX91X) || defined(CONFIG_SIWX91X_NCP_BLE_RF_TEST)
	rsi_ble_per_transmit_t ble_per_tx_cfg;
	bool ble_per_tx_test_running;
	bool ble_stack_initialized;
	bool ble_radio_disabled;
#endif
};

/* Singleton device pointer used by both host interface callbacks and shell CLI. */
static const struct device *ncp_dev_instance;

static void siwx91x_wifi_tx_test_set_defaults(sl_wifi_transmitter_test_info_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->enable = 1;
	cfg->power = 18;
	cfg->rate = 0;
	cfg->length = 200;
	cfg->mode = 1;
	cfg->channel = 11;
}

static int siwx91x_wifi_freq_mhz_to_channel(uint16_t freq_mhz, uint16_t *channel)
{
	if (freq_mhz < 2412 || freq_mhz > 2472) {
		return -EINVAL;
	}

	if (((freq_mhz - 2407U) % 5U) != 0U) {
		return -EINVAL;
	}

	*channel = (uint16_t)((freq_mhz - 2407U) / 5U);
	if (*channel < 1U || *channel > 13U) {
		return -EINVAL;
	}

	return 0;
}

static bool siwx91x_wifi_tx_length_is_valid(const sl_wifi_transmitter_test_info_t *cfg)
{
	if (cfg->mode == 1U) {
		return (cfg->length >= 24U) && (cfg->length <= 260U);
	}

	return (cfg->length >= 24U) && (cfg->length <= 1500U);
}

static int siwx91x_wifi_tx_test_start_internal(const struct device *dev)
{
	struct siwx91x_ncp_data *data = dev->data;
	sl_status_t status;

	if (!siwx91x_wifi_tx_length_is_valid(&data->wifi_tx_test_cfg)) {
		LOG_ERR("Invalid length %u for mode %u", data->wifi_tx_test_cfg.length,
			data->wifi_tx_test_cfg.mode);
		return -EINVAL;
	}

	if (data->wifi_tx_test_running) {
		status = sl_wifi_transmit_test_stop(SL_WIFI_CLIENT_INTERFACE);
		if (status != SL_STATUS_OK) {
			LOG_ERR("Failed to stop active WiFi TX test: 0x%x", status);
			return -EIO;
		}
		data->wifi_tx_test_running = false;
		k_msleep(10);
	}

	data->wifi_tx_test_cfg.enable = 1;
	status = sl_wifi_transmit_test_start(SL_WIFI_CLIENT_INTERFACE, &data->wifi_tx_test_cfg);
	if (status != SL_STATUS_OK) {
		LOG_ERR("Failed to start WiFi TX test: 0x%x", status);
		return -EIO;
	}

	data->wifi_tx_test_running = true;
	LOG_INF("WiFi TX test started: ch=%u power=%u rate=%u mode=%u len=%u",
		data->wifi_tx_test_cfg.channel, data->wifi_tx_test_cfg.power,
		data->wifi_tx_test_cfg.rate, data->wifi_tx_test_cfg.mode,
		data->wifi_tx_test_cfg.length);

	return 0;
}

static int siwx91x_wifi_tx_test_stop_internal(const struct device *dev)
{
	struct siwx91x_ncp_data *data = dev->data;
	sl_status_t status;

	if (!data->wifi_tx_test_running) {
		return 0;
	}

	status = sl_wifi_transmit_test_stop(SL_WIFI_CLIENT_INTERFACE);
	if (status != SL_STATUS_OK) {
		LOG_ERR("Failed to stop WiFi TX test: 0x%x", status);
		return -EIO;
	}

	data->wifi_tx_test_running = false;
	LOG_INF("WiFi TX test stopped");
	return 0;
}

#if defined(CONFIG_BT_SILABS_SIWX91X) || defined(CONFIG_SIWX91X_NCP_BLE_RF_TEST)
static uint16_t siwx91x_ble_per_get_pkt_len(const rsi_ble_per_transmit_t *cfg)
{
	return (uint16_t)cfg->pkt_len[0] | ((uint16_t)cfg->pkt_len[1] << 8);
}

static void siwx91x_ble_per_set_pkt_len(rsi_ble_per_transmit_t *cfg, uint16_t pkt_len)
{
	cfg->pkt_len[0] = (uint8_t)(pkt_len & 0xFFU);
	cfg->pkt_len[1] = (uint8_t)((pkt_len >> 8) & 0xFFU);
}

static uint32_t siwx91x_ble_per_get_num_pkts(const rsi_ble_per_transmit_t *cfg)
{
	return (uint32_t)cfg->num_pkts[0] | ((uint32_t)cfg->num_pkts[1] << 8) |
	       ((uint32_t)cfg->num_pkts[2] << 16) | ((uint32_t)cfg->num_pkts[3] << 24);
}

static uint16_t siwx91x_ble_channel_to_freq_mhz(uint8_t ch)
{
	if (ch <= 10U) {
		return (uint16_t)(2404U + (2U * ch));
	}

	if (ch <= 36U) {
		return (uint16_t)(2406U + (2U * ch));
	}

	if (ch == 37U) {
		return 2402U;
	}

	if (ch == 38U) {
		return 2426U;
	}

	if (ch == 39U) {
		return 2480U;
	}

	return 0U;
}

static void siwx91x_ble_per_tx_test_set_defaults(rsi_ble_per_transmit_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->cmd_ix = HCI_BLE_TRANSMIT_CMD_ID;
	cfg->transmit_enable = 1;
	cfg->access_addr[0] = 0x8E;
	cfg->access_addr[1] = 0x89;
	cfg->access_addr[2] = 0xBE;
	cfg->access_addr[3] = 0xD6;
	cfg->phy_rate = 1;
	cfg->rx_chnl_num = 0;
	cfg->tx_chnl_num = 0;
	siwx91x_ble_per_set_pkt_len(cfg, 37);
	cfg->payload_type = 0;
	cfg->tx_power = 31;
	cfg->transmit_mode = 1;
	cfg->scrambler_seed = 5;
	cfg->le_chnl_type = 1;
	cfg->freq_hop_en = 0;
	cfg->ant_sel = 2;
	cfg->pll_mode = 0;
	cfg->rf_type = 1;
	cfg->rf_chain = 2;
	cfg->inter_pkt_gap = 0;
}

static bool siwx91x_ble_per_tx_cfg_is_valid(const rsi_ble_per_transmit_t *cfg)
{
	const uint16_t pkt_len = siwx91x_ble_per_get_pkt_len(cfg);

	if (cfg->tx_chnl_num > 39U || cfg->rx_chnl_num > 39U) {
		return false;
	}

	if (!(cfg->phy_rate == 1U || cfg->phy_rate == 2U || cfg->phy_rate == 4U ||
	      cfg->phy_rate == 8U)) {
		return false;
	}

	if (cfg->tx_power == 0U || cfg->tx_power == 32U) {
		return false;
	}

	if (cfg->payload_type > 7U || cfg->transmit_mode > 2U) {
		return false;
	}

	if (pkt_len == 0U || pkt_len > 255U) {
		return false;
	}

	return true;
}

static void siwx91x_ble_per_build_minimal_cmd(const struct siwx91x_ncp_data *data,
						       rsi_ble_per_transmit_t *per_tx,
						       bool enable)
{
	memset(per_tx, 0, sizeof(*per_tx));

	per_tx->cmd_ix = HCI_BLE_TRANSMIT_CMD_ID;
	per_tx->transmit_enable = enable ? 1U : 0U;

	if (!enable) {
		return;
	}

	/* Keep only the core TX knobs to avoid over-constraining firmware defaults. */
	per_tx->access_addr[0] = data->ble_per_tx_cfg.access_addr[0];
	per_tx->access_addr[1] = data->ble_per_tx_cfg.access_addr[1];
	per_tx->access_addr[2] = data->ble_per_tx_cfg.access_addr[2];
	per_tx->access_addr[3] = data->ble_per_tx_cfg.access_addr[3];
	per_tx->phy_rate = data->ble_per_tx_cfg.phy_rate;
	per_tx->rx_chnl_num = data->ble_per_tx_cfg.rx_chnl_num;
	per_tx->tx_chnl_num = data->ble_per_tx_cfg.tx_chnl_num;
	per_tx->tx_power = data->ble_per_tx_cfg.tx_power;
	per_tx->transmit_mode = data->ble_per_tx_cfg.transmit_mode;
	per_tx->payload_type = data->ble_per_tx_cfg.payload_type;
	per_tx->inter_pkt_gap = data->ble_per_tx_cfg.inter_pkt_gap;
	per_tx->pkt_len[0] = data->ble_per_tx_cfg.pkt_len[0];
	per_tx->pkt_len[1] = data->ble_per_tx_cfg.pkt_len[1];
	per_tx->le_chnl_type = data->ble_per_tx_cfg.le_chnl_type;
	per_tx->freq_hop_en = data->ble_per_tx_cfg.freq_hop_en;
	per_tx->ant_sel = data->ble_per_tx_cfg.ant_sel;
	per_tx->pll_mode = data->ble_per_tx_cfg.pll_mode;
	per_tx->rf_type = data->ble_per_tx_cfg.rf_type;
	per_tx->rf_chain = data->ble_per_tx_cfg.rf_chain;

	if (per_tx->transmit_mode == 1U) {
		per_tx->scrambler_seed = 5U;
	}
}

static int siwx91x_ble_per_tx_test_start_internal(const struct device *dev)
{
	struct siwx91x_ncp_data *data = dev->data;
	rsi_ble_per_transmit_t per_tx;
	int32_t status;
	sl_status_t sl_status;
	int attempt;

	if (!data->ble_radio_disabled) {
		/* Match WiseConnect BLE test examples: disable radio before BLE test commands. */
		sl_status = SL_STATUS_TIMEOUT;
		for (attempt = 0; attempt < 3; attempt++) {
			sl_status = sl_si91x_disable_radio();
			if (sl_status == SL_STATUS_OK ||
			    sl_status == SL_STATUS_SI91X_COMMAND_GIVEN_IN_INVALID_STATE) {
				break;
			}

			if (sl_status != SL_STATUS_TIMEOUT) {
				LOG_ERR("Failed to disable radio for BLE test: 0x%x", (unsigned int)sl_status);
				return (int)sl_status;
			}

			k_msleep(20);
		}

		if (sl_status != SL_STATUS_OK &&
		    sl_status != SL_STATUS_SI91X_COMMAND_GIVEN_IN_INVALID_STATE) {
			LOG_ERR("Failed to disable radio for BLE test after retries: 0x%x",
				(unsigned int)sl_status);
			return (int)sl_status;
		}

		data->ble_radio_disabled = true;
	}

	if (!data->ble_stack_initialized) {
		status = rsi_bt_init();
		if (status != 0 && status != RSI_ERROR_COMMAND_GIVEN_IN_WRONG_STATE) {
			LOG_ERR("BLE stack init failed: %d", (int)status);
			return (int)status;
		}

		data->ble_stack_initialized = true;
	}

	if (!siwx91x_ble_per_tx_cfg_is_valid(&data->ble_per_tx_cfg)) {
		LOG_ERR("Invalid BLE PER TX configuration");
		return -EINVAL;
	}

	if (data->ble_per_tx_test_running) {
		siwx91x_ble_per_build_minimal_cmd(data, &per_tx, false);
		status = rsi_ble_per_transmit(&per_tx);
		if (status != 0) {
			LOG_ERR("Failed to stop active BLE PER TX: %d", (int)status);
			return (int)status;
		}
		data->ble_per_tx_test_running = false;
		k_msleep(10);
	}

	siwx91x_ble_per_build_minimal_cmd(data, &per_tx, true);
	status = RSI_ERROR_RESPONSE_TIMEOUT;
	for (attempt = 0; attempt < 3; attempt++) {
		status = rsi_ble_per_transmit(&per_tx);
		if (status != (int32_t)SL_STATUS_TIMEOUT) {
			break;
		}

		k_msleep(20);
	}
	if (status != 0) {
		LOG_ERR("Failed to start BLE PER TX: %d (cmd_ix=%u ch=%u phy=%u pwr=%u mode=%u len=%u payload=%u)",
			(int)status, per_tx.cmd_ix, per_tx.tx_chnl_num, per_tx.phy_rate,
			per_tx.tx_power, per_tx.transmit_mode,
			siwx91x_ble_per_get_pkt_len(&per_tx), per_tx.payload_type);
		return (int)status;
	}

	data->ble_per_tx_test_running = true;
	LOG_INF("BLE PER TX started: ch=%u (%u MHz) phy=%u pwr=%u mode=%u len=%u payload=%u ch_type=%u rf_type=%u rf_chain=%u",
		data->ble_per_tx_cfg.tx_chnl_num,
		siwx91x_ble_channel_to_freq_mhz(data->ble_per_tx_cfg.tx_chnl_num),
		data->ble_per_tx_cfg.phy_rate,
		data->ble_per_tx_cfg.tx_power, data->ble_per_tx_cfg.transmit_mode,
		siwx91x_ble_per_get_pkt_len(&data->ble_per_tx_cfg),
		data->ble_per_tx_cfg.payload_type, data->ble_per_tx_cfg.le_chnl_type,
		data->ble_per_tx_cfg.rf_type, data->ble_per_tx_cfg.rf_chain);

	return 0;
}

static int siwx91x_ble_per_tx_test_stop_internal(const struct device *dev)
{
	struct siwx91x_ncp_data *data = dev->data;
	rsi_ble_per_transmit_t per_tx;
	int32_t status;

	if (!data->ble_per_tx_test_running) {
		return 0;
	}

	siwx91x_ble_per_build_minimal_cmd(data, &per_tx, false);
	status = rsi_ble_per_transmit(&per_tx);
	if (status != 0) {
		LOG_ERR("Failed to stop BLE PER TX: %d", (int)status);
		return (int)status;
	}

	data->ble_per_tx_test_running = false;
	LOG_INF("BLE PER TX stopped");
	return 0;
}
#endif

#if defined(CONFIG_SHELL)
static int siwx91x_wifi_tx_shell_print_cfg(const struct shell *shell,
					    const struct siwx91x_ncp_data *data)
{
	shell_print(shell, "running=%u channel=%u freq_mhz=%u power=%u rate=%u mode=%u length=%u",
		    data->wifi_tx_test_running ? 1U : 0U,
		    data->wifi_tx_test_cfg.channel,
		    (uint16_t)(2407U + (5U * data->wifi_tx_test_cfg.channel)),
		    data->wifi_tx_test_cfg.power,
		    data->wifi_tx_test_cfg.rate,
		    data->wifi_tx_test_cfg.mode,
		    data->wifi_tx_test_cfg.length);

	return 0;
}

static int cmd_siwx917_wifi_tx_show(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	const struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	return siwx91x_wifi_tx_shell_print_cfg(shell, data);
}

static int cmd_siwx917_wifi_tx_set_channel(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val < 1UL || val > 13UL) {
		shell_error(shell, "channel must be 1..13 for 2.4 GHz");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->wifi_tx_test_cfg.channel = (uint16_t)val;
	shell_print(shell, "wifi tx channel=%u", data->wifi_tx_test_cfg.channel);
	return 0;
}

static int cmd_siwx917_wifi_tx_set_freq(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	uint16_t channel;

	if (val > UINT16_MAX || siwx91x_wifi_freq_mhz_to_channel((uint16_t)val, &channel) < 0) {
		shell_error(shell, "freq_mhz must be 2412..2472 in 5 MHz steps");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->wifi_tx_test_cfg.channel = channel;
	shell_print(shell, "wifi tx freq_mhz=%u (channel=%u)", (uint16_t)val,
		    data->wifi_tx_test_cfg.channel);
	return 0;
}

static int cmd_siwx917_wifi_tx_set_power(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (!((val >= 2UL && val <= 18UL) || val == 127UL)) {
		shell_error(shell, "power must be 2..18 dBm or 127 (region max)");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->wifi_tx_test_cfg.power = (uint16_t)val;
	shell_print(shell, "wifi tx power=%u", data->wifi_tx_test_cfg.power);
	return 0;
}

static int cmd_siwx917_wifi_tx_set_mode(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val > 4UL) {
		shell_error(shell, "mode must be 0..4");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->wifi_tx_test_cfg.mode = (uint16_t)val;

	if (!siwx91x_wifi_tx_length_is_valid(&data->wifi_tx_test_cfg)) {
		data->wifi_tx_test_cfg.length = (val == 1UL) ? 260U : 200U;
	}

	shell_print(shell, "wifi tx mode=%u", data->wifi_tx_test_cfg.mode);
	return 0;
}

static int cmd_siwx917_wifi_tx_set_rate(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val > UINT32_MAX) {
		shell_error(shell, "invalid rate value");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->wifi_tx_test_cfg.rate = (uint32_t)val;
	shell_print(shell, "wifi tx rate=%u", (unsigned int)data->wifi_tx_test_cfg.rate);
	return 0;
}

static int cmd_siwx917_wifi_tx_set_length(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val > UINT16_MAX) {
		shell_error(shell, "invalid length value");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->wifi_tx_test_cfg.length = (uint16_t)val;

	if (!siwx91x_wifi_tx_length_is_valid(&data->wifi_tx_test_cfg)) {
		shell_error(shell,
			    "length invalid for mode %u (mode 1: 24..260, other modes: 24..1500)",
			    data->wifi_tx_test_cfg.mode);
		return -EINVAL;
	}

	shell_print(shell, "wifi tx length=%u", data->wifi_tx_test_cfg.length);
	return 0;
}

static int cmd_siwx917_wifi_tx_start(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	int ret = siwx91x_wifi_tx_test_start_internal(ncp_dev_instance);
	if (ret < 0) {
		shell_error(shell, "failed to start wifi tx test: %d", ret);
		return ret;
	}

	const struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	return siwx91x_wifi_tx_shell_print_cfg(shell, data);
}

static int cmd_siwx917_wifi_tx_stop(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	int ret = siwx91x_wifi_tx_test_stop_internal(ncp_dev_instance);
	if (ret < 0) {
		shell_error(shell, "failed to stop wifi tx test: %d", ret);
		return ret;
	}

	shell_print(shell, "wifi tx stopped");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(siwx917_wifi_tx_cmds,
	SHELL_CMD_ARG(show, NULL, "Show WiFi TX test configuration", cmd_siwx917_wifi_tx_show, 1, 0),
	SHELL_CMD_ARG(channel, NULL, "Set WiFi TX channel (1..13)", cmd_siwx917_wifi_tx_set_channel, 2,
		      0),
	SHELL_CMD_ARG(freq, NULL, "Set WiFi TX center frequency MHz (2412..2472 step 5)",
		      cmd_siwx917_wifi_tx_set_freq, 2, 0),
	SHELL_CMD_ARG(power, NULL, "Set WiFi TX power dBm (2..18 or 127)",
		      cmd_siwx917_wifi_tx_set_power, 2, 0),
	SHELL_CMD_ARG(mode, NULL, "Set WiFi TX mode (0..4)", cmd_siwx917_wifi_tx_set_mode, 2, 0),
	SHELL_CMD_ARG(rate, NULL, "Set WiFi TX rate code (e.g. 0, 139, 256)",
		      cmd_siwx917_wifi_tx_set_rate, 2, 0),
	SHELL_CMD_ARG(length, NULL, "Set WiFi TX payload length", cmd_siwx917_wifi_tx_set_length, 2,
		      0),
	SHELL_CMD_ARG(start, NULL, "Start WiFi TX test", cmd_siwx917_wifi_tx_start, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop WiFi TX test", cmd_siwx917_wifi_tx_stop, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(siwx917_wifi_tx, &siwx917_wifi_tx_cmds,
		       "SiWx917 WiFi TX test CLI", NULL, 1, 0);

#if defined(CONFIG_BT_SILABS_SIWX91X) || defined(CONFIG_SIWX91X_NCP_BLE_RF_TEST)
static int siwx91x_ble_tx_shell_print_cfg(const struct shell *shell,
					   const struct siwx91x_ncp_data *data)
{
	shell_print(shell,
		    "running=%u tx_ch=%u (%u MHz) rx_ch=%u (%u MHz) phy=%u power=%u mode=%u len=%u payload=%u ch_type=%u rf_type=%u rf_chain=%u num_pkts=%u",
		    data->ble_per_tx_test_running ? 1U : 0U,
		    data->ble_per_tx_cfg.tx_chnl_num,
		    siwx91x_ble_channel_to_freq_mhz(data->ble_per_tx_cfg.tx_chnl_num),
		    data->ble_per_tx_cfg.rx_chnl_num,
		    siwx91x_ble_channel_to_freq_mhz(data->ble_per_tx_cfg.rx_chnl_num),
		    data->ble_per_tx_cfg.phy_rate,
		    data->ble_per_tx_cfg.tx_power,
		    data->ble_per_tx_cfg.transmit_mode,
		    siwx91x_ble_per_get_pkt_len(&data->ble_per_tx_cfg),
		    data->ble_per_tx_cfg.payload_type,
		    data->ble_per_tx_cfg.le_chnl_type,
		    data->ble_per_tx_cfg.rf_type,
		    data->ble_per_tx_cfg.rf_chain,
		    (unsigned int)siwx91x_ble_per_get_num_pkts(&data->ble_per_tx_cfg));

	return 0;
}

static int cmd_siwx917_ble_tx_show(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	const struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	return siwx91x_ble_tx_shell_print_cfg(shell, data);
}

static int cmd_siwx917_ble_tx_set_channel(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val > 39UL) {
		shell_error(shell, "channel must be 0..39");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->ble_per_tx_cfg.tx_chnl_num = (uint8_t)val;
	data->ble_per_tx_cfg.rx_chnl_num = (uint8_t)val;
	/* Advertising channels are 37-39; others are data channels. */
	data->ble_per_tx_cfg.le_chnl_type = (val >= 37UL) ? 0U : 1U;
	shell_print(shell, "ble tx channel=%u (%u MHz) ch_type=%u",
		    data->ble_per_tx_cfg.tx_chnl_num,
		    siwx91x_ble_channel_to_freq_mhz(data->ble_per_tx_cfg.tx_chnl_num),
		    data->ble_per_tx_cfg.le_chnl_type);
	return 0;
}

static int cmd_siwx917_ble_tx_set_phy(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (!(val == 1UL || val == 2UL || val == 4UL || val == 8UL)) {
		shell_error(shell, "phy must be one of: 1, 2, 4, 8");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->ble_per_tx_cfg.phy_rate = (uint8_t)val;
	shell_print(shell, "ble tx phy=%u", data->ble_per_tx_cfg.phy_rate);
	return 0;
}

static int cmd_siwx917_ble_tx_set_power(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val == 0UL || val > 127UL || val == 32UL) {
		shell_error(shell, "power index must be 1..127 (32 is invalid)");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->ble_per_tx_cfg.tx_power = (uint8_t)val;
	shell_print(shell, "ble tx power=%u", data->ble_per_tx_cfg.tx_power);
	return 0;
}

static int cmd_siwx917_ble_tx_set_length(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val == 0UL || val > 255UL) {
		shell_error(shell, "length must be 1..255 bytes");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	siwx91x_ble_per_set_pkt_len(&data->ble_per_tx_cfg, (uint16_t)val);
	shell_print(shell, "ble tx length=%u", (unsigned int)val);
	return 0;
}

static int cmd_siwx917_ble_tx_set_payload(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val > 7UL) {
		shell_error(shell, "payload must be 0..7");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->ble_per_tx_cfg.payload_type = (uint8_t)val;
	shell_print(shell, "ble tx payload=%u", data->ble_per_tx_cfg.payload_type);
	return 0;
}

static int cmd_siwx917_ble_tx_set_mode(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val > 2UL) {
		shell_error(shell, "mode must be 0..2");
		return -EINVAL;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	data->ble_per_tx_cfg.transmit_mode = (uint8_t)val;
	if (data->ble_per_tx_cfg.transmit_mode == 1U) {
		data->ble_per_tx_cfg.scrambler_seed = 5U;
	} else if (data->ble_per_tx_cfg.scrambler_seed == 5U) {
		data->ble_per_tx_cfg.scrambler_seed = 0U;
	}
	shell_print(shell, "ble tx mode=%u", data->ble_per_tx_cfg.transmit_mode);
	return 0;
}

static int cmd_siwx917_ble_tx_start(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	int ret = siwx91x_ble_per_tx_test_start_internal(ncp_dev_instance);
	if (ret != 0) {
		shell_error(shell, "failed to start BLE PER tx test: %d (0x%x)", ret,
			    (unsigned int)ret);
		return ret;
	}

	const struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	return siwx91x_ble_tx_shell_print_cfg(shell, data);
}

static int cmd_siwx917_ble_tx_stop(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ncp_dev_instance == NULL) {
		shell_error(shell, "NCP device not initialized");
		return -ENODEV;
	}

	int ret = siwx91x_ble_per_tx_test_stop_internal(ncp_dev_instance);
	if (ret != 0) {
		shell_error(shell, "failed to stop BLE PER tx test: %d (0x%x)", ret,
			    (unsigned int)ret);
		return ret;
	}

	shell_print(shell, "ble tx stopped");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(siwx917_ble_tx_cmds,
	SHELL_CMD_ARG(show, NULL, "Show BLE PER TX test configuration", cmd_siwx917_ble_tx_show, 1,
		      0),
	SHELL_CMD_ARG(channel, NULL, "Set BLE TX/RX channel (0..39)", cmd_siwx917_ble_tx_set_channel,
		      2, 0),
	SHELL_CMD_ARG(phy, NULL, "Set BLE PHY (1, 2, 4, 8)", cmd_siwx917_ble_tx_set_phy, 2, 0),
	SHELL_CMD_ARG(power, NULL, "Set BLE TX power index (1..127, 32 invalid)",
		      cmd_siwx917_ble_tx_set_power, 2, 0),
	SHELL_CMD_ARG(length, NULL, "Set BLE TX packet length (1..255)",
		      cmd_siwx917_ble_tx_set_length, 2, 0),
	SHELL_CMD_ARG(payload, NULL, "Set BLE payload type (0..7)", cmd_siwx917_ble_tx_set_payload,
		      2, 0),
	SHELL_CMD_ARG(mode, NULL, "Set BLE transmit mode (0 burst, 1 continuous, 2 CW)",
		      cmd_siwx917_ble_tx_set_mode, 2, 0),
	SHELL_CMD_ARG(start, NULL, "Start BLE PER TX test", cmd_siwx917_ble_tx_start, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop BLE PER TX test", cmd_siwx917_ble_tx_stop, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(siwx917_ble_tx, &siwx917_ble_tx_cmds,
		       "SiWx917 BLE PER TX test CLI", NULL, 1, 0);
#endif
#endif

/* We support exactly one instance (singleton) */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "Only one SiWx91x NCP SPI instance is supported");

/* Pointer to the singleton device — needed by the sl_si91x_host_*() functions
 * which have no device parameter (they are global platform callbacks).
 */

/* ========================================================================== */
/* Country code / region mapping (shared with SoC NWP driver)                  */
/* ========================================================================== */

typedef struct {
	const char *const *codes;
	size_t num_codes;
	sl_wifi_region_code_t region_code;
	const sli_wifi_set_region_ap_request_t *sdk_reg;
} region_map_t;

extern const sli_wifi_set_region_ap_request_t default_US_region_2_4GHZ_configurations;
extern const sli_wifi_set_region_ap_request_t default_EU_region_2_4GHZ_configurations;
extern const sli_wifi_set_region_ap_request_t default_JP_region_2_4GHZ_configurations;
extern const sli_wifi_set_region_ap_request_t default_KR_region_2_4GHZ_configurations;
extern const sli_wifi_set_region_ap_request_t default_SG_region_2_4GHZ_configurations;
extern const sli_wifi_set_region_ap_request_t default_CN_region_2_4GHZ_configurations;

static const char *const us_codes[] = {
	"AE", "AR", "AS", "BB", "BM", "BR", "BS", "CA", "CO", "CR", "CU", "CX", "DM", "DO",
	"EC", "FM", "GD", "GY", "GU", "HN", "HT", "JM", "KY", "LB", "LK", "MH", "MN", "MP",
	"MO", "MY", "NI", "PA", "PE", "PG", "PH", "PK", "PR", "PW", "PY", "SG", "MX", "SV",
	"TC", "TH", "TT", "US", "UY", "VE", "VI", "VN", "VU", "00"
};
static const char *const eu_codes[] = {
	"AD", "AF", "AI", "AL", "AM", "AN", "AT", "AW", "AU", "AZ", "BA", "BE", "BG", "BH", "BL",
	"BT", "BY", "CH", "CY", "CZ", "DE", "DK", "EE", "ES", "FR", "GB", "GE", "GF", "GL", "GP",
	"GR", "GT", "HK", "HR", "HU", "ID", "IE", "IL", "IN", "IR", "IS", "IT", "JO", "KH", "FI",
	"KN", "KW", "KZ", "LC", "LI", "LT", "LU", "LV", "MD", "ME", "MK", "MF", "MT", "MV", "MQ",
	"NL", "NO", "NZ", "OM", "PF", "PL", "PM", "PT", "QA", "RO", "RS", "RU", "SA", "SE", "SI",
	"SK", "SR", "SY", "TR", "TW", "UA", "UZ", "VC", "WF", "WS", "YE", "RE", "YT"
};
static const char *const jp_codes[] = {"BD", "BN", "BO", "CL", "BZ", "JP", "NP"};
static const char *const kr_codes[] = {"KR", "KP"};
static const char *const cn_codes[] = {"CN"};
static const region_map_t region_maps[] = {
	{us_codes, ARRAY_SIZE(us_codes), SL_WIFI_REGION_US,
	 &default_US_region_2_4GHZ_configurations},
	{eu_codes, ARRAY_SIZE(eu_codes), SL_WIFI_REGION_EU,
	 &default_EU_region_2_4GHZ_configurations},
	{jp_codes, ARRAY_SIZE(jp_codes), SL_WIFI_REGION_JP,
	 &default_JP_region_2_4GHZ_configurations},
	{kr_codes, ARRAY_SIZE(kr_codes), SL_WIFI_REGION_KR,
	 &default_KR_region_2_4GHZ_configurations},
	{cn_codes, ARRAY_SIZE(cn_codes), SL_WIFI_REGION_CN,
	 &default_CN_region_2_4GHZ_configurations},
};

int siwx91x_store_country_code(const struct device *dev, const char *country_code)
{
	__ASSERT(country_code, "country_code cannot be NULL");
	struct siwx91x_ncp_data *data = dev->data;

	memcpy(data->current_country_code, country_code, WIFI_COUNTRY_CODE_LEN);
	return 0;
}

const char *siwx91x_get_country_code(const struct device *dev)
{
	const struct siwx91x_ncp_data *data = dev->data;

	return data->current_country_code;
}

sl_wifi_region_code_t siwx91x_map_country_code_to_region(const char *country_code)
{
	__ASSERT(country_code, "country_code cannot be NULL");

	ARRAY_FOR_EACH(region_maps, i) {
		for (size_t j = 0; j < region_maps[i].num_codes; j++) {
			if (memcmp(country_code, region_maps[i].codes[j],
				   WIFI_COUNTRY_CODE_LEN) == 0) {
				return region_maps[i].region_code;
			}
		}
	}
	return SL_WIFI_DEFAULT_REGION;
}

const sli_wifi_set_region_ap_request_t *siwx91x_find_sdk_region_table(uint8_t region_code)
{
	ARRAY_FOR_EACH(region_maps, i) {
		if (region_maps[i].region_code == region_code) {
			return region_maps[i].sdk_reg;
		}
	}
	return NULL;
}

/* ========================================================================== */
/* sl_si91x_host_interface.h implementation                                   */
/* ========================================================================== */

static void siwx91x_ncp_irq_callback(const struct device *port, struct gpio_callback *cb,
				      gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (ncp_dev_instance == NULL) {
		return;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	// LOG_INF("irq cb %d %p", data->bus_irq_enabled, data->rx_irq_handler);

	if (data->bus_irq_enabled && data->rx_irq_handler != NULL) {
		data->rx_irq_handler();
	}
}

sl_status_t sl_si91x_host_init(const sl_si91x_host_init_configuration_t *config)
{
	if (ncp_dev_instance == NULL) {
		return SL_STATUS_NOT_INITIALIZED;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;
	int ret;

	if (config != NULL) {
		data->rx_irq_handler = config->rx_irq;
	}

	/* Verify SPI device is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return SL_STATUS_NOT_INITIALIZED;
	}

	/* Configure RESET GPIO */
	if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
		LOG_ERR("Reset GPIO device not ready");
		return SL_STATUS_NOT_INITIALIZED;
	}
	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);
	if (ret < 0) {
		LOG_ERR("Failed to configure reset GPIO: %d", ret);
		return SL_STATUS_FAIL;
	}

	/* Configure CS GPIO */
	if (!gpio_is_ready_dt(&cfg->cs_gpio)) {
		LOG_ERR("CS GPIO device not ready");
		return SL_STATUS_NOT_INITIALIZED;
	}
	ret = gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure CS GPIO: %d", ret);
		return SL_STATUS_FAIL;
	}

#if DT_INST_NODE_HAS_PROP(0, sleep_request_gpios)
	if (!gpio_is_ready_dt(&cfg->sleep_gpio)) {
		LOG_ERR("Sleep GPIO device not ready");
		return SL_STATUS_NOT_INITIALIZED;
	}
	ret = gpio_pin_configure_dt(&cfg->sleep_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure Sleep GPIO: %d", ret);
		return SL_STATUS_FAIL;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, wake_gpios)
	if (!gpio_is_ready_dt(&cfg->wake_gpio)) {
		LOG_ERR("Wake GPIO device not ready");
		return SL_STATUS_NOT_INITIALIZED;
	}
	ret = gpio_pin_configure_dt(&cfg->wake_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure Wake GPIO: %d", ret);
		return SL_STATUS_FAIL;
	}
#endif
	gpio_pin_configure(gpiog_dev, LOAD_SW_PIN,
                       GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	gpio_pin_set(gpiog_dev, LOAD_SW_PIN, 1);
	k_msleep(100);

	gpio_pin_configure(gpioa_dev, 15,
                       GPIO_OUTPUT | GPIO_ACTIVE_LOW | GPIO_PULL_UP);

	/* Configure IRQ GPIO as input with interrupt on rising edge */
	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO device not ready");
		return SL_STATUS_NOT_INITIALIZED;
	}
	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
		return SL_STATUS_FAIL;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO interrupt: %d", ret);
		return SL_STATUS_FAIL;
	}

	gpio_init_callback(&data->irq_cb_data, siwx91x_ncp_irq_callback,
			   BIT(cfg->irq_gpio.pin));
	ret = gpio_add_callback(cfg->irq_gpio.port, &data->irq_cb_data);
	if (ret < 0) {
		LOG_ERR("Failed to add IRQ GPIO callback: %d", ret);
		return SL_STATUS_FAIL;
	}

	data->bus_irq_enabled = false;

	LOG_INF("SiWx91x NCP SPI host initialized (freq: %u Hz)",
		data->spi_cfg.frequency);

	return SL_STATUS_OK;
}

sl_status_t sl_si91x_host_deinit(void)
{
	if (ncp_dev_instance == NULL) {
		return SL_STATUS_NOT_INITIALIZED;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;

	data->bus_irq_enabled = false;
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
	gpio_remove_callback(cfg->irq_gpio.port, &data->irq_cb_data);

	LOG_INF("SiWx91x NCP SPI host deinitialized");
	return SL_STATUS_OK;
}

/* === SPI Transfer === */
#define SLI_SPI_BUFFER_LENGTH 2300
uint8_t sli_spi_buffer[SLI_SPI_BUFFER_LENGTH];
sl_status_t sl_si91x_host_spi_transfer(const void *tx_buffer, void *rx_buffer,
				       uint16_t buffer_length)
{
	if (ncp_dev_instance == NULL) {
		return SL_STATUS_NOT_INITIALIZED;
	}

	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;
	int ret = 0;

	if (tx_buffer == NULL) {
		tx_buffer = sli_spi_buffer;
	}

	if (rx_buffer == NULL) {
		rx_buffer = sli_spi_buffer;
	}
	
	struct spi_buf tx_buf = {
		.buf = (void *)tx_buffer,
		.len = buffer_length,
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = buffer_length,
	};
	struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};
	ret = spi_transceive(cfg->spi.bus, &data->spi_cfg, &tx_set, &rx_set);

	if (ret < 0) {
		LOG_ERR("SPI transfer failed: %d (len=%u)", ret, buffer_length);
		return SL_STATUS_FAIL;
	}

	return SL_STATUS_OK;
}

/* === Chip Select === */

void sl_si91x_host_spi_cs_assert(void)
{
	if (ncp_dev_instance == NULL) {
		return;
	}
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;

	/* CS is on the SPI bus cs-gpios — assert via GPIO for manual control */
	// gpio_pin_set_dt(&cfg->cs_gpio, 1);
	gpio_pin_set(gpioa_dev, 15, 1);
}

void sl_si91x_host_spi_cs_deassert(void)
{
	if (ncp_dev_instance == NULL) {
		return;
	}
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;

	// gpio_pin_set_dt(&cfg->cs_gpio, 0);
	gpio_pin_set(gpioa_dev, 15, 0);
}

/* === Reset Control === */

void sl_si91x_host_hold_in_reset(void)
{
	if (ncp_dev_instance == NULL) {
		return;
	}
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;

	gpio_pin_set_dt(&cfg->reset_gpio, 1); /* Assert reset (active-low handled by DT flags) */
	LOG_DBG("SiWx91x held in reset");
}

void sl_si91x_host_release_from_reset(void)
{
	if (ncp_dev_instance == NULL) {
		return;
	}
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;

	gpio_pin_set_dt(&cfg->reset_gpio, 0); /* Deassert reset */
	LOG_DBG("SiWx91x released from reset");
}

/* === Bus Interrupt Control === */

void sl_si91x_host_enable_bus_interrupt(void)
{
	if (ncp_dev_instance == NULL) {
		return;
	}
	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;

	data->bus_irq_enabled = true;
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	LOG_DBG("Bus interrupt enabled");
}

void sl_si91x_host_disable_bus_interrupt(void)
{
	if (ncp_dev_instance == NULL) {
		return;
	}
	struct siwx91x_ncp_data *data = ncp_dev_instance->data;
	const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;

	data->bus_irq_enabled = false;
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
	LOG_DBG("Bus interrupt disabled");
}

/* === High Speed Bus === */

void sl_si91x_host_enable_high_speed_bus(void)
{
	if (ncp_dev_instance == NULL) {
		return;
	}
	struct siwx91x_ncp_data *data = ncp_dev_instance->data;

	data->spi_cfg.frequency = CONFIG_SIWX91X_NCP_SPI_HIGH_SPEED_FREQUENCY;
	LOG_INF("SPI switched to high-speed: %u Hz", data->spi_cfg.frequency);
}

/* === Sleep / Wake Indicators === */

void sl_si91x_host_set_sleep_indicator(void)
{
// #if DT_INST_NODE_HAS_PROP(0, sleep_request_gpios)
// 	if (ncp_dev_instance != NULL) {
// 		const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;
// 		LOG_INF("SET SLEEP INDICATOR");
// 		gpio_pin_set_dt(&cfg->sleep_gpio, 1);
// 	}
// #endif
}

void sl_si91x_host_clear_sleep_indicator(void)
{
// #if DT_INST_NODE_HAS_PROP(0, sleep_request_gpios)
// 	if (ncp_dev_instance != NULL) {
// 		const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;
// 		LOG_INF("CLEAR SLEEP INDICATOR");
// 		gpio_pin_set_dt(&cfg->sleep_gpio, 0);
// 	}
// 	LOG_DBG("Sleep indicator cleared");
// #endif
}

uint32_t sl_si91x_host_get_wake_indicator(void)
{
// #if DT_INST_NODE_HAS_PROP(0, wake_indicator_gpios)
// 	if (ncp_dev_instance != NULL) {
// 		const struct siwx91x_ncp_config *cfg = ncp_dev_instance->config;
// 		// LOG_INF("GET WAKE INDICATOR");
// 		return gpio_pin_get_dt(&cfg->wake_gpio);
// 	}
// #endif
	/* Default: always awake */
	return 1;
}

/* === UART stubs (unused in SPI mode) === */

sl_status_t sl_si91x_host_uart_transfer(const void *tx_buffer, void *rx_buffer,
					uint16_t buffer_length)
{
	ARG_UNUSED(tx_buffer);
	ARG_UNUSED(rx_buffer);
	ARG_UNUSED(buffer_length);
	return SL_STATUS_NOT_SUPPORTED;
}

void sl_si91x_host_flush_uart_rx(void)
{
	/* Not used in SPI mode */
}

void sl_si91x_host_uart_enable_hardware_flow_control(void)
{
	/* Not used in SPI mode */
}

/* === Context Check === */

bool sl_si91x_host_is_in_irq_context(void)
{
	return k_is_in_isr();
}

/* ========================================================================== */
/* NWP configuration and boot                                                 */
/* ========================================================================== */

#define AP_MAX_NUM_STA 4
#define MEMORY_CONFIG (BIT(20) | BIT(21))
static void siwx91x_ncp_configure_sta_mode(sl_si91x_boot_configuration_t *boot_config)
{
	const bool wifi_enabled = IS_ENABLED(CONFIG_WIFI_SILABS_SIWX91X);
	const bool bt_enabled = IS_ENABLED(CONFIG_BT_SILABS_SIWX91X) ||
				IS_ENABLED(CONFIG_SIWX91X_NCP_BLE_RF_TEST);
	const bool ble_rf_test_only = IS_ENABLED(CONFIG_SIWX91X_NCP_BLE_RF_TEST) &&
				      !IS_ENABLED(CONFIG_BT_SILABS_SIWX91X);

	if (ble_rf_test_only) {
		/* Keep BLE RF-test boot profile close to WiseConnect BLE examples. */
		boot_config->oper_mode = SL_SI91X_CLIENT_MODE;
		boot_config->coex_mode = SL_SI91X_WLAN_BLE_MODE;
		boot_config->feature_bit_map = SL_SI91X_FEAT_WPS_DISABLE |
			SL_SI91X_FEAT_ULP_GPIO_BASED_HANDSHAKE |
			SL_SI91X_FEAT_DEV_TO_HOST_ULP_GPIO_1;
		boot_config->tcp_ip_feature_bit_map = SL_SI91X_TCP_IP_FEAT_DHCPV4_CLIENT |
			SL_SI91X_TCP_IP_FEAT_EXTENSION_VALID;
		boot_config->custom_feature_bit_map = SL_SI91X_CUSTOM_FEAT_EXTENTION_VALID;
		boot_config->ext_custom_feature_bit_map = SL_SI91X_EXT_FEAT_LOW_POWER_MODE |
			SL_SI91X_EXT_FEAT_XTAL_CLK |
			MEMORY_CONFIG |
			SL_SI91X_EXT_FEAT_FRONT_END_SWITCH_PINS_ULP_GPIO_4_5_0 |
			SL_SI91X_EXT_FEAT_BT_CUSTOM_FEAT_ENABLE;
		boot_config->bt_feature_bit_map = SL_SI91X_BT_RF_TYPE |
			SL_SI91X_ENABLE_BLE_PROTOCOL;
		boot_config->ext_tcp_ip_feature_bit_map = SL_SI91X_CONFIG_FEAT_EXTENSION_VALID;
		boot_config->ble_feature_bit_map =
			SL_SI91X_BLE_MAX_NBR_ATT_REC(80) |
			SL_SI91X_BLE_MAX_NBR_ATT_SERV(10) |
			SL_SI91X_BLE_MAX_NBR_PERIPHERALS(1) |
			SL_SI91X_BLE_PWR_INX(31) |
			SL_SI91X_BLE_PWR_SAVE_OPTIONS(0) |
			SL_SI91X_BLE_MAX_NBR_CENTRALS(1) |
			SL_SI91X_916_BLE_COMPATIBLE_FEAT_ENABLE |
			SL_SI91X_FEAT_BLE_CUSTOM_FEAT_EXTENSION_VALID;
		boot_config->ble_ext_feature_bit_map =
			SL_SI91X_BLE_NUM_CONN_EVENTS(20) |
			SL_SI91X_BLE_NUM_REC_BYTES(64);
		boot_config->config_feature_bit_map = 0;
		return;
	}

	boot_config->oper_mode = ble_rf_test_only ? SL_SI91X_CLIENT_MODE :
				      SL_SI91X_TRANSMIT_TEST_MODE;

	if (IS_ENABLED(CONFIG_WIFI_SILABS_SIWX91X_ROAMING_USE_DEAUTH)) {
		boot_config->custom_feature_bit_map |=
			SL_SI91X_CUSTOM_FEAT_ROAM_WITH_DEAUTH_OR_NULL_DATA | 
			SL_SI91X_CUSTOM_FEAT_WAKE_ON_WIRELESS | SL_SI91X_CUSTOM_FEAT_EXTENTION_VALID;
	}

	if (ble_rf_test_only) {
		boot_config->coex_mode = SL_SI91X_BLE_MODE;
	} else if (wifi_enabled && bt_enabled) {
		LOG_ERR("WiFi + Bluetooth coexistence is not fully supported in STA mode; using WLAN_BLE_MODE which has limited BT features");
		boot_config->coex_mode = SL_SI91X_WLAN_BLE_MODE;
	} else if (wifi_enabled) {
		boot_config->coex_mode = SL_SI91X_WLAN_BLE_MODE;
	} else if (bt_enabled) {
		boot_config->coex_mode = SL_SI91X_WLAN_BLE_MODE;
	} else {
		boot_config->coex_mode = SL_SI91X_WLAN_BLE_MODE;
	}

#ifdef CONFIG_WIFI_SILABS_SIWX91X
	if (!ble_rf_test_only) {
		/* 802.11W (management frame protection) — needed for WPA3 */
		boot_config->ext_custom_feature_bit_map |= SL_SI91X_EXT_FEAT_IEEE_80211W |
			SL_SI91X_EXT_FEAT_FRONT_END_SWITCH_PINS_ULP_GPIO_4_5_0;

		if (IS_ENABLED(CONFIG_WIFI_SILABS_SIWX91X_ENHANCED_MAX_PSP)) {
			boot_config->config_feature_bit_map = SL_SI91X_FEAT_SLEEP_GPIO_SEL_BITMAP |
				SL_SI91X_ULP_GPIO9_FOR_UART2_TX |
				SL_SI91X_ENABLE_ENHANCED_MAX_PSP;
		}
	}
#endif
	
	if (bt_enabled) {
		boot_config->ext_custom_feature_bit_map |= SL_SI91X_EXT_FEAT_BT_CUSTOM_FEAT_ENABLE;
		boot_config->bt_feature_bit_map |= SL_SI91X_BT_RF_TYPE | SL_SI91X_ENABLE_BLE_PROTOCOL;

		if (ble_rf_test_only) {
			/* Keep valid-but-minimal BLE ranges for RF test-only mode. */
			boot_config->ble_feature_bit_map |=
				SL_SI91X_BLE_MAX_NBR_ATT_REC(20) |
				SL_SI91X_BLE_MAX_NBR_ATT_SERV(1) |
				SL_SI91X_BLE_MAX_NBR_PERIPHERALS(1) |
				SL_SI91X_BLE_PWR_INX(31) |
				SL_SI91X_BLE_MAX_NBR_CENTRALS(1) |
				SL_SI91X_FEAT_BLE_CUSTOM_FEAT_EXTENSION_VALID;
			boot_config->ble_ext_feature_bit_map |=
				SL_SI91X_BLE_NUM_CONN_EVENTS(1) |
				SL_SI91X_BLE_NUM_REC_BYTES(64);
		} else {
			boot_config->ble_feature_bit_map |=
				SL_SI91X_BLE_MAX_NBR_ATT_REC(124) |
				SL_SI91X_BLE_MAX_NBR_ATT_SERV(10) |
				SL_SI91X_BLE_MAX_NBR_PERIPHERALS(8) |
				SL_SI91X_BLE_PWR_INX(31) |
				SL_SI91X_BLE_PWR_SAVE_OPTIONS(0) |
				SL_SI91X_BLE_MAX_NBR_CENTRALS(2) |
				SL_SI91X_BLE_GATT_ASYNC_ENABLE |
				SL_SI91X_916_BLE_COMPATIBLE_FEAT_ENABLE |
				SL_SI91X_FEAT_BLE_CUSTOM_FEAT_EXTENSION_VALID;

			boot_config->ble_ext_feature_bit_map |=
				SL_SI91X_BLE_NUM_CONN_EVENTS(20) | SL_SI91X_BLE_ENABLE_ADV_EXTN |
				SL_SI91X_BLE_GATT_INIT |
				SL_SI91X_BLE_AE_MAX_ADV_SETS(2);
		}
	}
}

static void siwx91x_ncp_configure_ap_mode(sl_si91x_boot_configuration_t *boot_config,
					   bool hidden_ssid, uint8_t max_num_sta)
{
	boot_config->oper_mode = SL_SI91X_ACCESS_POINT_MODE;
	boot_config->coex_mode = SL_SI91X_WLAN_ONLY_MODE;

	if (IS_ENABLED(CONFIG_WIFI_SILABS_SIWX91X_LIMIT_PACKET_BUF_PER_STA)) {
		boot_config->custom_feature_bit_map |= SL_SI91X_CUSTOM_FEAT_LIMIT_PACKETS_PER_STA;
	}

	if (hidden_ssid) {
		boot_config->custom_feature_bit_map |= SL_SI91X_CUSTOM_FEAT_AP_IN_HIDDEN_MODE;
	}

	boot_config->custom_feature_bit_map |= SL_WIFI_CUSTOM_FEAT_MAX_NUM_OF_CLIENTS(max_num_sta);
}

static void siwx91x_ncp_configure_network_stack(sl_si91x_boot_configuration_t *boot_config,
						 uint8_t wifi_oper_mode)
{
	if (!IS_ENABLED(CONFIG_WIFI_SILABS_SIWX91X_NET_STACK_OFFLOAD)) {
		/* Host manages the TCP/IP stack; NWP passes raw frames */
		boot_config->tcp_ip_feature_bit_map = SL_SI91X_TCP_IP_FEAT_BYPASS | SL_SI91X_TCP_IP_FEAT_EXTENSION_VALID;
		// boot_config->ext_tcp_ip_feature_bit_map = 0;
		return;
	}

	/* Offloaded stack — bits in the base config are the SDK defaults,
	 * add only mode-specific extras here.
	 */
	if (IS_ENABLED(CONFIG_NET_IPV6)) {
		boot_config->tcp_ip_feature_bit_map |= SL_SI91X_TCP_IP_FEAT_IPV6;
		if (wifi_oper_mode == WIFI_STA_MODE) {
			boot_config->tcp_ip_feature_bit_map |= SL_SI91X_TCP_IP_FEAT_DHCPV6_CLIENT;
		} else if (wifi_oper_mode == WIFI_SOFTAP_MODE) {
			boot_config->tcp_ip_feature_bit_map |= SL_SI91X_TCP_IP_FEAT_DHCPV6_SERVER;
		}
	}

	if (IS_ENABLED(CONFIG_NET_IPV4)) {
		if (wifi_oper_mode == WIFI_SOFTAP_MODE) {
			boot_config->tcp_ip_feature_bit_map |= SL_SI91X_TCP_IP_FEAT_DHCPV4_SERVER;
		}
		/* STA mode already has DHCPV4_CLIENT in the base config */
	}
}

static int siwx91x_ncp_get_config(const struct device *dev,
				   sl_wifi_device_configuration_t *get_config,
				   uint8_t wifi_oper_mode, bool hidden_ssid, uint8_t max_num_sta)
{
	/*
	 * Base configuration built from the SDK default
	 * sl_wifi_default_client_configuration (NCP path, i.e.
	 * !SLI_SI91X_MCU_INTERFACE).  Only Zephyr-specific Kconfig knobs
	 * should add bits on top of this baseline.
	 *
	 * MEMORY_CONFIG in NCP mode resolves to (BIT(20)|BIT(21)) which
	 * equals SL_SI91X_EXT_FEAT_672K_M4SS_0K — all SRAM goes to the
	 * NWP since the M4 is not executing user code.
	 */
	const bool ble_rf_test_only = IS_ENABLED(CONFIG_SIWX91X_NCP_BLE_RF_TEST) &&
				      !IS_ENABLED(CONFIG_BT_SILABS_SIWX91X);
	sl_wifi_device_configuration_t default_config = ble_rf_test_only ?
		sl_wifi_default_client_configuration :
		sl_wifi_default_transmit_test_configuration;
	default_config.region_code = SL_WIFI_IGNORE_REGION;

	sl_si91x_boot_configuration_t *boot_config = &default_config.boot_config;

	__ASSERT(get_config, "get_config cannot be NULL");
	__ASSERT((hidden_ssid == false && max_num_sta == 0) ||
		 wifi_oper_mode == WIFI_SOFTAP_MODE,
		 "hidden_ssid or max_num_sta requires SOFT AP mode");

	if (wifi_oper_mode == WIFI_SOFTAP_MODE && max_num_sta > AP_MAX_NUM_STA) {
		LOG_ERR("Exceeded maximum supported stations (%d)", AP_MAX_NUM_STA);
		return -EINVAL;
	}

	siwx91x_store_country_code(dev, DEFAULT_COUNTRY_CODE);

	switch (wifi_oper_mode) {
	case WIFI_STA_MODE:
		siwx91x_ncp_configure_sta_mode(boot_config);
		LOG_INF("Configured STA mode, coex_mode=%d, bt_feature_bit_map=0x%x, ble_feature_bit_map=0x%x, ble_ext_feature_bit_map=0x%x",
			boot_config->coex_mode, boot_config->bt_feature_bit_map, boot_config->ble_feature_bit_map, boot_config->ble_ext_feature_bit_map);
		LOG_INF("STA boot cfg: oper=%u feat=0x%x tcp=0x%x cust=0x%x ext_cust=0x%x ext_tcp=0x%x cfg=0x%x",
			boot_config->oper_mode, boot_config->feature_bit_map,
			boot_config->tcp_ip_feature_bit_map, boot_config->custom_feature_bit_map,
			boot_config->ext_custom_feature_bit_map,
			boot_config->ext_tcp_ip_feature_bit_map,
			boot_config->config_feature_bit_map);
		break;
	case WIFI_SOFTAP_MODE:
		siwx91x_ncp_configure_ap_mode(boot_config, hidden_ssid, max_num_sta);
		break;
	default:
		return -EINVAL;
	}

	siwx91x_ncp_configure_network_stack(boot_config, wifi_oper_mode);
	

	memcpy(get_config, &default_config, sizeof(default_config));
	return 0;
}

static int siwx91x_ncp_check_fw_version(void)
{
	sl_wifi_firmware_version_t version;
	int ret;

	ret = sl_wifi_get_firmware_version(&version);
	if (ret != SL_STATUS_OK) {
		LOG_ERR("Failed to read NWP firmware version: 0x%x", ret);
		return -EINVAL;
	}

	LOG_ERR("NWP firmware %x%x.%u.%u.%u.%u.%u.%u (expected %X.%u.%u.%u.%u.%u.%u)",
		version.chip_id, version.rom_id, version.major, version.minor,
		version.security_version, version.patch_num, version.customer_id,
		version.build_num,
		siwx91x_nwp_fw_expected_version.rom_id,
		siwx91x_nwp_fw_expected_version.major,
		siwx91x_nwp_fw_expected_version.minor,
		siwx91x_nwp_fw_expected_version.security_version,
		siwx91x_nwp_fw_expected_version.patch_num,
		siwx91x_nwp_fw_expected_version.customer_id,
		siwx91x_nwp_fw_expected_version.build_num);

	if (siwx91x_nwp_fw_expected_version.major != version.major ||
	    siwx91x_nwp_fw_expected_version.minor != version.minor ||
	    siwx91x_nwp_fw_expected_version.security_version != version.security_version ||
	    siwx91x_nwp_fw_expected_version.patch_num != version.patch_num) {
		return -EINVAL;
	}

	if (siwx91x_nwp_fw_expected_version.customer_id != version.customer_id) {
		LOG_DBG("customer_id diverge: expected %d, actual %d",
			siwx91x_nwp_fw_expected_version.customer_id, version.customer_id);
	}
	if (siwx91x_nwp_fw_expected_version.build_num != version.build_num) {
		LOG_DBG("build_num diverge: expected %d, actual %d",
			siwx91x_nwp_fw_expected_version.build_num, version.build_num);
	}

	return 0;
}

int siwx91x_nwp_apply_power_profile(const struct device *dev,
				    const sl_wifi_performance_profile_v2_t *wifi_profile)
{
	sl_wifi_performance_profile_v2_t profile = {
		.profile = SL_WIFI_SYSTEM_HIGH_PERFORMANCE,
	};
	int ret;

	ARG_UNUSED(dev);

	if (wifi_profile != NULL) {
		profile = *wifi_profile;
	}

	ret = sl_wifi_set_performance_profile_v2(&profile);
	if (ret != SL_STATUS_OK) {
		return -EINVAL;
 	}
 
#if defined(CONFIG_BT_SILABS_SIWX91X) || defined(CONFIG_SIWX91X_NCP_BLE_RF_TEST)
	if (IS_ENABLED(CONFIG_BT_SILABS_SIWX91X) || IS_ENABLED(CONFIG_SIWX91X_NCP_BLE_RF_TEST)) {
		sl_bt_performance_profile_t bt_profile = { .profile = profile.profile };

		ret = sl_si91x_bt_set_performance_profile(&bt_profile);
		if (ret != SL_STATUS_OK) {
			return -EINVAL;
		}
	}
#endif

	return 0;
}

/* ========================================================================== */
/* NWP mode switch (called by siwx91x_wifi.c)                                 */
/* ========================================================================== */

int siwx91x_nwp_mode_switch(const struct device *dev, uint8_t oper_mode, bool hidden_ssid,
			    uint8_t max_num_sta)
{
	sl_wifi_device_configuration_t nwp_config;
	int status;

	status = siwx91x_ncp_get_config(dev, &nwp_config, oper_mode, hidden_ssid, max_num_sta);
	if (status < 0) {
		return status;
	}

	status = sl_wifi_deinit();
	if (status != SL_STATUS_OK) {
		return -ETIMEDOUT;
	}

	status = sl_wifi_init(&nwp_config, NULL, sl_wifi_default_event_handler);
	if (status != SL_STATUS_OK) {
		return -ETIMEDOUT;
	}

	return 0;
}

/* ========================================================================== */
/* Device init                                                                 */
/* ========================================================================== */

static int siwx91x_nwp_ncp_init(const struct device *dev)
{
	struct siwx91x_ncp_data *data = dev->data;
	sl_wifi_device_configuration_t network_config;
	int ret;

	/* Store singleton reference for sl_si91x_host_*() callbacks */
	ncp_dev_instance = dev;

	/* Initialize SPI configuration from DT */
	const struct siwx91x_ncp_config *cfg = dev->config;

	data->spi_cfg = cfg->spi.config;
	data->spi_cfg.frequency = CONFIG_SIWX91X_NCP_SPI_FREQUENCY;
	data->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB;

	/* Get NWP boot configuration */
	ret = siwx91x_ncp_get_config(dev, &network_config, WIFI_STA_MODE, false, 0);
	if (ret < 0) {
		LOG_ERR("Failed to get NWP config: %d", ret);
		return ret;
	}

	/* Boot the NWP via sl_wifi_init — this triggers the SPI handshake,
	 * firmware load/verify, and NWP initialization sequence through the
	 * WiseConnect SDK.
	 */
	ret = sl_wifi_init(&network_config, NULL, sl_wifi_default_event_handler);
	if (ret != SL_STATUS_OK) {
		LOG_ERR("sl_wifi_init failed: 0x%x", ret);
		return -EINVAL;
	}

	sl_mac_address_t mac_addr;

	ret = sl_wifi_get_mac_address(SL_WIFI_CLIENT_INTERFACE, &mac_addr);
	if (ret == SL_STATUS_OK) {
		LOG_INF("SiWx91x MAC: %02x:%02x:%02x:%02x:%02x:%02x",
			mac_addr.octet[0], mac_addr.octet[1], mac_addr.octet[2],
			mac_addr.octet[3], mac_addr.octet[4], mac_addr.octet[5]);
	} else {
		LOG_WRN("Failed to read SiWx91x MAC address: 0x%x", ret);
	}

	/* Use HIGH_PERFORMANCE until sleep/wake GPIO handshaking is implemented.
	 * ASSOCIATED_POWER_SAVE requires the sleep-request and wake-indicator GPIOs
	 * to be functional — without them the NWP may sleep and never wake for SPI
	 * transactions, causing command timeouts (especially BLE).
	 */
	sl_wifi_performance_profile_v2_t wifi_profile = { .profile = HIGH_PERFORMANCE };
	ret = sl_wifi_set_performance_profile_v2(&wifi_profile);
	if (ret != SL_STATUS_OK) {
		LOG_ERR("Failed to set WiFi performance profile: 0x%x", ret);
	}

#if defined(CONFIG_BT_SILABS_SIWX91X) || defined(CONFIG_SIWX91X_NCP_BLE_RF_TEST)
	if (IS_ENABLED(CONFIG_BT_SILABS_SIWX91X) || IS_ENABLED(CONFIG_SIWX91X_NCP_BLE_RF_TEST)) {
		sl_bt_performance_profile_t bt_profile = { .profile = HIGH_PERFORMANCE };
		ret = sl_si91x_bt_set_performance_profile(&bt_profile);
		if (ret != SL_STATUS_OK) {
			LOG_ERR("Failed to set BT performance profile: 0x%x", ret);
		}
	}
#endif

	/* Check firmware version */
	ret = siwx91x_ncp_check_fw_version();
	if (ret < 0) {
		//LOG_ERR("Unexpected NWP firmware version (expected: %s)",
			//SIWX91X_NWP_FW_EXPECTED_VERSION);
		/* Continue — version mismatch is a warning, not fatal */
	}

	LOG_INF("SiWx91x NCP NWP initialized successfully");
	siwx91x_wifi_tx_test_set_defaults(&data->wifi_tx_test_cfg);
	data->wifi_tx_test_running = false;
#if defined(CONFIG_BT_SILABS_SIWX91X) || defined(CONFIG_SIWX91X_NCP_BLE_RF_TEST)
	siwx91x_ble_per_tx_test_set_defaults(&data->ble_per_tx_cfg);
	data->ble_per_tx_test_running = false;
	data->ble_stack_initialized = false;
	data->ble_radio_disabled = false;
#endif

	return 0;
}

/* ========================================================================== */
/* Device instantiation                                                        */
/* ========================================================================== */

#define SIWX91X_NWP_NCP_DEFINE(inst)                                                          \
                                                                                               \
	static struct siwx91x_ncp_data siwx91x_ncp_data_##inst = {};                          \
                                                                                               \
	static const struct siwx91x_ncp_config siwx91x_ncp_config_##inst = {                   \
		.spi = SPI_DT_SPEC_INST_GET(inst,                                              \
			SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB),              \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                        \
		.cs_gpio = GPIO_DT_SPEC_INST_GET(inst, cs_gpios),                                \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                            \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, sleep_request_gpios),                   \
			(.sleep_gpio = GPIO_DT_SPEC_INST_GET(inst, sleep_request_gpios),))     \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, wake_indicator_gpios),                  \
			(.wake_gpio = GPIO_DT_SPEC_INST_GET(inst, wake_indicator_gpios),))     \
	};                                                                                     \
                                                                                               \
	DEVICE_DT_INST_DEFINE(inst, &siwx91x_nwp_ncp_init, NULL,                               \
			      &siwx91x_ncp_data_##inst, &siwx91x_ncp_config_##inst,            \
			      POST_KERNEL, CONFIG_SIWX91X_NCP_SPI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_NWP_NCP_DEFINE)
