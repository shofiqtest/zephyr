/*
 * Copyright (c) 2026 Md Shofiqul Islam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max30001

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "max30001.h"

LOG_MODULE_REGISTER(MAX30001, CONFIG_SENSOR_LOG_LEVEL);

/* CNFG_ECG GAIN field (bits [17:16]) encoding -> V/V. See max30001.h. */
static const uint16_t max30001_ecg_gain_v_per_v[] = {
	[0] = 20,
	[1] = 40,
	[2] = 80,
	[3] = 160,
};

/* CNFG_BIOZ GAIN field (bits [17:16]) encoding -> V/V. Different from the
 * ECG table above -- see the comment on MAX30001_CNFG_BIOZ_GAIN_MASK.
 */
static const uint16_t max30001_bioz_gain_v_per_v[] = {
	[0] = 10,
	[1] = 20,
	[2] = 40,
	[3] = 80,
};

static int max30001_reg_read(const struct device *dev, uint8_t reg, uint32_t *val)
{
	const struct max30001_config *cfg = dev->config;
	uint8_t tx[MAX30001_REG_XFER_BYTES] = {
		MAX30001_CMD_BYTE(reg, MAX30001_RW_READ),
		0, 0, 0,
	};
	uint8_t rx[MAX30001_REG_XFER_BYTES] = {0};
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret;

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret) {
		LOG_ERR("SPI read of reg 0x%02x failed: %d", reg, ret);
		return ret;
	}

	/* 3 data bytes follow the command byte, MSB first -> 24-bit register value. */
	*val = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];

	return 0;
}

static int max30001_reg_write(const struct device *dev, uint8_t reg, uint32_t val)
{
	const struct max30001_config *cfg = dev->config;
	uint8_t tx[MAX30001_REG_XFER_BYTES] = {
		MAX30001_CMD_BYTE(reg, MAX30001_RW_WRITE),
		(uint8_t)((val >> 16) & 0xFF),
		(uint8_t)((val >> 8) & 0xFF),
		(uint8_t)(val & 0xFF),
	};
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	int ret;

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret) {
		LOG_ERR("SPI write of reg 0x%02x failed: %d", reg, ret);
	}

	return ret;
}

/*
 * Decode one ECG_FIFO word: an 18-bit left-justified two's complement sample
 * in bits [23:6], an ETAG status field in bits [5:3], and a PTAG (pace) field
 * in bits [2:0]. Only ETAG_VALID samples are meaningful data; other ETAG
 * values mark FIFO-empty/overflow/fast-mode conditions (not yet handled here
 * -- this first pass only reads the sample and ignores the tag beyond a
 * basic validity check).
 */
static int32_t max30001_decode_ecg_sample(uint32_t fifo_word)
{
	uint32_t etag = FIELD_GET(MAX30001_ECG_FIFO_ETAG_MASK, fifo_word);
	int32_t sample18;

	if (etag != MAX30001_ECG_FIFO_ETAG_VALID) {
		return 0;
	}

	sample18 = (int32_t)(fifo_word >> MAX30001_ECG_FIFO_SAMPLE_SHIFT);
	return sign_extend(sample18, MAX30001_ECG_FIFO_SAMPLE_BITS - 1);
}

/* BioZ FIFO word: same 18-bit-sample-plus-tag shape as ECG. See the
 * MAX30001_BIOZ_FIFO_SAMPLE_SHIFT comment in max30001.h for how this was
 * derived. No documented ETAG-equivalent validity encoding was confirmed
 * for BTAG this session, so unlike the ECG decoder this does not gate on
 * a tag value yet -- it decodes every sample unconditionally.
 */
static int32_t max30001_decode_bioz_sample(uint32_t fifo_word)
{
	int32_t sample18 = (int32_t)(fifo_word >> MAX30001_BIOZ_FIFO_SAMPLE_SHIFT);

	return sign_extend(sample18, MAX30001_BIOZ_FIFO_SAMPLE_BITS - 1);
}

static int max30001_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct max30001_data *data = dev->data;
	enum sensor_channel_max30001 priv_chan = (enum sensor_channel_max30001)chan;
	uint32_t fifo_word;
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_VOLTAGE &&
	    priv_chan != SENSOR_CHAN_MAX30001_BIOZ_VOLTAGE) {
		return -ENOTSUP;
	}

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {
		ret = max30001_reg_read(dev, MAX30001_REG_ECG_FIFO, &fifo_word);
		if (ret) {
			return ret;
		}
		data->ecg_sample = max30001_decode_ecg_sample(fifo_word);
	}

	if (chan == SENSOR_CHAN_ALL || priv_chan == SENSOR_CHAN_MAX30001_BIOZ_VOLTAGE) {
		ret = max30001_reg_read(dev, MAX30001_REG_BIOZ_FIFO, &fifo_word);
		if (ret) {
			return ret;
		}
		data->bioz_sample = max30001_decode_bioz_sample(fifo_word);
	}

	return 0;
}

static int max30001_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct max30001_data *data = dev->data;
	enum sensor_channel_max30001 priv_chan = (enum sensor_channel_max30001)chan;
	int64_t micro_v;

	if (chan == SENSOR_CHAN_VOLTAGE) {
		/*
		 * V_in = ADC_code * VREF / (2^17 * ECG_GAIN), verified against
		 * ADI's EngineerZone FAQ -- see the comment on
		 * MAX30001_ECG_ADC_DENOM in max30001.h. ecg_gain_v_per_v is read
		 * back from CNFG_ECG at init, not assumed, since it is
		 * runtime-configurable.
		 */
		micro_v = ((int64_t)data->ecg_sample * MAX30001_ECG_VREF_UV) /
			  ((int64_t)MAX30001_ECG_ADC_DENOM * data->ecg_gain_v_per_v);
	} else if (priv_chan == SENSOR_CHAN_MAX30001_BIOZ_VOLTAGE) {
		/* Same formula shape as ECG; see max30001.h for the caveat
		 * that this is provisional, not primary-source-verified.
		 */
		micro_v = ((int64_t)data->bioz_sample * MAX30001_ECG_VREF_UV) /
			  ((int64_t)MAX30001_ECG_ADC_DENOM * data->bioz_gain_v_per_v);
	} else {
		return -ENOTSUP;
	}

	val->val1 = (int32_t)(micro_v / 1000000);
	val->val2 = (int32_t)(micro_v % 1000000);

	return 0;
}

static DEVICE_API(sensor, max30001_driver_api) = {
	.sample_fetch = max30001_sample_fetch,
	.channel_get = max30001_channel_get,
};

static int max30001_init(const struct device *dev)
{
	const struct max30001_config *cfg = dev->config;
	struct max30001_data *data = dev->data;
	uint32_t info;
	uint32_t cnfg_reg;
	uint8_t gain_idx;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (cfg->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}
	}

	/* Software reset, in place of a hardware reset pulse if no GPIO given. */
	ret = max30001_reg_write(dev, MAX30001_REG_SW_RST, 0);
	if (ret) {
		LOG_ERR("Software reset failed: %d", ret);
		return ret;
	}

	/*
	 * Sanity check only: confirm the device responds over SPI at all.
	 * NOT a verified part-ID check -- the reference implementation this
	 * was checked against (MAX30003, the ECG-only sibling) flags the
	 * INFO register's bit layout as not fully confirmed even there, so
	 * this deliberately does not assert a specific expected value yet.
	 */
	ret = max30001_reg_read(dev, MAX30001_REG_INFO, &info);
	if (ret) {
		LOG_ERR("Failed to read INFO register: %d", ret);
		return ret;
	}
	if (info == 0x000000 || info == 0xFFFFFF) {
		LOG_ERR("INFO register read back 0x%06x, device not responding", info);
		return -ENODEV;
	}

	LOG_INF("MAX30001 INFO register: 0x%06x", info);

	/* Clear FIFOs before first use. */
	ret = max30001_reg_write(dev, MAX30001_REG_FIFO_RST, 0);
	if (ret) {
		return ret;
	}

	/*
	 * Read back CNFG_ECG's power-on-reset GAIN setting rather than
	 * assuming one, since it's runtime-configurable (no attr_set wiring
	 * for it yet in this first pass -- see the driver file header note).
	 */
	ret = max30001_reg_read(dev, MAX30001_REG_CNFG_ECG, &cnfg_reg);
	if (ret) {
		LOG_ERR("Failed to read CNFG_ECG register: %d", ret);
		return ret;
	}
	gain_idx = FIELD_GET(MAX30001_CNFG_ECG_GAIN_MASK, cnfg_reg);
	data->ecg_gain_v_per_v = max30001_ecg_gain_v_per_v[gain_idx];

	LOG_INF("MAX30001 ECG gain: %u V/V", data->ecg_gain_v_per_v);

	ret = max30001_reg_read(dev, MAX30001_REG_CNFG_BIOZ, &cnfg_reg);
	if (ret) {
		LOG_ERR("Failed to read CNFG_BIOZ register: %d", ret);
		return ret;
	}
	gain_idx = FIELD_GET(MAX30001_CNFG_BIOZ_GAIN_MASK, cnfg_reg);
	data->bioz_gain_v_per_v = max30001_bioz_gain_v_per_v[gain_idx];

	LOG_INF("MAX30001 BioZ gain: %u V/V", data->bioz_gain_v_per_v);

	return 0;
}

#define MAX30001_INIT(inst)                                                                      \
	static struct max30001_data max30001_data_##inst;                                        \
                                                                                                   \
	static const struct max30001_config max30001_config_##inst = {                          \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),        \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                  \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, max30001_init, NULL, &max30001_data_##inst,          \
				     &max30001_config_##inst, POST_KERNEL,                       \
				     CONFIG_SENSOR_INIT_PRIORITY, &max30001_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX30001_INIT)
