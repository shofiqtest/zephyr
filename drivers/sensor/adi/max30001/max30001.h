/*
 * Copyright (c) 2026 Md Shofiqul Islam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADI_MAX30001_MAX30001_H_
#define ZEPHYR_DRIVERS_SENSOR_ADI_MAX30001_MAX30001_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/*
 * Register addresses, verified against the MAX30003 (ECG-only sibling,
 * identical register layout for the ECG core) reference implementation at
 * https://github.com/SealHAT/MAX30003 -- not yet cross-checked against the
 * MAX30001-specific datasheet sections (BioZ registers, pace detection).
 * BioZ register addresses below are NOT yet verified -- placeholders only,
 * do not trust until checked against the real MAX30001 datasheet.
 */
#define MAX30001_REG_NO_OP          0x00 /* RW - no internal effect */
#define MAX30001_REG_STATUS         0x01 /* R  - interrupt/status flags */
#define MAX30001_REG_EN_INT         0x02 /* RW - INTB interrupt enables */
#define MAX30001_REG_EN_INT2        0x03 /* RW - INT2B interrupt enables */
#define MAX30001_REG_MNGR_INT       0x04 /* RW - interrupt management */
#define MAX30001_REG_MNGR_DYN       0x05 /* RW - dynamic modes */
#define MAX30001_REG_SW_RST         0x08 /* W  - software reset */
#define MAX30001_REG_SYNCH          0x09 /* W  - synchronize/reset FIFO */
#define MAX30001_REG_FIFO_RST       0x0A /* W  - reset all FIFOs */
#define MAX30001_REG_INFO           0x0F /* R  - part identification */
#define MAX30001_REG_CNFG_GEN       0x10 /* RW - general config */
#define MAX30001_REG_CNFG_CAL       0x12 /* RW - calibration config */
#define MAX30001_REG_CNFG_EMUX      0x14 /* RW - ECG input mux config */
#define MAX30001_REG_CNFG_ECG       0x15 /* RW - ECG channel config (gain, rate) */
#define MAX30001_REG_CNFG_RTOR1     0x1D /* RW - R-to-R detection config 1 */
#define MAX30001_REG_CNFG_RTOR2     0x1E /* RW - R-to-R detection config 2 */
#define MAX30001_REG_CNFG_BIOZ      0x18 /* RW - BioZ channel config (gain, drive current) */
#define MAX30001_REG_CNFG_BIOZ_LC   0x1A /* RW - BioZ leads-off current config */
#define MAX30001_REG_ECG_FIFO_BURST 0x20 /* R  - ECG FIFO burst read */
#define MAX30001_REG_ECG_FIFO       0x21 /* R  - ECG FIFO single read */
#define MAX30001_REG_BIOZ_FIFO_BURST 0x22 /* R  - BioZ FIFO burst read */
#define MAX30001_REG_BIOZ_FIFO      0x23 /* R  - BioZ FIFO single read */
#define MAX30001_REG_RTOR           0x25 /* R  - latest R-to-R interval */
#define MAX30001_REG_NO_OP2         0x7F /* RW - no internal effect */

/* SPI command byte: (register address << 1) | rw_bit. rw_bit: 1 = read, 0 = write. */
#define MAX30001_RW_READ  1
#define MAX30001_RW_WRITE 0
#define MAX30001_CMD_BYTE(reg, rw) (uint8_t)(((reg) << 1) | (rw))

/* Register transactions are always 1 command byte + 3 data bytes (24-bit registers). */
#define MAX30001_REG_DATA_BYTES 3
#define MAX30001_REG_XFER_BYTES (1 + MAX30001_REG_DATA_BYTES)

/*
 * STATUS register (0x01) bits -- verified against the MAX30003 reference
 * (same ECG status bits apply to MAX30001's ECG channel).
 */
#define MAX30001_STATUS_EINT  BIT(23) /* ECG FIFO interrupt */
#define MAX30001_STATUS_EOVF  BIT(22) /* ECG FIFO overflow */
#define MAX30001_STATUS_FSTINT BIT(21) /* Fast recovery mode interrupt */
#define MAX30001_STATUS_DCLOFFINT BIT(20) /* DC lead-off interrupt */
#define MAX30001_STATUS_LONINT BIT(11) /* Lead-on interrupt */
#define MAX30001_STATUS_RRINT BIT(10) /* R-to-R interrupt */
#define MAX30001_STATUS_SAMP  BIT(9)  /* Sample synchronization pulse */
#define MAX30001_STATUS_PLLINT BIT(8) /* PLL unlocked interrupt */

/*
 * ECG FIFO word format (0x20/0x21), 24-bit register:
 *   bits [23:6] = ECG sample, 18-bit left-justified two's complement
 *   bits [5:3]  = ETAG (FIFO status tag: valid sample, fast mode, overflow...)
 *   bits [2:0]  = PTAG (pace detect tag, when pace detection is enabled)
 * Verified against the MAX30003 reference driver's bit layout.
 */
#define MAX30001_ECG_FIFO_SAMPLE_SHIFT 6
#define MAX30001_ECG_FIFO_SAMPLE_BITS  18
#define MAX30001_ECG_FIFO_ETAG_MASK    GENMASK(5, 3)
#define MAX30001_ECG_FIFO_ETAG_VALID   0x0

/*
 * CNFG_ECG register (0x15) GAIN field: bits [17:16] of the 24-bit register
 * (mask 0x030000) -- NOT bits [1:0] as the field's own datasheet-local
 * "GAIN[1:0]" notation might suggest at a glance. Verified against the
 * MAX30003 reference driver's register mask (CNFGECG_GAIN = 0x030000),
 * after an AI-generated search summary first (wrongly) suggested bits
 * [1:0]; only trust the primary-source value, not the summary.
 *
 * Encoding: 0 = 20 V/V, 1 = 40 V/V, 2 = 80 V/V, 3 = 160 V/V.
 */
#define MAX30001_CNFG_ECG_GAIN_MASK GENMASK(17, 16)

/*
 * ECG voltage conversion, verified via ADI's own EngineerZone FAQ
 * (https://ez.analog.com/.../how-to-convert-max30001-ecg-adc-sample-data-to-relative-voltage):
 *
 *   V_in = ADC_code * VREF / (2^17 * ECG_GAIN)
 *
 * with VREF = 1 V fixed, 2^17 because the 18-bit two's-complement sample's
 * positive full-scale code (2^17 - 1) maps to +FSR = VREF / ECG_GAIN.
 * In microvolts: uV = ADC_code * 1,000,000 / (131072 * ECG_GAIN_V_PER_V).
 * Cross-checked: gain=20 V/V gives 1 LSB = 381.47 nV, matching the
 * EngineerZone-quoted value of 381.469 nV exactly.
 */
#define MAX30001_ECG_VREF_UV      1000000
#define MAX30001_ECG_ADC_DENOM    131072 /* 2^17 */

/*
 * BioZ FIFO word format (0x22/0x23), 24-bit register: same layout as the
 * ECG FIFO (18-bit left-justified two's complement sample in bits [23:6],
 * a 6-bit BTAG status field in bits [5:0]). Verified by reverse-engineering
 * the sign-extension arithmetic in the Protocentral MAX30001 Arduino
 * library (github.com/Protocentral/protocentral_max30001_arduino_library):
 * it masks the low 6 bits of the 3rd data byte (0xC0 keeps only bits
 * [7:6]) before sign-extending, which only makes sense if those low 6
 * bits are a tag field and bits [23:6] are the signed sample -- identical
 * in shape to the already-verified ECG FIFO word.
 */
#define MAX30001_BIOZ_FIFO_SAMPLE_SHIFT 6
#define MAX30001_BIOZ_FIFO_SAMPLE_BITS  18

/*
 * CNFG_BIOZ register (0x18) GAIN field: bits [17:16], same bit position as
 * CNFG_ECG's GAIN field but a DIFFERENT encoding -- cross-checked against
 * two independent spots in the same Protocentral library (the
 * max30001_bioz_gain_t enum AND a separate inline comment on a raw
 * register write), which agree with each other:
 *   0 = 10 V/V, 1 = 20 V/V, 2 = 40 V/V, 3 = 80 V/V.
 * NOT independently verified against ADI's primary datasheet yet (the
 * EngineerZone page and datasheet PDF both failed to fetch here) -- treat
 * as reference-library-confirmed, not primary-source-confirmed.
 */
#define MAX30001_CNFG_BIOZ_GAIN_MASK GENMASK(17, 16)

/*
 * BioZ voltage conversion: same formula shape as ECG (same ADC, same
 * VREF, same 18-bit full-scale code), with BIOZ_GAIN substituted:
 *   uV = ADC_code * 1,000,000 / (131072 * BIOZ_GAIN_V_PER_V)
 * NOT yet verified against a primary source (unlike the ECG formula,
 * which matched ADI's EngineerZone FAQ exactly) -- treat as provisional.
 *
 * This only yields a BioZ voltage, not an impedance in ohms. Converting
 * to ohms needs the actual drive current for the configured CGMAG
 * setting (register bits [6:4] of CNFG_BIOZ), and no verified per-code
 * current table was found this session (only a vague 55 nA - 96 uA
 * range from a web search, not a per-code table) -- deliberately NOT
 * implemented until that's confirmed against the real datasheet.
 */

struct max30001_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
};

struct max30001_data {
	int32_t ecg_sample;
	int32_t bioz_sample;
	/* ECG_GAIN in V/V, read back from CNFG_ECG at init; see max30001_ecg_gain_v_per_v[]. */
	uint16_t ecg_gain_v_per_v;
	/* BIOZ_GAIN in V/V, read back from CNFG_BIOZ at init; see max30001_bioz_gain_v_per_v[]. */
	uint16_t bioz_gain_v_per_v;
};

/* Vendor-specific channel: BioZ voltage. No standard Zephyr sensor channel
 * fits (it is not a generic voltage/current/impedance channel), so this
 * follows the standard Zephyr convention for a driver-private channel.
 */
enum sensor_channel_max30001 {
	SENSOR_CHAN_MAX30001_BIOZ_VOLTAGE = SENSOR_CHAN_PRIV_START,
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ADI_MAX30001_MAX30001_H_ */
