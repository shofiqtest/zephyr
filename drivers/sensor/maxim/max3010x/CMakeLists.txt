# Makefile - MAX30101/MAX30102 heart rate and pulse oximeter sensor series
#
# Copyright (c) 2017, NXP
# Copyright (c) 2025, CATIE
# Copyright (c) 2026, Md Shofiqul Islam
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MAX3010X max3010x.c)

zephyr_library_sources_ifdef(CONFIG_MAX3010X_TRIGGER max3010x_trigger.c)
