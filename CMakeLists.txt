# Copyright (c) 2026 Guilherme Luiz Moritz
# SPDX-License-Identifier: Apache-2.0
#
# labsc_filament_oven — safety-oriented oven controller (Zephyr + ESP32-C6).
# See docs/ARCHITECTURE.md for the design this firmware implements.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(labsc_filament_oven C)

target_sources(app PRIVATE
  src/main.c
  src/shared.c
  src/fault.c
  src/oven_state.c
  src/pid.c
  src/pid_pub.c
  src/pid_thread.c
  src/output_thread.c
  src/watchdog_thread.c
  src/shell_cmds.c
)

# Bench diagnostics consumer. The publication side (src/pid_pub.c) is in every
# image by design — see pid_pub.h; only the reader is optional.
target_sources_ifdef(CONFIG_OVEN_DEBUG app PRIVATE src/oven_debug.c)

# --- Workaround: hal_espressif ESP32-C6 SPI HAL bug --------------------------
# In the hal_espressif revision pinned by this container's Zephyr (main HEAD),
# zephyr/esp32c6/CMakeLists.txt compiles esp_hal_gpspi/spi_hal.c unconditionally
# but gates the per-SoC esp32c6/spi_periph.c (which *defines* spi_periph_signal)
# behind CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP — a symbol that does not
# exist for the C6, so it is never true. Result: any C6 SPI build fails to link
# with an undefined reference to spi_periph_signal. The two MAX6675 thermocouple
# readers ride SPI, so we hit it. Add the missing source ourselves.
# Remove once the container's Zephyr/hal_espressif revision fixes this.
if(CONFIG_SOC_SERIES_ESP32C6 AND CONFIG_SPI)
  zephyr_library_sources(
    ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/esp_hal_gpspi/${CONFIG_SOC_SERIES}/spi_periph.c)
endif()
