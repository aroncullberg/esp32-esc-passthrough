# SPDX-License-Identifier: GPL-3.0-or-later
idf_component_register(SRCS "main.cpp" INCLUDE_DIRS "." "../../common"
    PRIV_REQUIRES esp_driver_usb_serial_jtag esp_hw_support esp_timer)
# DShot/passthrough requirements are injected from idf_component.yml.
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
