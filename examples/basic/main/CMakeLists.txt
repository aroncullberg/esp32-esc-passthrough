# SPDX-License-Identifier: GPL-3.0-or-later
get_filename_component(PASSTHROUGH_COMPONENT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
get_filename_component(PASSTHROUGH_COMPONENT_NAME "${PASSTHROUGH_COMPONENT_ROOT}" NAME)
idf_component_register(SRCS "main.c" INCLUDE_DIRS "../../common"
    REQUIRES ${PASSTHROUGH_COMPONENT_NAME}
    PRIV_REQUIRES esp_driver_usb_serial_jtag esp_hw_support esp_timer)
