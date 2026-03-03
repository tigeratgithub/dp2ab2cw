# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_CDC_ACM 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

# sdk_inc(../../../config)
# sdk_inc(../../../hpm_sdk/samples/cherryusb/config)

project(dp2ab2cw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

sdk_inc(D:/hpmicroSDK/sdk_env-1.11.0/hpm_sdk/samples/cherryusb/config)
sdk_inc(src)
sdk_app_src(src/mmc.c)
sdk_app_src(src/qeo_abz.c)
sdk_app_src(src/webusb_serial.c)
generate_ide_projects()

# 每次编译后自动将 build 目录中的 compile_commands.json 复制到项目根目录
add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${CMAKE_SOURCE_DIR}/compile_commands.json"
    COMMENT "Copying compile_commands.json to project root"
)

# cmake --preset "flash_xip Debug"