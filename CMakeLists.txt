cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32C3_PD)


# Add extra component directories
set(EXTRA_COMPONENT_DIRS
    ${IDF_PATH}/examples/system/console/advanced/components/cmd_system
    ${IDF_PATH}/examples/system/console/advanced/components/cmd_nvs
    ${IDF_PATH}/examples/system/console/advanced/components/cmd_wifi
)
