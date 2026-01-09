cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{IDF_PATH})
    message(FATAL_ERROR "IDF_PATH environment variable is not set. Run the ESP-IDF export script first.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dummy_slave_demo)
