cmake_minimum_required(VERSION 3.16)

# Allow greeting override via environment variable
if(DEFINED ENV{SLAVE_GREETING_OVERRIDE})
    add_compile_definitions(SLAVE_GREETING="$ENV{SLAVE_GREETING_OVERRIDE}")
endif()

# Pull in CANopenNode from root folder
get_filename_component(ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${ROOT_DIR}/canopennode")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(demoslave)
