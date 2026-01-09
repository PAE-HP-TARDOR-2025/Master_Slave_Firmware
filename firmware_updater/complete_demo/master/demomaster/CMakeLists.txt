# ESP-IDF Master Demo Project
# This project builds the CANopen master firmware uploader for ESP32

cmake_minimum_required(VERSION 3.16)

# Reference the CANopenNode component from root folder
get_filename_component(ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${ROOT_DIR}/canopennode")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(master_uploader)

# Package the contents of storage as a SPIFFS partition image
set(SPIFFS_IMAGE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/storage")

# Try to sync slave_fw.bin from slave artifacts
set(SLAVE_ARTIFACT "${CMAKE_CURRENT_LIST_DIR}/../../slave/artifacts/slave_fw.bin")
set(SPIFFS_FW_DEST "${SPIFFS_IMAGE_SOURCE_DIR}/slave_fw.bin")

if(EXISTS "${SLAVE_ARTIFACT}")
    set(SPIFFS_SYNC_STAMP "${CMAKE_CURRENT_BINARY_DIR}/.spiffs_fw_stamp")
    add_custom_command(
        OUTPUT "${SPIFFS_SYNC_STAMP}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SLAVE_ARTIFACT}" "${SPIFFS_FW_DEST}"
        COMMAND ${CMAKE_COMMAND} -E touch "${SPIFFS_SYNC_STAMP}"
        DEPENDS "${SLAVE_ARTIFACT}"
        COMMENT "Syncing slave firmware into SPIFFS payload"
        VERBATIM)
    add_custom_target(sync_spiffs_payload ALL DEPENDS "${SPIFFS_SYNC_STAMP}")
else()
    message(WARNING "Slave artifact not found at ${SLAVE_ARTIFACT}. Using previously staged SPIFFS payload.")
endif()

spiffs_create_partition_image(storage ${SPIFFS_IMAGE_SOURCE_DIR} FLASH_IN_PROJECT)

if(TARGET sync_spiffs_payload)
    add_dependencies(spiffs_storage_bin sync_spiffs_payload)
endif()
