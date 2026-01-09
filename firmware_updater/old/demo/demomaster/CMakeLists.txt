# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
    
# Pull in the CANopenNode component that now lives inside this demo tree.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/canopennode")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(demomaster)

# Package the contents of demo/demomaster/storage as a SPIFFS partition image.
set(SPIFFS_IMAGE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/storage")
set(SLAVE_ARTIFACT_BYE "${CMAKE_CURRENT_LIST_DIR}/../demoslave/artifacts/bye.bin")
set(SPIFFS_BYE_DEST "${SPIFFS_IMAGE_SOURCE_DIR}/bye.bin")

if(EXISTS "${SLAVE_ARTIFACT_BYE}")
	set(SPIFFS_SYNC_STAMP "${CMAKE_CURRENT_BINARY_DIR}/.spiffs_bye_stamp")
	add_custom_command(
		OUTPUT "${SPIFFS_SYNC_STAMP}"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SLAVE_ARTIFACT_BYE}" "${SPIFFS_BYE_DEST}"
		COMMAND ${CMAKE_COMMAND} -E touch "${SPIFFS_SYNC_STAMP}"
		DEPENDS "${SLAVE_ARTIFACT_BYE}"
		COMMENT "Syncing demoslave bye.bin into SPIFFS payload"
		VERBATIM)
	add_custom_target(sync_spiffs_payload ALL DEPENDS "${SPIFFS_SYNC_STAMP}")
else()
	message(WARNING "Slave artifact ${SLAVE_ARTIFACT_BYE} not found. Using previously staged SPIFFS payload.")
endif()

spiffs_create_partition_image(storage ${SPIFFS_IMAGE_SOURCE_DIR} FLASH_IN_PROJECT)

if(TARGET sync_spiffs_payload)
	add_dependencies(spiffs_storage_bin sync_spiffs_payload)
endif()
