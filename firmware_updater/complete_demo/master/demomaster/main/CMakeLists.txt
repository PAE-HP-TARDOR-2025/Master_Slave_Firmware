# Component registration for master uploader
# Uses sources from parent directory and CANopenNode component

get_filename_component(MASTER_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

idf_component_register(
    SRCS
        "master_main.c"
        "${MASTER_SRC_DIR}/fw_master_update.c"
    PRIV_REQUIRES
        spi_flash
        nvs_flash
        spiffs
        esp_timer
        driver
        canopennode
    INCLUDE_DIRS
        "."
        "${MASTER_SRC_DIR}"
)
