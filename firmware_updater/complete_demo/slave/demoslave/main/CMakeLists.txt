get_filename_component(SLAVE_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

idf_component_register(
    SRCS
        "${SLAVE_LIB_DIR}/slave_main.c"
        "${SLAVE_LIB_DIR}/fw_update_server.c"
        "${SLAVE_LIB_DIR}/fw_slave_update.c"
        "${SLAVE_LIB_DIR}/OD.c"
    INCLUDE_DIRS
        "${SLAVE_LIB_DIR}"
    REQUIRES
        driver
        esp_timer
        nvs_flash
        app_update
        esp_partition
        canopennode
)
