idf_component_register(
    SRCS
        "../../dummy_slave_main.c"
        "fw_update_server.c"
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        nvs_flash
        esp_timer
        driver
        canopennode
        app_update
)

# Allow developers and automation to override the greeting without touching sources.
set(_slave_greeting_text "${CONFIG_SLAVE_GREETING}")
if(DEFINED SLAVE_GREETING_OVERRIDE AND NOT "${SLAVE_GREETING_OVERRIDE}" STREQUAL "")
    set(_slave_greeting_text "${SLAVE_GREETING_OVERRIDE}")
elseif(DEFINED ENV{SLAVE_GREETING_OVERRIDE} AND NOT "$ENV{SLAVE_GREETING_OVERRIDE}" STREQUAL "")
    set(_slave_greeting_text "$ENV{SLAVE_GREETING_OVERRIDE}")
endif()

if(_slave_greeting_text STREQUAL "")
    set(_slave_greeting_text "Hello from slave")
endif()

string(REPLACE "\"" "\\\"" _slave_greeting_escaped "${_slave_greeting_text}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE SLAVE_GREETING="${_slave_greeting_escaped}")
