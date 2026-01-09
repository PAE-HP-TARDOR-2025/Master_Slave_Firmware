idf_component_register(SRCS "../../dummy_slave_main.c" INCLUDE_DIRS "")

# Allow callers to override the greeting from either CMake (-DSLAVE_GREETING_OVERRIDE)
# or the environment (SLAVE_GREETING_OVERRIDE); fall back to Kconfig otherwise.
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
