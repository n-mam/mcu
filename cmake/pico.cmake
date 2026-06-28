pico_sdk_init()

add_compile_definitions(PICO)

if(PICO_CYW43_SUPPORTED)
    add_compile_definitions(PICO_CYW43_SUPPORTED)
endif()

