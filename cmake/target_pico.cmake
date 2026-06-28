target_link_libraries(mcu PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_i2c
    mcu.proto.lib
)

if(PICO_CYW43_SUPPORTED)
    target_link_libraries(mcu PRIVATE
        pico_cyw43_arch_lwip_threadsafe_background
    )
endif()

pico_enable_stdio_usb(mcu 1)
pico_enable_stdio_uart(mcu 0)
pico_add_extra_outputs(mcu)