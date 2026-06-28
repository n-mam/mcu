add_compile_definitions(STM32F4)
add_link_options(-mcpu=cortex-m4)
add_compile_options(-mcpu=cortex-m4 -mfpu=fpv4-sp-d16)
add_compile_definitions(USBD_SOF_DISABLED USER_VECT_TAB_ADDRESS)

if(CMSIS_TARGET STREQUAL "f411")
    add_compile_definitions(mcu PRIVATE
        STM32F411xE
        HSE_VALUE=25000000
    )
elseif(CMSIS_TARGET STREQUAL "f446")
    add_compile_definitions(mcu PRIVATE
        STM32F446xx
        HSE_VALUE=8000000
    )
else()
    message(FATAL_ERROR "Unknown F4 target ${CMSIS_TARGET}")
endif()