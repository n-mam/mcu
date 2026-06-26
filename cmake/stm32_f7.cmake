add_compile_definitions(STM32F7)
add_link_options(-mcpu=cortex-m7)
add_compile_options(-mcpu=cortex-m7 -mfpu=fpv5-d16)

if (CMSIS_TARGET STREQUAL "f767")
    add_compile_definitions(STM32F767xx HSE_VALUE=8000000)
endif()