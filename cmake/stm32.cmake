add_compile_definitions(STM32)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

add_compile_options(
    -mthumb
    -c -MMD -MP
    -fno-exceptions
    --specs=nano.specs
    -ffunction-sections
)

add_link_options(
    --specs=nosys.specs
    -Wl,-Map=mcu.map -Wl,--gc-sections
    -T${CMAKE_SOURCE_DIR}/mcl/${CMSIS_LINE}/${CMSIS_CORE}/linker.ld
    -specs=rdimon.specs -lc -lrdimon #--> uncomment this for semihosting
)

include(cmake/stm32_${CMSIS_LINE}.cmake)