# target-specific sources
if (CMSIS_LINE STREQUAL "f4")
    list(APPEND SOURCE_FILES ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Source/Templates/system_stm32f4xx.c)
    if (CMSIS_TARGET STREQUAL "f411")
        add_subdirectory(mcl/cdc) # usb-cdc lib
        list(APPEND SOURCE_FILES ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Source/Templates/gcc/startup_stm32f411xe.s)
    elseif (CMSIS_TARGET STREQUAL "f446")
        list(APPEND SOURCE_FILES ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Source/Templates/gcc/startup_stm32f446xx.s)
    endif()
elseif (CMSIS_LINE STREQUAL "f7")
    list(APPEND SOURCE_FILES ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Source/Templates/system_stm32f7xx.c)
    if (CMSIS_TARGET STREQUAL "f767")
        list(APPEND SOURCE_FILES ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Source/Templates/gcc/startup_stm32f767xx.s)
    endif()
elseif (CMSIS_LINE STREQUAL "h7")
    # common to cm4 and cm7
    list(APPEND SOURCE_FILES
        ${CMAKE_SOURCE_DIR}/mcl/h7/${CMSIS_CORE}/core.c
        ${CMAKE_SOURCE_DIR}/mcl/h7/startup_stm32h755zitx.s
        ${CMAKE_SOURCE_DIR}/mcl/h7/system_stm32h7xx_dualcore_boot_cm4_cm7.c
    )
    file(GLOB_RECURSE IPC_SOURCES ${CMAKE_SOURCE_DIR}/mcl/h7/Middlewares/*.c)
    file(GLOB H7_DRIVERS ${CMAKE_SOURCE_DIR}/mcl/h7/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
    list(APPEND SOURCE_FILES ${H7_DRIVERS} ${IPC_SOURCES})
    target_include_directories(mcu PRIVATE
        ${CMAKE_SOURCE_DIR}/mcl/h7
        ${CMAKE_SOURCE_DIR}/mcl/h7/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_SOURCE_DIR}/mcl/h7/Drivers/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/mcl/h7/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/mcl/h7/Drivers/STM32H7xx_HAL_Driver/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/mcl/h7/Middlewares
        ${CMAKE_SOURCE_DIR}/mcl/h7/Middlewares/Third_Party/OpenAMP/libmetal/lib/include
        ${CMAKE_SOURCE_DIR}/mcl/h7/Middlewares/Third_Party/OpenAMP/open-amp/lib/include
    )
endif()

target_sources(
    mcu
    PRIVATE
    ${SOURCE_FILES}
    ${CMAKE_SOURCE_DIR}/mcl/fx/syscalls.c #comment this for semihosting
)

target_include_directories(
    mcu
    PRIVATE
    ${CMAKE_SOURCE_DIR}/mcl/fx
    ${CMSIS_ROOT}/CMSIS/Core/Include
    ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Include
)

set_target_properties(mcu PROPERTIES LINK_DEPENDS
    ${CMAKE_SOURCE_DIR}/mcl/${CMSIS_LINE}/${CMSIS_CORE}/linker.ld
)

if (CMSIS_TARGET STREQUAL "f411")
    target_link_libraries(mcu PRIVATE cdc)
endif()

set_target_properties(mcu PROPERTIES LINK_DEPENDS
    ${CMAKE_SOURCE_DIR}/mcl/${CMSIS_LINE}/${CMSIS_CORE}/linker.ld
)