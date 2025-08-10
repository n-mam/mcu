cmake_minimum_required(VERSION 3.13)

if (PLATFORM STREQUAL "PICO")
    include(D:/pico-sdk/pico_sdk_init.cmake)
endif()

project(mcu C CXX ASM)

if (PLATFORM STREQUAL "PICO")
    pico_sdk_init()
endif()

set(CMAKE_MODULE_PATH ${NANOPB_SRC_ROOT_FOLDER}/extra)

if (PLATFORM STREQUAL "PICO")
    add_compile_definitions(PICO)
    if (PICO_CYW43_SUPPORTED)
        add_compile_definitions(PICO_CYW43_SUPPORTED)
    endif()
    find_package(Nanopb REQUIRED)
    nanopb_generate_cpp(TARGET mcu.proto.lib ${CMAKE_CURRENT_SOURCE_DIR}/npb/mcu.proto)
else()
    add_compile_definitions(STM32)
    if (NOT CMSIS_ROOT OR NOT CMSIS_LINE)
        message(FATAL_ERROR "Error: Please ensure that CMSIS_ROOT, CMSIS_LINE and CMSIS_TARGET are set")
    endif()
    set (CMAKE_EXECUTABLE_SUFFIX ".elf")
    set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
    # Set common  options
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
        -T${CMAKE_CURRENT_LIST_DIR}/mcl/${CMSIS_LINE}/${CMSIS_CORE}/linker.ld
        -specs=rdimon.specs -lc -lrdimon #--> uncomment this for semihosting
    )
    # Set target-specific options
    if (CMSIS_LINE STREQUAL "f4")
        add_compile_definitions(STM32F4)
        add_link_options(-mcpu=cortex-m4)
        add_compile_options(-mcpu=cortex-m4 -mfpu=fpv4-sp-d16)
        add_compile_definitions(USBD_SOF_DISABLED USER_VECT_TAB_ADDRESS)
        if (CMSIS_TARGET STREQUAL "f411")
            add_compile_definitions(STM32F411xE HSE_VALUE=25000000)
        elseif (CMSIS_TARGET STREQUAL "f446")
            add_compile_definitions(STM32F446xx HSE_VALUE=8000000)
        else()
            message(FATAL_ERROR "Error: unknown CMSIS_TARGET")
        endif()
    elseif (CMSIS_LINE STREQUAL "f7")
        add_compile_definitions(STM32F7)
        add_link_options(-mcpu=cortex-m7)
        add_compile_options(-mcpu=cortex-m7 -mfpu=fpv5-d16)
        if (CMSIS_TARGET STREQUAL "f767")
            add_compile_definitions(STM32F767xx HSE_VALUE=8000000)
        endif()
    elseif (CMSIS_LINE STREQUAL "h7")
        add_compile_definitions(STM32H7)
        if (CMSIS_CORE STREQUAL "CM4")
            add_compile_definitions(CORE_CM4 VIRTIO_SLAVE_ONLY)
            add_compile_options(-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
            add_link_options(-mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)
        elseif(CMSIS_CORE STREQUAL "CM7")
            add_compile_definitions(CORE_CM7 VIRTIO_MASTER_ONLY)
            add_compile_options(-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard)
            add_link_options(-mcpu=cortex-m7 --specs=nano.specs  -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb)
        endif()
        add_compile_definitions(
            STM32H755xx
            USE_HAL_DRIVER
            METAL_INTERNAL
            NO_ATOMIC_64_SUPPORT
            RPMSG_BUFFER_SIZE=512
            METAL_MAX_DEVICE_REGIONS=2
        )
    else()
        message(FATAL_ERROR "Error: unknown CMSIS_LINE")
    endif()
    find_package(Nanopb REQUIRED)
    nanopb_generate_cpp(TARGET mcu.proto.lib ${CMAKE_CURRENT_SOURCE_DIR}/npb/mcu.proto)
endif()

add_executable(mcu)

target_compile_features(mcu PUBLIC cxx_std_20)

target_sources(
    mcu
    PRIVATE
    main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sns/imu/sh2/sh2.c
    ${CMAKE_CURRENT_LIST_DIR}/sns/imu/sh2/shtp.c
    ${CMAKE_CURRENT_LIST_DIR}/sns/imu/sh2/euler.c
    ${CMAKE_CURRENT_LIST_DIR}/sns/imu/sh2/sh2_util.c
    ${CMAKE_CURRENT_LIST_DIR}/sns/imu/bno055/bno055.c
    ${CMAKE_CURRENT_LIST_DIR}/sns/imu/sh2/sh2_SensorValue.c
    ${CMAKE_CURRENT_LIST_DIR}/sns/imu/bno055/bno055_support.c
)

target_include_directories(
    mcu
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/mcl
    ${CMAKE_CURRENT_LIST_DIR}/sns
    ${CMAKE_CURRENT_LIST_DIR}/net
    ${CMAKE_CURRENT_LIST_DIR}/pid
)

if (PLATFORM STREQUAL "PICO")
    # pull in common dependencies and additional pwm hardware support
    target_link_libraries(mcu pico_stdlib hardware_pwm hardware_i2c mcu.proto.lib)
    if (PICO_CYW43_SUPPORTED)
        target_link_libraries(mcu pico_cyw43_arch_lwip_threadsafe_background)
    endif()
    # create map/bin/hex file etc.
    pico_add_extra_outputs(mcu)
    # add url via pico_set_program_url
    #example_auto_set_url(mcu)
    # enable usb output
    pico_enable_stdio_usb(mcu 1)
    # disable uart output
    pico_enable_stdio_uart(mcu 0)
else()
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
        #${CMAKE_SOURCE_DIR}/mcl/fx/syscalls.c #comment this for semihosting
    )
    target_include_directories(
        mcu
        PRIVATE
        ${CMAKE_SOURCE_DIR}/mcl/fx
        ${CMSIS_ROOT}/CMSIS/Core/Include
        #${CMAKE_SOURCE_DIR}/mcl/${CMSIS_LINE}
        ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Include
    )
    set_target_properties(mcu PROPERTIES LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/mcl/${CMSIS_LINE}/${CMSIS_CORE}/linker.ld
    )
    target_link_libraries(mcu mcu.proto.lib)
    if (CMSIS_TARGET STREQUAL "f411")
        target_link_libraries(mcu cdc)
    endif()
    # add_custom_command(TARGET mcu POST_BUILD
    #     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/build
    #     COMMAND ${TOOLCHAIN_PREFIX}/bin/arm-none-eabi-objcopy ARGS -S -O binary mcu.elf mcu.bin
    #)
endif()