target_sources(mcu PRIVATE
    main.cpp
    sns/imu/sh2/sh2.c
    sns/imu/sh2/shtp.c
    sns/imu/sh2/euler.c
    sns/imu/sh2/sh2_util.c
    sns/imu/bno055/bno055.c
    sns/imu/sh2/sh2_SensorValue.c
    sns/imu/bno055/bno055_support.c
)

target_include_directories(mcu PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/sns
    ${CMAKE_SOURCE_DIR}/net
    ${CMAKE_SOURCE_DIR}/pid
    ${CMAKE_SOURCE_DIR}/mcl
    ${CMAKE_SOURCE_DIR}/mcl/fx
    ${CMSIS_ROOT}/CMSIS/Core/Include
    ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Include
)

if(PLATFORM STREQUAL "PICO")
    include(cmake/target_pico.cmake)
elseif(PLATFORM STREQUAL "STM32")
    include(cmake/target_stm32.cmake)
else()
    message(FATAL_ERROR "Invalid PLATFORM")
endif()

target_link_libraries(mcu PRIVATE mcu.proto.lib)