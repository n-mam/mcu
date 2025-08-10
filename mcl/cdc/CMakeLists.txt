file(GLOB SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/mcl/cdc/src/*.c
    ${CMAKE_SOURCE_DIR}/mcl/cdc/src/*.S
    ${CMAKE_SOURCE_DIR}/mcl/cdc/demo/*.c
)

add_library(cdc STATIC ${SOURCE_FILES})

target_include_directories(
    cdc
    PRIVATE
    ${NANOPB_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/mcl/cdc/inc
    ${CMSIS_ROOT}/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/mcl/${CMSIS_LINE}
    ${CMSIS_ROOT}/Device/ST/cmsis_device_${CMSIS_LINE}/Include
)

target_link_libraries(cdc mcu.proto.lib)