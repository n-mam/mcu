set(CMAKE_MODULE_PATH ${NANOPB_SRC_ROOT_FOLDER}/extra)

find_package(Nanopb REQUIRED)

nanopb_generate_cpp(
    TARGET mcu.proto.lib
    ${CMAKE_CURRENT_SOURCE_DIR}/npb/mcu.proto
)