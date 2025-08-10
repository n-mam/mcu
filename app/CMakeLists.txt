cmake_minimum_required(VERSION 3.13)

SET(CMAKE_VERBOSE_MAKEFILE ON)

project(app C CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt6 REQUIRED COMPONENTS Quick QuickControls2 Gui SerialPort)

set(CMAKE_MODULE_PATH ${NANOPB_SRC_ROOT_FOLDER}/extra)
find_package(Nanopb REQUIRED)
nanopb_generate_cpp(TARGET mcu.proto.lib ${CMAKE_SOURCE_DIR}/../npb/mcu.proto)

qt6_add_resources(QT_RESOURCES_APP
    ${CMAKE_CURRENT_SOURCE_DIR}/qml/qml.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/images/images.qrc
)

add_executable(app ${QT_RESOURCES_APP}
    models/ServerManager.h
    models/TcpServerManager.h
    models/UdpServerManager.h
    models/SerialPortManager.h
)

set_property(TARGET app PROPERTY CXX_STANDARD 20)

target_sources(
    app
    PRIVATE
    main.cpp
    ${CMAKE_SOURCE_DIR}/models/ServerManager.cpp
    ${CMAKE_SOURCE_DIR}/models/TcpServerManager.cpp
    ${CMAKE_SOURCE_DIR}/models/UdpServerManager.cpp
    ${CMAKE_SOURCE_DIR}/models/SerialPortmanager.cpp
)

target_include_directories(
    app
    PRIVATE
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/models
)

target_link_libraries(
    app
    PRIVATE
    Qt6::Gui
    Qt6::Core
    Qt6::Quick
    mcu.proto.lib
    Qt6::SerialPort
    Qt6::QuickControls2
)

if (WIN32)
    SET (CMAKE_CXX_FLAGS_RELEASE "/Zi /Od")
    SET_TARGET_PROPERTIES(app PROPERTIES LINK_FLAGS "/DEBUG")
endif (WIN32)