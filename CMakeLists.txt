cmake_minimum_required(VERSION 3.16)
project(TelloDrone)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Werror)
add_compile_options(-fno-exceptions)

set(LIB_NAME TelloDroneLib)
set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Lib")
set(DEMOS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Demos")

file(GLOB_RECURSE LIB_SOURCES ${LIB_PATH}/*.cpp)
file(GLOB DEMOS_SOURCES ${DEMOS_PATH}/*.cpp)

add_library(${LIB_NAME} Lib/TelloDrone.cpp Lib/TelloDrone.h Lib/DronePacket.cpp Lib/DronePacket.h Lib/Utils/Types.h Lib/Utils/CRCHelpers.h)
target_link_libraries(${LIB_NAME} pthread)

foreach(demo_source_file ${DEMOS_SOURCES})
    get_filename_component(demo_name ${demo_source_file} NAME_WE)
    add_executable(${demo_name} ${demo_source_file})
    target_link_libraries(${demo_name} ${LIB_NAME})
    target_include_directories(${demo_name} PUBLIC ${LIB_PATH})
endforeach()
