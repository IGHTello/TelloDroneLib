cmake_minimum_required(VERSION 3.16)
project(TelloDrone)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions")

set(LIB_NAME TelloDroneLib)
set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Lib")
set(TESTS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Tests")

file(GLOB_RECURSE LIB_SOURCES ${LIB_PATH}/*.cpp)
file(GLOB TEST_SOURCES ${TESTS_PATH}/*.cpp)

add_library(${LIB_NAME} Lib/TelloDrone.cpp Lib/TelloDrone.h Lib/DronePacket.cpp Lib/DronePacket.h Lib/Utils/Types.h Lib/Utils/CRCHelpers.h)
target_link_libraries(${LIB_NAME} pthread)

foreach(test_source_file ${TEST_SOURCES})
    get_filename_component(test_name ${test_source_file} NAME_WE)
    add_executable(${test_name} ${test_source_file})
    target_link_libraries(${test_name} ${LIB_NAME})
    target_include_directories(${test_name} PUBLIC ${LIB_PATH})
endforeach()
