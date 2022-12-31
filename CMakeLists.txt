cmake_minimum_required(VERSION 3.21.0)
project(mst)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/install/)

find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "The Opecv lib is found!") 
endif()

include_directories(
    ./
    ./common
    ./aco_ws
    ./mst_ws
    ./pso_ws
    ${OpenCV_INCLUDE_DIRS}
)

aux_source_directory(./common COMMON)
aux_source_directory(./aco_ws ACO_WS)
aux_source_directory(./mst_ws MST_WS)
aux_source_directory(./pso_ws PSO_WS)

add_executable(
    main_test
    main_test.cpp
    ${COMMON}
    ${ACO_WS}
    ${MST_WS}
    ${PSO_WS}
)

target_link_libraries( main_test ${OpenCV_LIBS} )
