cmake_minimum_required(VERSION 3.21.0)
project(mst VERSION 1.0)

Option(CMAKE_BUILD_TYPE "debug")
set(CMAKE_CXX_STANDARD 17)

message(${CMAKE_C_COMPILER})
message(${CMAKE_CXX_COMPILER})

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/opencv/mingw_build/install)

find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "The Opecv lib is found!") 
endif()

set(INIPARSER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/iniparser/src/)
set(INIPARSER_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/iniparser/mingw_build/)

message(${OpenCV_INCLUDE_DIRS})
message(${INIPARSER_INCLUDE_DIR})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/aco_ws
    ${CMAKE_CURRENT_SOURCE_DIR}/mst_ws
    ${CMAKE_CURRENT_SOURCE_DIR}/pso_ws
    ${OpenCV_INCLUDE_DIRS}
    ${INIPARSER_INCLUDE_DIR}
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/common COMMON)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/aco_ws ACO_WS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/mst_ws MST_WS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/pso_ws PSO_WS)

add_library(
    mst
    ${COMMON}
    ${ACO_WS}
    ${MST_WS}
    ${PSO_WS}
)

target_link_libraries(mst ${OpenCV_LIBS} ${INIPARSER_LIBRARY_DIR}/libiniparser.a)

add_executable(
    main_test
    main_test.cpp
)

target_link_libraries( main_test mst ${OpenCV_LIBS} )
