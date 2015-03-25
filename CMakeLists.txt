include(ExternalProject)

cmake_minimum_required(VERSION 2.8)

#set the project name and version
project(CovReg)
set(localizer_VERSION_MAJOR 0)
set(localizer_VERSION_MINOR 1)

#set the debug flag and enable all warnings
set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unknown-pragmas")

#set the required libraries
#find_package(Eigen3 REQUIRED)
find_package (Threads)
find_package(PCL 1.7 REQUIRED)
find_package(GSL REQUIRED)

# HACK! Should probably move a copy of flirt into /external
ExternalProject_Add(flirt
PREFIX "/home/humphreh/Software/flirtlib/trunk"
SOURCE_DIR "/home/humphreh/Software/flirtlib/trunk")

ExternalProject_Get_Property(flirt CMAKE_INSTALL_PREFIX)
set(flirt_INCLUDE_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/include/flirtlib")
include_directories(${flirt_INCLUDE_DIRECTORIES})
message(STATUS "FLIRT found at: ${flirt_INCLUDE_DIRECTORIES}")

set(flirt_LIBRARY_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib/flirtlib")
message(STATUS ${flirt_LIBRARY_DIRECTORIES})
file(GLOB flirt_LIBRARIES "${flirt_LIBRARY_DIRECTORIES}/*.so")
message(STATUS "FLIRT libraries found at: ${flirt_LIBRARY_DIRECTORIES}")

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${GSL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_subdirectory(src)
add_subdirectory(workflow)