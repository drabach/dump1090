# Copyright 2013 A. Bach
#
# This file is part of dump1090
#


########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.6)
project(dump1090)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

########################################################################
# Compiler specific setup
########################################################################
if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
    ADD_DEFINITIONS(-ansi)
    ADD_DEFINITIONS(-pedantic)
    ADD_DEFINITIONS(-Wall)
    ADD_DEFINITIONS(-Wextra)
    ADD_DEFINITIONS(-Wunused)
#    ADD_DEFINITIONS(-Weffc++)
    ADD_DEFINITIONS(-Wsign-compare)
    #http://gcc.gnu.org/wiki/Visibility
    add_definitions(-fvisibility=hidden)
endif()

add_definitions(-DRADAR_OUTPUT)
add_definitions(-D_FILE_OFFSET_BITS=64)

########################################################################
# Find build dependencies
########################################################################
find_package(PkgConfig)
find_package(LibUSB)
find_package(LibRTLSDR)
set(THREADS_USE_PTHREADS_WIN32 true)
find_package(Threads)

if(NOT LIBUSB_FOUND)
    message(FATAL_ERROR "LibUSB 1.0 required to compile dump1090")
endif()
if(NOT LIBRTLSDR_FOUND)
    message(FATAL_ERROR "Lib RTL_SDR required to compile dump1090")
endif()
if(NOT THREADS_FOUND)
    message(FATAL_ERROR "pthreads(-win32) required to compile dump1090")
endif()
########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${LIBUSB_INCLUDE_DIR}
    ${LIBRTLSDR_INCLUDE_DIR}
    ${THREADS_PTHREADS_INCLUDE_DIR}
)

link_directories(
  ${LIBRTLSDR_PKG_LIBRARY_DIRS}
  ${LIBUSB_PKG_LIBRARY_DIRS}
)

# Set component parameters
set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "" FORCE)

# Set install dir
set (CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../..")

########################################################################
# Specify the target dependencies
########################################################################
add_executable(dump1090
  dump1090.cc
  Client.cc
  anet.cc
  modesDecode.cc
  modesMessage.cc
)

target_link_libraries(dump1090 
  ${LIBRTLSDR_LIBRARIES} 
  ${CMAKE_THREAD_LIBS_INIT} 
)

########################################################################
# Create install target
########################################################################
install (TARGETS dump1090 DESTINATION bin)
install (TARGETS dump1090 DESTINATION dump1090/dump1090)

########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)

########################################################################
# Add subdirectories
########################################################################
#add_subdirectory(...)

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix \${prefix})
set(libdir \${exec_prefix}/lib)
set(includedir \${prefix}/include)

