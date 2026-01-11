# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Fetch the content using cmake
include(FetchContent)
FetchContent_Declare(
    tdk_lib
    GIT_REPOSITORY https://github.com/tdk-invn-oss/motion.mcu.icm42670p.driver.git
    GIT_TAG        5e9604112e3804d4a11acb0b8d93e23a80532e9c
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(tdk_lib)
if(NOT tdk_lib_POPULATED)
    message(STATUS "Downloading TDK ICM-42670-P Driver...")
    FetchContent_Populate(tdk_lib)
endif()
set(TDK_ROOT "${tdk_lib_SOURCE_DIR}" CACHE INTERNAL "Path to TDK Driver sources")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware)
