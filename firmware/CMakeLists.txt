# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Fetch ICM-42670-P code
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

# Fetch Fusion
FetchContent_Declare(
    fusion_lib
    GIT_REPOSITORY https://github.com/xioTechnologies/Fusion.git
    GIT_TAG        v1.2.11
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(fusion_lib)
if(NOT fusion_lib_POPULATED)
    message(STATUS "Downloading Fusion Library v1.2.11...")
    FetchContent_Populate(fusion_lib)
endif()
set(FUSION_PATH "${fusion_lib_SOURCE_DIR}/Fusion" CACHE INTERNAL "Path to Fusion source")

# micro_ros_espidf_component add extra_packages, clean: idf.py clean-microros
get_filename_component(ROS_WS_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../ws/src" ABSOLUTE)
set(COMPONENT_EXTRA_PKG_DIR "${CMAKE_CURRENT_LIST_DIR}/components/micro_ros_espidf_component/extra_packages")
message(STATUS "--------------------------------------------------")
message(STATUS "injecting custom packages from: ${ROS_WS_SRC_DIR}")
message(STATUS "                         to: ${COMPONENT_EXTRA_PKG_DIR}")
file(MAKE_DIRECTORY "${COMPONENT_EXTRA_PKG_DIR}")
file(REMOVE_RECURSE "${COMPONENT_EXTRA_PKG_DIR}/batbot_msgs")
if(EXISTS "${ROS_WS_SRC_DIR}/batbot_msgs")
    file(COPY "${ROS_WS_SRC_DIR}/batbot_msgs" DESTINATION "${COMPONENT_EXTRA_PKG_DIR}")
    message(STATUS "Status: batbot_msgs injected successfully.")
else()
    message(FATAL_ERROR "Error: Source package not found at ${ROS_WS_SRC_DIR}/batbot_msgs")
endif()
message(STATUS "--------------------------------------------------")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware)
