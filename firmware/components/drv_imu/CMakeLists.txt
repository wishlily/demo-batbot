if(NOT EXISTS "${TDK_ROOT}")
    message(WARNING "TDK Driver not found at ${TDK_ROOT}. It will be downloaded during build.")
endif()

if(NOT EXISTS "${FUSION_PATH}")
    message(WARNING "Fusion Lib not found at ${FUSION_PATH}. It will be downloaded during build.")
endif()

file(GLOB FUSION_SRCS "${FUSION_PATH}/*.c")
file(GLOB_RECURSE COMPONENT_SRC *.c)

idf_component_register(
    SRCS
        ${COMPONENT_SRC}
        "${TDK_ROOT}/icm42670p/imu/inv_imu_driver.c"
        "${TDK_ROOT}/icm42670p/imu/inv_imu_transport.c"
        "${TDK_ROOT}/icm42670p/imu/inv_imu_selftest.c"
        ${FUSION_SRCS}

    INCLUDE_DIRS
        "."
        "${TDK_ROOT}/icm42670p"
        "${FUSION_PATH}"

    REQUIRES
        driver esp_timer
)