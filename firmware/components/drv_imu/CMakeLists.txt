if(NOT EXISTS "${TDK_ROOT}")
    message(WARNING "TDK Driver not found at ${TDK_ROOT}. It will be downloaded during build.")
endif()

file(GLOB_RECURSE COMPONENT_SRC *.c)

idf_component_register(
    SRCS
        ${COMPONENT_SRC}
        "${TDK_ROOT}/icm42670p/imu/inv_imu_driver.c"
        "${TDK_ROOT}/icm42670p/imu/inv_imu_transport.c"
        "${TDK_ROOT}/icm42670p/imu/inv_imu_selftest.c"

    INCLUDE_DIRS
        "."
        "${TDK_ROOT}/icm42670p"

    REQUIRES
        driver esp_timer
)