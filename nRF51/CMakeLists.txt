# set(TCA_SOURCE_FILES
#     "${CMAKE_SOURCE_DIR}/mod/tca/target_libs/app_fifo.c"
#     "${CMAKE_SOURCE_DIR}/mod/tca/target_libs/dlestxetx.c"
#     "${CMAKE_SOURCE_DIR}/mod/tca/target_libs/mem_manager.c"
#     "${CMAKE_SOURCE_DIR}/mod/tca/target_libs/simple_adapter_protocol.c"
# )


set(MOPEN_S130_HEADERS
    "${SDK_ROOT}/components/softdevice/s130/headers"
    "${SDK_ROOT}/components/softdevice/s130/headers/nrf51"
    )

set(MOPEN_S132_HEADERS
    "${SDK_ROOT}/components/softdevice/s132/headers"
    "${SDK_ROOT}/components/softdevice/s132/headers/nrf52"
    )

set(CMAKE_TOOLCHAIN_HEADERS
    "${CMAKE_CONFIG}/toolchain"
    "${CMAKE_CONFIG}/toolchain/device"
    "${CMAKE_CONFIG}/toolchain/gcc"
    "${CMAKE_CONFIG}/toolchain/cmsis/include"
    )

if (NRF51_SOFTDEVICE STREQUAL "S130")
    set(NRF51_SD_HEADERS   "${MOPEN_S130_HEADERS}")
    set(NRF51_LINKER_FILE  "${MOPEN_SOURCE_DIR}/nRF51/bootloader/gcc/gcc_bl_nrf51_xxAC.ld")
    set(NRF51_SCATTER_FILE "${MOPEN_SOURCE_DIR}/nRF51/bootloader/arm/bootloader_serial_xxAC.sct")
    set(SDK_SOURCE_FILES
        "${SDK_ROOT}/components/libraries/sha256/sha256.c"
    )
else ()
    message(FATAL_ERROR "Unsupported SoftDevice version for nRF51: ${NRF51_SOFTDEVICE}")
endif()

if (NRF52_SOFTDEVICE STREQUAL "S132")
    set(NRF52_SD_HEADERS   "${MOPEN_S132_HEADERS}")
    set(NRF52_LINKER_FILE  "${MOPEN_SOURCE_DIR}/nRF51/bootloader/gcc/gcc_bl_nrf52_xxAC.ld")
    set(NRF52_SCATTER_FILE "${MOPEN_SOURCE_DIR}/nRF51/bootloader/arm/bootloader_serial_nRF52_xxAA.sct")
    set(SDK_SOURCE_FILES
        "${SDK_ROOT}/components/libraries/sha256/sha256.c"
    )
else()
    message(FATAL_ERROR "Unsupported SoftDevice version for nRF52: ${NRF52_SOFTDEVICE}")
endif()

set(MOPEN_SDK_HEADERS
    "${SDK_ROOT}/components/libraries/sha256"
    "${SDK_ROOT}/components/device"
    "${SDK_ROOT}/examples/bsp"
    "${SDK_ROOT}/components/libraries/util"
    "${SDK_ROOT}/components/drivers_nrf/hal"
    # "${MOPEN_SOURCE_DIR}/examples"
    )

set(MOPEN_LIB_HEADERS
    "${MOPEN_SOURCE_DIR}/lib/micro-ecc"
    "${MOPEN_SOURCE_DIR}/lib/rtt/include"
    )

set (STD_TEST_FLAGS
    "-D__HEAP_SIZE=1500"
    "-DSOFTDEVICE_PRESENT"
    "-DBSP_DEFINES_ONLY"
    "-DBLE_STACK_SUPPORT_REQD"
    "-DDEBUG=3"
    "-DSWI_DISABLE0"
    "-DLOG_CALLBACK_DEFAULT=log_callback_rtt"
    ${CMAKE_DEFINITIONS}
    )

set (STD_MESH_FLAGS
    "-DMEM_MANAGER_DISABLE_LOGS"
    "-DINTERNAL_EVT_ENABLE"
    )



# add_subdirectory("rbc_mesh")
# add_subdirectory("examples")
add_subdirectory("bootloader")

