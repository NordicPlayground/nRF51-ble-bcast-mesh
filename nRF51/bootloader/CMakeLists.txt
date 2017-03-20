get_testname(targetname)

set(SRCS
    main.c
    bootloader_rtc.c
    bootloader.c
    transport.c
    core/bootloader_app_bridge.c
    core/bootloader_info.c
    core/dfu_bank.c
    core/dfu_mesh.c
    core/dfu_transfer_mesh.c
    core/uECC.c
    "${SDK_ROOT}/components/libraries/sha256/sha256.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/radio_control.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/fifo.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/mesh_packet.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/mesh_aci.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/bootloader_util.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/rand.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/nrf_flash.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/serial_handler_uart.c"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/src/dfu_util.c"
)

include_directories(
    include
    core/include
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh"
    "${CMAKE_SOURCE_DIR}/nRF51/RTT"
    "${CMAKE_SOURCE_DIR}/nRF51/rbc_mesh/include"
    "${MOPEN_SDK_HEADERS}"
    )


if(HOST)

    # Remove warning about attribute 'naked' caused by softdevice headers:
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-attributes")

    #make SVC-calls run inline:
    add_definitions(-DSVCALL_AS_NORMAL_FUNCTION)

    if(CMAKE_HOST_UNIX)
        # add_definitions(-DNETWORK_BEARER=BEARER_SOCKET)
    endif()
    #add_subdirectory("test")
else()

    add_definitions(
        "-DDEBUG=3"
        "-DDEBUG_LEDS"
        "-DRBC_MESH_SERIAL"
        "-DuECC_CURVE=uECC_secp256r1"
        "-DRBC_MESH_PACKET_POOL_SIZE=32"
        "-DBOOTLOADER"
        # "-DNO_TIMEOUTS"
    )

    set(exec_name "${targetname}")
    add_executable_platforms("${exec_name}" ${SRCS})

    list(APPEND CMAKE_C_FLAGS "--feedback unused.txt")
    string(REGEX REPLACE ";" " " CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

    # list(APPEND CMAKE_C_LINK_FLAGS "--library_type=microlib")
    # list(APPEND CMAKE_C_LINK_FLAGS "--feedback unused.txt")
    # string(REGEX REPLACE ";" " " CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS}")

    # add_target_property_platforms(           ${exec_name} COMPILE_FLAGS "--feedback_type=iw")
    add_target_property_platforms(           ${exec_name} LINK_FLAGS    "--feedback unused.txt")

    create_hex_platform(${exec_name})
    add_flash_target(${exec_name})

endif()

