cmake_minimum_required(VERSION 3.0)

set(NRF51_SOFTDEVICE "S130" CACHE STRING "nRF51 SoftDevice version (S110 or S130)")
set(NRF52_SOFTDEVICE "S132" CACHE STRING "nRF52 SoftDevice version (S132)")

set(TARGET_PLATFORM "51" CACHE STRING "Target platform (51, 52 or x86)")
set(DEBUG_LEVEL 3 CACHE STRING "Debug verbosity if built in debug mode (0, 1, 2 or 3)")

#####################
# Toolchain options #
#####################

set(BUILD_TOOLCHAIN "" CACHE STRING "Which toolchain to use (ARM, GCC)")

if (NOT BUILD_HOST)
    if (BUILD_TOOLCHAIN STREQUAL "")
        if (CMAKE_HOST_WIN32)
            set(BUILD_TOOLCHAIN "ARM")
        else ()
            set(BUILD_TOOLCHAIN "GCC")
        endif()
    endif()

    if (TARGET_PLATFORM MATCHES "51")
        list( APPEND HWID "51")
    endif()
    if (TARGET_PLATFORM MATCHES "52")
        list( APPEND  HWID "52")
    endif()
else()
    set(TARGET_PLATFORM "x86")
    list( APPEND  HWID "ut")
endif()

message(STATUS "nRF51 SD = ${NRF51_SOFTDEVICE}")
message(STATUS "nRF52 SD = ${NRF52_SOFTDEVICE}")
message(STATUS "HWID = ${HWID}")
################################
# Global configuration options #
###############################
# option(UECC_ENABLE "Enables compiling the micro-ecc ECDH library" ON)

#########################################
# Setup paths for external dependencies #
#########################################
include(${CMAKE_SOURCE_DIR}/../../mod/cmake/dependency_utils.cmake)

##                   NAME           DESCRIPTION                             PATH                                         FILE_CHECK
# find_dependency(CMOCK_ROOT "Root folder of CMock test library" "${MBTLE_SOURCE_DIR}/../cmock" "src/cmock.c")
# find_dependency(UNITY_ROOT "Root folder of Unity test library" "${CMOCK_ROOT}/vendor/unity" "src/unity.c")
find_dependency(CMAKE_CONFIG "Root folder of CMAKE"     "${CMAKE_SOURCE_DIR}/../../mod/cmake"      "/configure_arm.cmake")
if (TARGET_PLATFORM MATCHES "51")
  find_dependency(SDK_ROOT     "Root folder of SDK"     "${CMAKE_SOURCE_DIR}/../../mod/sdk9"        "/components/toolchain/system_nrf51.c")
elseif (TARGET_PLATFORM MATCHES "52")
  find_dependency(SDK_ROOT     "Root folder of SDK"     "${CMAKE_SOURCE_DIR}/../../mod/sdk12"       "/components/toolchain/system_nrf52.c")
endif()


set (MOPEN_SOURCE_DIR ${CMAKE_SOURCE_DIR})

include(${CMAKE_CONFIG}/configure_arm.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG}")

project(MBTLE_BL C ASM)

if (NOT BUILD_HOST)
    if(BUILD_TOOLCHAIN MATCHES "ARM")
        include(${CMAKE_CONFIG}/Modules/Compiler/ARMCC-C.cmake)
    endif()
    set(CMAKE_EXECUTABLE_SUFFIX ".elf")
endif()

macro(append_suffix out suffix)
    unset(_tmp)
    foreach(arg ${ARGN})
        list( APPEND _tmp ${arg}_${suffix})
    endforeach()
    set(${out} ${_tmp})
endmacro()

macro(get_testname out_name)
  get_filename_component(testname ${CMAKE_CURRENT_LIST_DIR} NAME)
  set(${out_name} ${testname})
endmacro()

set(BL_DEFINITIONS "")
# Fill SRC list variables
# include(${BL_SOURCE_DIR}/mesh/cmake_defines.cmake)
# include(${BL_SOURCE_DIR}/mesh/mbtle_src_definition.cmake)

add_subdirectory("nRF51")

