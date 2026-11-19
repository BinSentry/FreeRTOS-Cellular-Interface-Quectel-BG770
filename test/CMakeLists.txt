cmake_minimum_required ( VERSION 3.13.0 )
project ( "cellular_bg770 unit test"
          VERSION 1.0.0
          LANGUAGES C )

# Allow the project to be organized into folders.
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

# The BG770 module sources use C11 features (static_assert).
set( CMAKE_C_STANDARD 11 )
set( CMAKE_C_STANDARD_REQUIRED ON )

# Do not allow in-source build.
if( ${PROJECT_SOURCE_DIR} STREQUAL ${PROJECT_BINARY_DIR} )
    message( FATAL_ERROR "In-source build is not allowed. Please build in a separate directory, such as ${PROJECT_SOURCE_DIR}/build." )
endif()

# Set global path variables.
get_filename_component( __BG770_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE )
set( BG770_ROOT_DIR ${__BG770_ROOT_DIR} CACHE INTERNAL "cellular_bg770 repository root." )

# The BG770 module is built against the FreeRTOS-Cellular-Interface common library, and these tests reuse its
# unit test infrastructure (CMock, Unity, test config and platform headers). In amazon-freertos the two repositories
# are siblings under libraries/freertos_plus/standard.
get_filename_component( __CELLULAR_INTERFACE_DIR "${BG770_ROOT_DIR}/../../FreeRTOS-Cellular-Interface" ABSOLUTE )
set( CELLULAR_INTERFACE_DIR ${__CELLULAR_INTERFACE_DIR} CACHE PATH "FreeRTOS-Cellular-Interface repository root." )

if( NOT EXISTS ${CELLULAR_INTERFACE_DIR}/source/include/cellular_api.h )
    message( FATAL_ERROR "FreeRTOS-Cellular-Interface not found at ${CELLULAR_INTERFACE_DIR}, set CELLULAR_INTERFACE_DIR." )
endif()

# The FreeRTOS-Cellular-Interface CMock helpers locate project.yml through MODULE_ROOT_DIR.
set( MODULE_ROOT_DIR ${CELLULAR_INTERFACE_DIR} CACHE INTERNAL "cellular_library repository root." )

# Set output directories.
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

# Set the source path and include path
set( BG770_SOURCE ${BG770_ROOT_DIR}/source )
set( CELLULAR_COMMON_SOURCE ${CELLULAR_INTERFACE_DIR}/source )
set( CELLULAR_INCLUDE_DIRS ${CELLULAR_COMMON_SOURCE}/include )
set( CELLULAR_COMMON_INCLUDE_DIRS ${CELLULAR_INCLUDE_DIRS}/common )
set( CELLULAR_COMMON_INCLUDE_PRIVATE_DIRS ${CELLULAR_INCLUDE_DIRS}/private )
set( CELLULAR_INTERFACE_INCLUDE_DIRS ${CELLULAR_COMMON_SOURCE}/interface )
set( CELLULAR_TEST_DIRS ${CELLULAR_INTERFACE_DIR}/test/unit-test )

#  ====================================  Test Configuration ========================================

# Define a CMock resource path.
set( CMOCK_DIR ${CELLULAR_TEST_DIRS}/CMock CACHE INTERNAL "CMock library source directory." )

# Include CMock build configuration.
include( ${CELLULAR_TEST_DIRS}/cmock_build.cmake )

if( NOT EXISTS ${CMOCK_DIR}/src )
    message( FATAL_ERROR "The required submodule CMock does not exist in ${CMOCK_DIR}, initialize the FreeRTOS-Cellular-Interface submodules." )
endif()

# Use CTest utility for managing test runs. This has to be added BEFORE
# defining test targets with add_test()
enable_testing()

# Add build targets for CMock and Unit, required for unit testing.
add_cmock_targets()

# Add function to enable CMock based tests and coverage.
include( ${CELLULAR_INTERFACE_DIR}/tools/cmock/create_test.cmake )

# Include build configuration for unit tests.
add_subdirectory( unit-test )
