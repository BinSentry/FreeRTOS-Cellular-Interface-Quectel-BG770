# ====================  Define your project name (edit) ========================
set(project_name "cellular_bg770")

# =====================  Create your mock here  (edit)  ========================

# list the files to mock here
list(APPEND mock_list
            "${CELLULAR_COMMON_INCLUDE_DIRS}/cellular_common.h"
            "${CELLULAR_COMMON_INCLUDE_DIRS}/cellular_common_api.h"
            "${CELLULAR_COMMON_INCLUDE_DIRS}/cellular_at_core.h"
        )

# list the directories your mocks need
list(APPEND mock_include_list
            ${CELLULAR_TEST_DIRS}
            ${CELLULAR_TEST_DIRS}/logging
            ${CELLULAR_COMMON_INCLUDE_DIRS}
            ${CELLULAR_COMMON_INCLUDE_PRIVATE_DIRS}
            ${CELLULAR_INTERFACE_INCLUDE_DIRS}
            ${CELLULAR_INCLUDE_DIRS}
        )
#list the definitions of your mocks to control what to be included
list(APPEND mock_define_list
            "MOCK_LIB_TEST=1"
        )

# ================= Create the library under test here (edit) ==================

# list the files you would like to test here
list(APPEND real_source_files
            ${BG770_SOURCE}/cellular_bg770.c
            ${BG770_SOURCE}/cellular_bg770_api.c
            ${BG770_SOURCE}/cellular_bg770_urc_handler.c
        )
# list the directories the module under test includes
list(APPEND real_include_directories
            ${CELLULAR_TEST_DIRS}
            ${CELLULAR_TEST_DIRS}/logging
            ${BG770_SOURCE}
            ${CELLULAR_INCLUDE_DIRS}
            ${CELLULAR_INTERFACE_INCLUDE_DIRS}
            ${CELLULAR_COMMON_INCLUDE_DIRS}
            ${CELLULAR_COMMON_INCLUDE_PRIVATE_DIRS}
        )

# =====================  Create UnitTest Code here (edit)  =====================

# list the directories your test needs to include
list(APPEND test_include_directories
            ${CMAKE_CURRENT_LIST_DIR}           # the generated test runner copies the test's includes
            ${CELLULAR_TEST_DIRS}
            ${CELLULAR_TEST_DIRS}/logging
            ${BG770_SOURCE}
            ${CELLULAR_INCLUDE_DIRS}
            ${CELLULAR_COMMON_INCLUDE_DIRS}
            ${CELLULAR_COMMON_INCLUDE_PRIVATE_DIRS}
            ${CELLULAR_INTERFACE_INCLUDE_DIRS}
        )

# =============================  (end edit)  ===================================

set(mock_name "${project_name}_mock")
set(real_name "${project_name}_real")

create_mock_list(${mock_name}
                "${mock_list}"
                "${MODULE_ROOT_DIR}/tools/cmock/project.yml"
                "${mock_include_list}"
                "${mock_define_list}"
        )

# cellular_common_api.h includes cellular_types.h before the cellular config, so the config is included first
# for the generated mocks.
target_compile_options(${mock_name} PRIVATE
            "SHELL:-include cellular_config.h"
            "SHELL:-include cellular_config_defaults.h"
        )

create_real_library(${real_name}
                    "${real_source_files}"
                    "${real_include_directories}"
                    "${mock_name}"
        )

# The module calls FreeRTOS and BinSentry functions the FreeRTOS-Cellular-Interface test platform header doesn't
# declare, force-include their prototypes so the module compiles without implicit declarations.
target_compile_options(${real_name} PRIVATE
            "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/cellular_bg770_test_platform.h"
        )

list(APPEND utest_link_list
            -l${mock_name}
            lib${real_name}.a
        )

list(APPEND utest_dep_list
            ${real_name}
        )

set(utest_name "${project_name}_api_utest")
set(utest_source "${project_name}_api_utest.c")
create_test(${utest_name}
            ${utest_source}
            "${utest_link_list}"
            "${utest_dep_list}"
            "${test_include_directories}"
        )
