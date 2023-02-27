afr_module(NAME FreeRTOS-Cellular-Interface-Module INTERNAL)

set(src_dir "${CMAKE_CURRENT_LIST_DIR}/source")
set(inc_dir "${CMAKE_CURRENT_LIST_DIR}/include")

afr_module_sources(
    FreeRTOS-Cellular-Interface-Module #${AFR_CURRENT_MODULE}
    PUBLIC
        "${src_dir}/cellular_bg96.c"
        "${src_dir}/cellular_bg96.h"
        "${src_dir}/cellular_bg96_api.c"
        "${src_dir}/cellular_bg96_urc_handler.c"
        "${src_dir}/cellular_bg96_wrapper.c"
        "${inc_dir}/cellular_config.h"
)

afr_module_include_dirs(
    FreeRTOS-Cellular-Interface-Module #${AFR_CURRENT_MODULE}
    PUBLIC "${inc_dir}"
)

afr_module_dependencies(
    ${AFR_CURRENT_MODULE}
    PUBLIC AFR::FreeRTOS-Cellular-Interface
)

#afr_test_module()
#
#afr_module_sources(
#    ${AFR_CURRENT_MODULE}
#    INTERFACE
#        "${test_dir}/iot_test_freertos_tcp.c"
#)
#
#afr_module_dependencies(
#    ${AFR_CURRENT_MODULE}
#    INTERFACE AFR::FreeRTOS-Cellular-Interface
#)
