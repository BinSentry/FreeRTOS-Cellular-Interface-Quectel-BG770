afr_module(NAME freertos_cellular_interface::module INTERNAL)

set(src_dir "${CMAKE_CURRENT_LIST_DIR}/source")
set(inc_dir "${CMAKE_CURRENT_LIST_DIR}/include")

afr_module_sources(
    freertos_cellular_interface::module #${AFR_CURRENT_MODULE}
    PRIVATE
        "${src_dir}/cellular_bg96.c"
        "${src_dir}/cellular_bg96.h"
        "${src_dir}/cellular_bg96_api.c"
        "${src_dir}/cellular_bg96_urc_handler.c"
        "${src_dir}/cellular_bg96_wrapper.c"
)

afr_module_include_dirs(
    freertos_cellular_interface::module #${AFR_CURRENT_MODULE}
    PUBLIC "${inc_dir}"
)

#afr_module_dependencies(
#    ${AFR_CURRENT_MODULE}
#    PUBLIC AFR::freertos_cellular_interface
#)

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
#    INTERFACE AFR::freertos_cellular_interface
#)
