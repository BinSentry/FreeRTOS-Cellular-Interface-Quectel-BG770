afr_module(NAME FreeRTOS-Cellular-Interface-Module INTERNAL)

set(src_dir "${CMAKE_CURRENT_LIST_DIR}/source")

afr_module_sources(
    ${AFR_CURRENT_MODULE}
    PUBLIC
        "${src_dir}/cellular_bg96.c"
        "${src_dir}/cellular_bg96.h"
        "${src_dir}/cellular_bg96_api.c"
        "${src_dir}/cellular_bg96_urc_handler.c"
        "${src_dir}/cellular_bg96_wrapper.c"
)

afr_module_include_dirs(
    ${AFR_CURRENT_MODULE}
    PUBLIC "${src_dir}"
)

afr_module_dependencies(
    ${AFR_CURRENT_MODULE}
    PUBLIC AFR::FreeRTOS-Cellular-Interface AFR::FreeRTOS-Cellular-Interface::mcu_port
)
