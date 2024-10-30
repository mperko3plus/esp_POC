set(srcs "esp_ot_br.c"
    "ble.c" "network_utils.c"
    # other source files
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       )