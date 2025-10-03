idf_component_register(
    SRCS "lora_driver.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver board_heltec_v3
)