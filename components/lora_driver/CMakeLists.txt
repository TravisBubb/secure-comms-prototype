idf_component_register(
    SRCS "lora_spi.c" "lora_cmd.c" "lora_hw.c" "lora_driver.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver board_heltec_v3
)