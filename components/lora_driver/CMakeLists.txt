idf_component_register(
    SRCS "lora_cmd_sx1262.c" "lora_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)