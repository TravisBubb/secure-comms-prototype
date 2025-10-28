idf_component_register(
    SRCS "src/lora_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio
)