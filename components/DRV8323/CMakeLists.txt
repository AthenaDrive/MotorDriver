idf_component_register(
    SRCS "DRV8323.cpp"
    INCLUDE_DIRS "include"
    REQUIRES SPIBase esp_driver_gpio
)
