idf_component_register(
    SRCS "DRV8320S.cpp"
    INCLUDE_DIRS "include"
    REQUIRES SPIBase esp_driver_gpio
)
