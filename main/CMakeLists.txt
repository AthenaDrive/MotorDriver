idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES I2CBase SPIBase LM75A INA238 LSM6DSO AS5047P EthernetTask FOCTask DRV8320S esp_timer
)
