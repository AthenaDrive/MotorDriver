idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES I2CBase SPIBase MCP23017 LM75A INA238 LSM6DSO AS5047P SdCard W5500 DRV8323 MCPWMDriver ADCOneshot esp_timer
)
