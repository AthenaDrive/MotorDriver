idf_component_register(
    SRCS "BrakeChopperPWM.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_mcpwm esp_driver_gpio)
