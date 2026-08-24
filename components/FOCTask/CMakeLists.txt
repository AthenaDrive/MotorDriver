idf_component_register(
    SRCS "FOCTask.cpp"
    INCLUDE_DIRS "include"
    REQUIRES GlobalVariableManager esp_driver_gpio esp_event esp_timer MCPWMDriver ADCOneshot AS5047P DRV8323 Controller StateEstimation CascadingPID)
