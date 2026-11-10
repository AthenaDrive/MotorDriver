idf_component_register(
    SRCS "FOCTask.cpp"
    INCLUDE_DIRS "include"
    REQUIRES GlobalVariableManager esp_driver_gpio esp_event esp_timer MCPWMDriver ADCContinuous AS5047P DRV8320S Controller StateEstimation CascadingPID)
