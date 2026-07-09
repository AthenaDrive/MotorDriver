idf_component_register(
    SRCS "EthernetTask.cpp" "Discovery.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_event esp_timer W5500 GlobalVariableManager
)
