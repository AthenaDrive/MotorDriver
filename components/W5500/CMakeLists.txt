idf_component_register(
    SRCS "W5500.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_eth espressif__w5500 esp_netif esp_driver_spi
)
