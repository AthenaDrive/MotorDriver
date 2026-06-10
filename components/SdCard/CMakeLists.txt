idf_component_register(
    SRCS "SdCard.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sdmmc fatfs esp_driver_sdspi esp_driver_spi
)
