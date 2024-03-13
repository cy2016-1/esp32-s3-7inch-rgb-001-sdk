idf_component_register(
    SRC_DIRS
        "." 
    INCLUDE_DIRS
        "include"
    REQUIRES driver 
    esp_common 
    log
    esp_lcd
    fatfs
    sdmmc)
