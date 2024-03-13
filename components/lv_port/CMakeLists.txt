idf_component_register(
    SRC_DIRS
    "." 
    INCLUDE_DIRS
    "include"
    PRIV_REQUIRES
    lcd_bsp
    esp_lcd
    main
)
target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_LIST_DIR}/liblv_port.a")