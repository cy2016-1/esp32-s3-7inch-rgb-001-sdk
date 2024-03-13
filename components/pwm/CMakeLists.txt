idf_component_register(
    SRC_DIRS
        "." 
    INCLUDE_DIRS
        "include"

    REQUIRES
        lcd_bsp
        log
        driver
        )
target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_LIST_DIR}/libpwm.a")