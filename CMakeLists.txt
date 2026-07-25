set(powerfeather_sdk_requires driver esp_timer)

# ESPHome builds Arduino projects with ESP-IDF's component manager.  The SDK's
# public Mainboard header has Arduino-dependent member layouts, so the SDK
# component itself must be compiled with the same definition as its consumer.
# ESPHome enables the Arduino component as a dependency, without enabling its
# Arduino sketch autostart option.
if(CONFIG_ENABLE_ARDUINO_DEPENDS)
    list(APPEND powerfeather_sdk_requires espressif__arduino-esp32)
endif()

idf_component_register(SRCS "${CMAKE_CURRENT_LIST_DIR}/src/Mainboard/Mainboard.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Mainboard/BQ2562x.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Mainboard/LC709204F.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Mainboard/MAX17260.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Utils/MasterI2C.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Utils/ArduinoMasterI2C.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Utils/Mutex.cpp"
                       INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/src" 
                       PRIV_REQUIRES spi_flash nvs_flash
                       REQUIRES ${powerfeather_sdk_requires})

if(CONFIG_ENABLE_ARDUINO_DEPENDS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ARDUINO)
endif()
