set(powerfeather_sdk_requires driver esp_timer)
set(powerfeather_sdk_private_requires spi_flash nvs_flash)

# ESPHome builds Arduino projects with ESP-IDF's component manager.  The SDK's
# public Mainboard header has Arduino-dependent member layouts, so the SDK
# component itself must be compiled with the same definition as its consumer.
# The SDK's CMakeLists is evaluated during ESP-IDF's early dependency pass,
# before BUILD_COMPONENTS is resolved. ESPHome publishes its managed components
# through this property for that pass, including Arduino when it is in use.
idf_build_get_property(powerfeather_sdk_managed_components ESPHOME_PROJECT_MANAGED_COMPONENTS)
if(espressif__arduino-esp32 IN_LIST powerfeather_sdk_managed_components)
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
                       PRIV_REQUIRES ${powerfeather_sdk_private_requires}
                       REQUIRES ${powerfeather_sdk_requires})
