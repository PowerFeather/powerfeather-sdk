idf_component_register(SRCS "${CMAKE_CURRENT_LIST_DIR}/src/MainBoard.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/MasterI2C.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/ArduinoMasterI2C.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/BQ2562x.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/LC709204F.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Mutex.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32/esp32-hal-i2c.c"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32/esp32-hal-gpio.c"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32/esp32-hal-adc.c"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32/esp32-hal-i2c-slave.c"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32/esp32-hal-misc.c"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32/Stream.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32/WString.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/libraries/Wire/src/Wire.cpp"
                       INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/src" 
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/cores/esp32"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/variants/esp32s3_powerfeather"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/libraries/Wire/src"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/libraries/Wire/src"
                            "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty/arduino-esp32/tools/sdk/esp32s3/include/esp_adc_cal/include"
                       PRIV_REQUIRES spi_flash nvs_flash
                            )

target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)