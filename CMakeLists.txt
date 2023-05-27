idf_component_register(SRCS "Board.cpp" "3rdparty/arduino-esp32/cores/esp32/esp32-hal-i2c.c"
                            "3rdparty/arduino-esp32/cores/esp32/esp32-hal-gpio.c"
                            "3rdparty/arduino-esp32/libraries/Wire/src/Wire.cpp"
                            "3rdparty/sparkfun_bq27441/src/SparkFunBQ27441.cpp"
                       INCLUDE_DIRS "." "3rdparty/arduino-esp32/cores/esp32"
                            "3rdparty/arduino-esp32/variants/powerfeather_esp32s3"
                            "3rdparty/sparkfun_bq27441/src"
                            "3rdparty/arduino-esp32/libraries/Wire/src"
                       PRIV_REQUIRES spi_flash
                            )
