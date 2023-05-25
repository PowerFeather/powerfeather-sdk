idf_component_register(SRCS "Board.cpp" "3rdparty/arduino-esp32/cores/esp32/esp32-hal-i2c.c"
                            "3rdparty/arduino-esp32/cores/esp32/esp32-hal-gpio.c"
                       INCLUDE_DIRS "." "3rdparty/arduino-esp32/cores/esp32"
                            "3rdparty/arduino-esp32/variants/powerfeather_esp32s3")
