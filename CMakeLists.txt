idf_component_register(SRCS "${CMAKE_CURRENT_LIST_DIR}/src/MainBoard.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/MasterI2C.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/ArduinoMasterI2C.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/BQ2562x.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/LC709204F.cpp"
                            "${CMAKE_CURRENT_LIST_DIR}/src/Mutex.cpp"
                       INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/src" 
                       PRIV_REQUIRES spi_flash nvs_flash)

target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)