cmake_minimum_required(VERSION 3.16)

project(powerfeather-sdk VERSION 1.0.7 LANGUAGES CXX)

include_directories(${CMAKE_CURRENT_LIST_DIR})
add_compile_options("-Os")
add_compile_options("-m32")
add_library(powerfeather-sdk STATIC
            Mainboard/BQ2562x.cpp
            Mainboard/LC709204F.cpp
            Mainboard/Mainboard.cpp
            Utils/SoC.cpp
            Utils/Mutex.cpp
            Utils/MasterI2C.cpp
            Utils/Logging.cpp)