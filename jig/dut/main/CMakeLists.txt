idf_component_register(SRCS "main.cpp" "tests.cpp")

target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)