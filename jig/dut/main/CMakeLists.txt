idf_component_register(SRCS "main.cpp" "tests.cpp")

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)