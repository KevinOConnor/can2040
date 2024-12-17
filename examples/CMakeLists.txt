add_executable(simple simple.c)

target_link_libraries(simple pico_stdlib can2040::can2040)

pico_add_extra_outputs(simple)
