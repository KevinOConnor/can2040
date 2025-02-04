# In order to use this library in your own cmake based project, you can use the
# following
# ~~~
# pico_sdk_init()
# include(FetchContent)
#
# FetchContent_Declare(can2040
#   GIT_REPOSITORY https://github.com/KevinOConnor/can2040.git
#   SOURCE_SUBDIR cmake)
# FetchContent_MakeAvailable(can2040)
# ~~~

add_executable(simple simple.c)

target_link_libraries(simple pico_stdlib can2040::can2040)

pico_add_extra_outputs(simple)
