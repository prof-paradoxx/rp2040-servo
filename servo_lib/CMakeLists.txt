add_library(rp2040_servo servo.h servo.c)

target_link_libraries(rp2040_servo pico_stdlib hardware_pwm hardware_clocks)

target_include_directories(rp2040_servo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")