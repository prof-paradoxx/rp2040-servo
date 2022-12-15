add_executable(sweep servo_sweep.c)

target_link_libraries(sweep PRIVATE pico_stdlib hardware_pwm rp2040_servo)

pico_add_extra_outputs(sweep)


#add_executable(angle_user_input angle_user_input.c)

#target_link_libraries(angle_user_input PRIVATE pico_stdlib hardware_pwm rp2040_servo)

#pico_add_extra_outputs(angle_user_input)

#pico_enable_stdio_usb(angle_user_input 1)
#pico_enable_stdio_uart(angle_user_input 0)