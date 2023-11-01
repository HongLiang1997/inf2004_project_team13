add_executable(motor
        motor.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(motor pico_stdlib hardware_pwm hardware_gpio)

# enable usb output, enable uart output
pico_enable_stdio_usb(ir_line_detector 1)
pico_enable_stdio_uart(ir_line_detector 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(motor)

# add url via pico_set_program_url
example_auto_set_url(motor)