add_executable(ir_line_detector
ir_line_detector.c
        )

target_link_libraries(ir_line_detector pico_stdlib hardware_adc)

# enable usb output, enable uart output
pico_enable_stdio_usb(ir_line_detector 1)
pico_enable_stdio_uart(ir_line_detector 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(ir_line_detector)

# add url via pico_set_program_url
example_auto_set_url(ir_line_detector)

