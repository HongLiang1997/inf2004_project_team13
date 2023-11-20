add_executable(adc_console
        adc_console.c
        )

target_link_libraries(adc_console pico_stdlib hardware_adc hardware_i2c hardware_timer)

# create map/bin/hex file etc.
pico_add_extra_outputs(adc_console)

# add url via pico_set_program_url
example_auto_set_url(adc_console)

# enable usb output, disable uart output
pico_enable_stdio_usb(adc_console 1)
pico_enable_stdio_uart(adc_console 0)

