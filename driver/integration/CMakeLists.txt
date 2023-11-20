add_executable(main
        main.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(main pico_stdlib hardware_pwm hardware_uart pico_ultrasonic FreeRTOS-Kernel-Heap4)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)
    
# create map/bin/hex file etc.
pico_add_extra_outputs(main)

# add url via pico_set_program_url
example_auto_set_url(main)

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        # Add any other directories as needed
        )
