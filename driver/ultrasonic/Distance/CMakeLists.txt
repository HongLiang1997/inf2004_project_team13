cmake_minimum_required(VERSION 3.13)

# Add the ultrasonic sensor source file and other necessary source files
add_executable(centimeter
        ultrasonic.c
        )

# Link necessary libraries
target_link_libraries(centimeter
        pico_stdlib
        hardware_uart
        pico_ultrasonic
        FreeRTOS-Kernel-Heap4
        )

# Enable STDIO
pico_enable_stdio_usb(centimeter 1)
pico_enable_stdio_uart(centimeter 0)

# Add extra outputs if needed
pico_add_extra_outputs(centimeter)

# Assuming you have WiFi-related definitions in your original code,
# you may want to add those here, if applicable.
# target_compile_definitions(centimeter PRIVATE
#        WIFI_SSID=\"${WIFI_SSID}\"
#        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
#        )

# Include necessary directories
target_include_directories(centimeter PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        # Add any other directories as needed
        )

# If you have additional dependencies or configuration specific to
# this project, you can add them here.
