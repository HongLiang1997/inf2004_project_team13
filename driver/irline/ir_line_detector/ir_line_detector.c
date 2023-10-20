#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

int main(void)
{
    stdio_init_all();

    // Configure the GPIO pins connected to the left and right IR sensors as inputs
    gpio_init(26); // Left sensor connected to GPIO 26
    gpio_init(27); // Right sensor connected to GPIO 27
    gpio_set_dir(26, GPIO_IN);
    gpio_set_dir(27, GPIO_IN);

    uint gpio_left = 26;
    uint gpio_right = 27;

    while (1)
    {

        bool left_result = gpio_get(gpio_left);   // Read from the left sensor (GPIO 26)
        bool right_result = gpio_get(gpio_right); // Read from the right sensor (GPIO 27)

        // Default State IR Sensor will return 1
        printf("Left Sensor Reading: %d\n", left_result);
        printf("Right Sensor Reading: %d\n", right_result);

        if (left_result == 0 && right_result == 0)
        {
            // Both sensors detect a black line.
            // This indicates that the car is directly on top of both lines.
            // You can perform a specific action like stopping or making corrections.
        }
        else if (left_result == 0 && right_result == 1)
        {
            // Left sensor detects a black line, but right sensor doesn't.
            // This means the car is deviating towards the left line.
            // You can steer the car to the right to get back on track. By making Left Wheel rotation higher
        }
        else if (left_result == 1 && right_result == 0)
        {
            // Right sensor detects a black line, but left sensor doesn't.
            // This means the car is deviating towards the right line.
            // You can steer the car to the left to get back on track. By making Right Wheel rotation higher
        }
        else if (left_result == 1 && right_result == 1)
        {
            // Both sensors do not detect a black line.
            // The car is off the lines.
            // You can perform a specific action, such as searching for the lines.
        }

        sleep_ms(1000); // Wait for a moment before taking the next reading
    }
}
