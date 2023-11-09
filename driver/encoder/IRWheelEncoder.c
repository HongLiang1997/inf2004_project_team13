// 1. distance if two spining at diff speed means pivoting, don't increase overall distance
// 2. freertos message buffer

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Constants for distance and speed calculation
const float wheel_circumference = 21; // Circumference of the wheel in centimeters
const float pulses_per_revolution = 20.0; // Number of slots on the encoder disc

// Variables to keep track of state for the first sensor (gp2)
volatile uint32_t pulse_count_1 = 0;
uint32_t pulse_start_time_1 = 0; // Change pulse_start_time_1 to uint32_t
bool pulse_started_1 = false;
uint32_t last_edge_fall_time_1 = 0; // Track the time of the last falling edge
// Variables to track the time since the last pulse and printing flag
uint32_t last_pulse_time_1 = 0;
bool print_flag_1 = false;

// Variables to keep track of state for the second sensor (gp3)
volatile uint32_t pulse_count_2 = 0;
uint32_t pulse_start_time_2 = 0; // Change pulse_start_time_1 to uint32_t
bool pulse_started_2 = false;
uint32_t last_edge_fall_time_2 = 0; // Track the time of the last falling edge
// Variables to track the time since the last pulse and printing flag
uint32_t last_pulse_time_2 = 0;
bool print_flag_2 = false;

// Fixed interval for pulse count 1 and 2 (both IR sensors)
const uint32_t pulse_print_interval = 2000000; // 2 seconds

// GPIO pin configuration for the first sensor
const uint gpio_pin_1 = 2;
uint gpio_encoder_left = 2; // for detection of left interrupt

// GPIO pin configuration for the second sensor
const uint gpio_pin_2 = 3;
uint gpio_encoder_right = 3; // for detection of right interrupt

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

// Function to handle pulse detection and printing for first sensor
void handle_pulse_detection_1(const char *event_str) {
    uint32_t pulse_end_time = (uint32_t)time_us_32(); // Convert to uint32_t
    uint32_t pulse_duration_us = pulse_end_time - pulse_start_time_1;
    float speed_mps = ((wheel_circumference / pulses_per_revolution) / 100) / (pulse_duration_us / 1e6); // Convert to meters per second, distance over time
    pulse_count_1++;
    last_pulse_time_1 = (uint32_t)time_us_32(); // Update the last pulse time
    print_flag_1 = true; // Set the flag to indicate that a print is needed
    printf("Sensor 1 - Pulse detected - Distance: %.2f m, Speed: %.2f m/s, Events: %s\n", (pulse_count_1 * (wheel_circumference / pulses_per_revolution)) / 100, speed_mps, event_str);
}

// Function to handle pulse detection and printing for second sensor
void handle_pulse_detection_2(const char *event_str) {
    uint32_t pulse_end_time = (uint32_t)time_us_32(); // Convert to uint32_t
    uint32_t pulse_duration_us = pulse_end_time - pulse_start_time_2;
    float speed_mps = ((wheel_circumference / pulses_per_revolution) / 100) / (pulse_duration_us / 1e6); // Convert to meters per second, distance over time
    pulse_count_2++;
    last_pulse_time_2 = (uint32_t)time_us_32(); // Update the last pulse time
    print_flag_2 = true; // Set the flag to indicate that a print is needed
    printf("Sensor 2 - Pulse detected - Distance: %.2f m, Speed: %.2f m/s, Events: %s\n", (pulse_count_2 * (wheel_circumference / pulses_per_revolution)) / 100, speed_mps, event_str);
}

// Function to handle pulse events for the first sensor
void pulse_event_handler_1(uint gpio, uint32_t events) {
    char event_str[128];
    gpio_event_string(event_str, events);

    if (gpio == gpio_pin_1 && events & GPIO_IRQ_EDGE_RISE) {
        if (!pulse_started_1) {
            pulse_start_time_1 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_1 = true;
        } else {
            // Calculate pulse duration based on the rise event
            handle_pulse_detection_1(event_str);
            pulse_started_1 = false;
        }
    } else if (gpio == gpio_pin_1 && events & GPIO_IRQ_EDGE_FALL) {
        if (!pulse_started_1) {
            // Track the time of the falling edge as the start time
            pulse_start_time_1 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_1 = true;
        } else {
            // Calculate pulse duration based on the fall event
            handle_pulse_detection_1(event_str);
            pulse_started_1 = false;
        }
    }
}

// Function to handle pulse events for the second sensor
void pulse_event_handler_2(uint gpio, uint32_t events) {
    char event_str[128];
    gpio_event_string(event_str, events);

    if (gpio == gpio_pin_2 && events & GPIO_IRQ_EDGE_RISE) {
        if (!pulse_started_2) {
            pulse_start_time_2 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_2 = true;
        } else {
            // Calculate pulse duration based on the rise event
            handle_pulse_detection_2(event_str);
            pulse_started_2 = false;
        }
    } else if (gpio == gpio_pin_2 && events & GPIO_IRQ_EDGE_FALL) {
        if (!pulse_started_2) {
            // Track the time of the falling edge as the start time
            pulse_start_time_2 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_2 = true;
        } else {
            // Calculate pulse duration based on the fall event
            handle_pulse_detection_2(event_str);
            pulse_started_2 = false;
        }
    }
}

int main() {
    stdio_init_all();

    printf("Wheel Encoder Driver\n");

    while (1) {
        // if not detecting anything default value is 1
        bool left_result = gpio_get(gpio_encoder_left);
        bool right_result = gpio_get(gpio_encoder_right);

        // if detected something it will be 0
        if(left_result == 0){
            gpio_set_irq_enabled_with_callback(gpio_pin_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_event_handler_1);
        }
        if(right_result == 0){
            gpio_set_irq_enabled_with_callback(gpio_pin_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_event_handler_2);
        }

        // Check if 2 seconds have passed since the last pulse
        uint32_t current_time = (uint32_t)time_us_32();
        
        if (print_flag_1 && current_time - last_pulse_time_1 >= pulse_print_interval) {
            printf("Sensor 1 Total Pulses: %d\n", pulse_count_1);
            print_flag_1 = false; // Reset the flag
        }

        if (print_flag_2 && current_time - last_pulse_time_2 >= pulse_print_interval) {
            printf("Sensor 2 Total Pulses: %d\n", pulse_count_2);
            print_flag_2 = false; // Reset the flag
        }

        sleep_ms(100); // Sleep for a shorter interval
    }
}