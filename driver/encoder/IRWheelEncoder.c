#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Constants for distance and speed calculation
const float wheel_circumference = 80.896; // Circumference of the wheel in millimeters
const float pulses_per_revolution = 20.0; // Number of slots on the encoder disc

// Variables to keep track of state
volatile uint32_t pulse_count = 0;
uint32_t pulse_start_time = 0; // Change pulse_start_time to uint32_t
bool pulse_started = false;
uint32_t last_edge_fall_time = 0; // Track the time of the last falling edge

// Variables to track the time since the last pulse and printing flag
uint32_t last_pulse_time = 0;
bool print_flag = false;
const uint32_t pulse_print_interval = 2000000; // 2 seconds

// GPIO pin configuration
const uint gpio_pin = 2;

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

// Function to handle pulse detection and printing
void handle_pulse_detection(const char *event_str) {
    uint32_t pulse_end_time = (uint32_t)time_us_32(); // Convert to uint32_t
    uint32_t pulse_duration_us = pulse_end_time - pulse_start_time;
    float speed_mps = (wheel_circumference / 1000.0) / (pulse_duration_us / 1e6); // Convert to meters per second
    pulse_count++;
    last_pulse_time = (uint32_t)time_us_32(); // Update the last pulse time
    print_flag = true; // Set the flag to indicate that a print is needed
    printf("Pulse detected - Distance: %.2f m, Speed: %.2f m/s, Events: %s\n", (pulse_count * (wheel_circumference / pulses_per_revolution)) / 100, speed_mps, event_str);
}

// Function to handle pulse events
void pulse_event_handler(uint gpio, uint32_t events) {
    char event_str[128];
    gpio_event_string(event_str, events);

    if (events & GPIO_IRQ_EDGE_RISE) {
        if (!pulse_started) {
            pulse_start_time = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started = true;
        } else {
            // Calculate pulse duration based on the rise event
            handle_pulse_detection(event_str);
            pulse_started = false;
        }
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        if (!pulse_started) {
            // Track the time of the falling edge as the start time
            pulse_start_time = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started = true;
        } else {
            // Calculate pulse duration based on the fall event
            handle_pulse_detection(event_str);
            pulse_started = false;
        }
    }
}

int main() {
    stdio_init_all();
    gpio_set_irq_enabled_with_callback(gpio_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_event_handler);

    printf("Wheel Encoder Driver\n");

    while (1) {
        // Check if 2 seconds have passed since the last pulse
        uint32_t current_time = (uint32_t)time_us_32();
        if (print_flag && current_time - last_pulse_time >= pulse_print_interval) {
            printf("Total Pulses: %d\n", pulse_count);
            print_flag = false; // Reset the flag
        }
        sleep_ms(100); // Sleep for a shorter interval
    }
}