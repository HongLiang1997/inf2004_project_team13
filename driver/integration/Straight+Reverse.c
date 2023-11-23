#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"
#include <string.h>

// Defines
// Ultrasonic
#define TRIG_PIN 0
#define ECHO_PIN 1
#define maxdistance 40      // maximum range of ultrasonic sensor
#define speedofsound 0.0343 // Speed of sound in cm

// Motor
#define LEFT_WHEEL 15
#define RIGHT_WHEEL 14
#define RIGHT_WHEEL_FORWARD 20
#define RIGHT_WHEEL_BACKWARD 21
#define LEFT_WHEEL_FORWARD 18
#define LEFT_WHEEL_BACKWARD 19
#define CLK_DIV 100
#define WRAP 12500

// Wheelencoder
#define MIN_PRINT_INTERVAL_US 2000000 // Define a constant for the minimum time between prints 2 seconds (in microseconds)

// Variables
// Ultrasonic
float distance = 21.0;
volatile uint32_t starttime = 0;    // timestamp for start of echo
volatile uint32_t endtime = 0;      // timestamp for end of echo
volatile bool detectedecho = false; // check if echo detected
uint trigPin = TRIG_PIN;
uint echoPin = ECHO_PIN;
MessageBufferHandle_t distanceBuffer; // Message buffer to share distance data between tasks
SemaphoreHandle_t echoSemaphore;

// Motorcontrol
uint slice_num_left;
uint slice_num_right;
float left_new = 0;       // initalise variable to set new left speed
float right_new = 0;      // initalise variable to set new right speed
float speed_right = 0.52; // Set an initial speed (LEFT)
float speed_left = 0.56;  // Set an initial speed (RIGHT)

// WHEELENCODER
const float wheel_circumference = 21;     // Circumference of the wheel in centimeters
const float pulses_per_revolution = 20.0; // Number of slots on the encoder disc
volatile uint32_t pulse_count_1 = 0;      //  Count number of pulses for wheel revolution
uint32_t pulse_start_time_1 = 0;          // Change pulse_start_time_1 to uint32_t
bool pulse_started_1 = false;             // Flag for pulse_start
uint32_t last_edge_fall_time_1 = 0;       // Track the time of the last falling edge
uint32_t last_pulse_time_1 = 0;           // Variables to track the time since the last pulse and printing flag
bool print_flag_1 = false;                // Variables to track the time since the last pulse and printing flag

// Variables to keep track of state for the second sensor (gp3)
volatile uint32_t pulse_count_2 = 0;           //  Count number of pulses for wheel revolution
uint32_t pulse_start_time_2 = 0;               // Change pulse_start_time_1 to uint32_t
bool pulse_started_2 = false;                  // Flag for pulse_start
uint32_t last_edge_fall_time_2 = 0;            // Track the time of the last falling edge
uint32_t last_pulse_time_2 = 0;                // Variables to track the time since the last pulse and printing flag
bool print_flag_2 = false;                     // Variables to track the time since the last pulse and printing flag
const uint32_t pulse_print_interval = 2000000; // 2 seconds Fixed interval for pulse count 1 and 2 (both IR sensors)
const uint gpio_pin_1 = 2;                     // GPIO pin configuration for the first sensor
uint gpio_encoder_left = 2;                    // for detection of left interrupt
const uint gpio_pin_2 = 3;                     // GPIO pin configuration for the second sensor
uint gpio_encoder_right = 3;                   // for detection of right interrupt
volatile uint32_t last_print_time_1 = 0;       // Define a variable to track the last time the function printed
volatile uint32_t last_print_time_2 = 0;       // Define a variable to track the last time the function printed
static const char *gpio_irq_str[] = {
    // struct for wheelencoder readings
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

// Functions

// Wheelencoder functions
void gpio_event_string(char *buf, uint32_t events)
{
    for (uint i = 0; i < 4; i++)
    {
        uint mask = (1 << i);
        if (events & mask)
        {
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0')
            {
                *buf++ = *event_str++;
            }
            events &= ~mask;
            if (events)
            {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

// Function to handle pulse detection and printing for first sensor
void handle_pulse_detection_1(const char *event_str)
{
    uint32_t current_time = (uint32_t)time_us_32(); // Get the current time

    if (current_time - last_print_time_1 >= MIN_PRINT_INTERVAL_US)
    {
        // Sufficient time has passed since the last print
        // Perform the print operation
        uint32_t pulse_end_time = (uint32_t)time_us_32(); // Convert to uint32_t
        uint32_t pulse_duration_us = pulse_end_time - pulse_start_time_1;
        float speed_mps = ((wheel_circumference / pulses_per_revolution) / 100) / (pulse_duration_us / 1e6); // Convert to meters per second, distance over time
        pulse_count_1++;
        last_pulse_time_1 = (uint32_t)time_us_32(); // Update the last pulse time

        // Print only if the time condition is met
        printf("Sensor 1 - Pulse detected - Distance: %.2f m, Speed: %.2f m/s, Events: %s\n", (pulse_count_1 * (wheel_circumference / pulses_per_revolution)) / 100, speed_mps, event_str);

        last_print_time_1 = current_time; // Update the last print time
    }
}

// Function to handle pulse detection and printing for second sensor
void handle_pulse_detection_2(const char *event_str)
{
    uint32_t current_time = (uint32_t)time_us_32(); // Get the current time

    if (current_time - last_print_time_2 >= MIN_PRINT_INTERVAL_US)
    {
        // Sufficient time has passed since the last print
        // Perform the print operation
        uint32_t pulse_end_time = (uint32_t)time_us_32(); // Convert to uint32_t
        uint32_t pulse_duration_us = pulse_end_time - pulse_start_time_2;
        float speed_mps = ((wheel_circumference / pulses_per_revolution) / 100) / (pulse_duration_us / 1e6); // Convert to meters per second, distance over time
        pulse_count_2++;
        last_pulse_time_2 = (uint32_t)time_us_32(); // Update the last pulse time

        // Print only if the time condition is met
        printf("Sensor 2 - Pulse detected - Distance: %.2f m, Speed: %.2f m/s, Events: %s\n", (pulse_count_2 * (wheel_circumference / pulses_per_revolution)) / 100, speed_mps, event_str);

        last_print_time_2 = current_time; // Update the last print time
    }
}

// Function to handle pulse events for the first sensor
void pulse_event_handler_1(uint gpio, uint32_t events)
{
    char event_str[128];
    gpio_event_string(event_str, events);

    if (gpio == gpio_pin_1 && events & GPIO_IRQ_EDGE_RISE)
    {
        if (!pulse_started_1)
        {
            pulse_start_time_1 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_1 = true;
        }
        else
        {
            // Calculate pulse duration based on the rise event
            handle_pulse_detection_1(event_str);
            pulse_started_1 = false;
        }
    }
    else if (gpio == gpio_pin_1 && events & GPIO_IRQ_EDGE_FALL)
    {
        if (!pulse_started_1)
        {
            // Track the time of the falling edge as the start time
            pulse_start_time_1 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_1 = true;
        }
        else
        {
            // Calculate pulse duration based on the fall event
            handle_pulse_detection_1(event_str);
            pulse_started_1 = false;
        }
    }
}

// Function to handle pulse events for the second sensor
void pulse_event_handler_2(uint gpio, uint32_t events)
{
    char event_str[128];
    gpio_event_string(event_str, events);

    if (gpio == gpio_pin_2 && events & GPIO_IRQ_EDGE_RISE)
    {
        if (!pulse_started_2)
        {
            pulse_start_time_2 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_2 = true;
        }
        else
        {
            // Calculate pulse duration based on the rise event
            handle_pulse_detection_2(event_str);
            pulse_started_2 = false;
        }
    }
    else if (gpio == gpio_pin_2 && events & GPIO_IRQ_EDGE_FALL)
    {
        if (!pulse_started_2)
        {
            // Track the time of the falling edge as the start time
            pulse_start_time_2 = (uint32_t)time_us_32(); // Convert to uint32_t
            pulse_started_2 = true;
        }
        else
        {
            // Calculate pulse duration based on the fall event
            handle_pulse_detection_2(event_str);
            pulse_started_2 = false;
        }
    }
}

// wheel Encoder Task
void encoder_task(void *pvParameters)
{
    (void)pvParameters; // Unused parameter

    while (1)
    {
        bool left_result = gpio_get(gpio_encoder_left);
        bool right_result = gpio_get(gpio_encoder_right);

        if (left_result == 0)
        {
            gpio_set_irq_enabled_with_callback(gpio_pin_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_event_handler_1);
        }
        if (right_result == 0)
        {
            gpio_set_irq_enabled_with_callback(gpio_pin_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_event_handler_2);
        }

        uint32_t current_time = (uint32_t)time_us_32();

        if (print_flag_1 && current_time - last_pulse_time_1 >= pulse_print_interval)
        {
            printf("Sensor 1 Total Pulses: %d\n", pulse_count_1);
            print_flag_1 = false; // Reset the flag
        }

        if (print_flag_2 && current_time - last_pulse_time_2 >= pulse_print_interval)
        {
            printf("Sensor 2 Total Pulses: %d\n", pulse_count_2);
            print_flag_2 = false; // Reset the flag
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as needed
    }
}

// Motor control functions
void init_motor(void *params)
{
    gpio_set_function(LEFT_WHEEL, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_WHEEL, GPIO_FUNC_PWM);

    slice_num_left = pwm_gpio_to_slice_num(LEFT_WHEEL);
    slice_num_right = pwm_gpio_to_slice_num(RIGHT_WHEEL);

    gpio_init(RIGHT_WHEEL_FORWARD);
    gpio_init(RIGHT_WHEEL_BACKWARD);
    gpio_init(LEFT_WHEEL_FORWARD);
    gpio_init(LEFT_WHEEL_BACKWARD);

    gpio_set_dir(RIGHT_WHEEL_FORWARD, GPIO_OUT);
    gpio_set_dir(RIGHT_WHEEL_BACKWARD, GPIO_OUT);
    gpio_set_dir(LEFT_WHEEL_FORWARD, GPIO_OUT);
    gpio_set_dir(LEFT_WHEEL_BACKWARD, GPIO_OUT);
}

void set_left_speed(float speed)
{
    left_new = WRAP * speed;
    if (left_new < 0)
    {
        left_new = 0;
    }
    else if (left_new > 12500)
    {
        left_new = 12500;
    }
    pwm_set_clkdiv(slice_num_left, CLK_DIV);
    pwm_set_wrap(slice_num_left, WRAP);
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, left_new);
    pwm_set_enabled(slice_num_left, true);
}

void set_right_speed(float speed)
{
    right_new = WRAP * speed;
    if (right_new < 0)
    {
        right_new = 0;
    }
    else if (right_new > 12500)
    {
        right_new = 12500;
    }
    pwm_set_clkdiv(slice_num_right, CLK_DIV);
    pwm_set_wrap(slice_num_right, WRAP);
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, right_new);
    pwm_set_enabled(slice_num_right, true);
}

void increase_left_speed(void)
{
    left_new += 500;
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, left_new);
}

void decrease_left_speed(void)
{
    left_new -= 500;
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, left_new);
}

void increase_right_speed(void)
{
    right_new += 500;
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, right_new);
}

void decrease_right_speed(void)
{
    right_new -= 500;
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, right_new);
}

void stop(void *params)
{
    gpio_put(RIGHT_WHEEL_FORWARD, 0);
    gpio_put(RIGHT_WHEEL_BACKWARD, 0);
    gpio_put(LEFT_WHEEL_FORWARD, 0);
    gpio_put(LEFT_WHEEL_BACKWARD, 0);
}

void move_forward(void *params)
{
    gpio_put(RIGHT_WHEEL_FORWARD, 1);
    gpio_put(RIGHT_WHEEL_BACKWARD, 0);
    gpio_put(LEFT_WHEEL_FORWARD, 1);
    gpio_put(LEFT_WHEEL_BACKWARD, 0);
}

void move_backward(void *params)
{
    gpio_put(RIGHT_WHEEL_FORWARD, 0);
    gpio_put(RIGHT_WHEEL_BACKWARD, 1);
    gpio_put(LEFT_WHEEL_FORWARD, 0);
    gpio_put(LEFT_WHEEL_BACKWARD, 1);
}

void turn_left(void *params)
{
    gpio_put(RIGHT_WHEEL_FORWARD, 1);
    gpio_put(RIGHT_WHEEL_BACKWARD, 0);
    gpio_put(LEFT_WHEEL_FORWARD, 0);
    gpio_put(LEFT_WHEEL_BACKWARD, 1);
}

void turn_left_90(void)
{
    turn_left(NULL);
    uint64_t startime = time_us_64();
    while (time_us_64() - startime < 400000)
    {
        sleep_ms(500);
    }
    stop(NULL);
}

void turn_right(void *params)
{
    gpio_put(RIGHT_WHEEL_FORWARD, 0);
    gpio_put(RIGHT_WHEEL_BACKWARD, 1);
    gpio_put(LEFT_WHEEL_FORWARD, 1);
    gpio_put(LEFT_WHEEL_BACKWARD, 0);
}

void turn_right_90(void)
{
    turn_right(NULL);
    uint64_t startime = time_us_64();
    while (time_us_64() - startime < 400000)
    {
        sleep_ms(500);
    }
    stop(NULL);
}

// Motor control task
void motor_control_task(void *pvParameters)
{

    while (1)
    {
        // Receive distance data from the message buffer
        xMessageBufferReceive(distanceBuffer, &distance, sizeof(float), portMAX_DELAY);
        bool left_result = gpio_get(26);  // Read from the left sensor (GPIO 26)
        bool right_result = gpio_get(27); // Read from the right sensor (GPIO 27)

        // Default State IR Sensor will return 1
        printf("Left Sensor Reading: %d\n", left_result);
        printf("Right Sensor Reading: %d\n", right_result);

        if (distance <= 20.0)
        {
            stop(NULL);
            set_left_speed(speed_left);
            set_right_speed(speed_right);

            printf("stop, L: %f R: %f", left_new, right_new);
            move_backward(NULL);
            vTaskDelay(pdMS_TO_TICKS(1000));
            stop(NULL);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (left_result && right_result)
        {
            stop(NULL);
        }
        else if (left_result)
        {
            decrease_left_speed();
            increase_right_speed();
            printf("adjust left, L: %f R: %f", left_new, right_new);
        }
        else if (right_result)
        {
            decrease_right_speed();
            increase_left_speed();
            printf("adjust right, L: %f R: %f", left_new, right_new);
        }
        else if (!left_result && !right_result)
        {
            set_left_speed(speed_left);
            set_right_speed(speed_right);
            move_forward(NULL);
            printf("stop, L: %f R: %f", left_new, right_new);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Adjust the delay as needed
    }
}

// Ultrasonic
// Ultrasonic sensor interrupt handler
// Ultrasonic sensor interrupt handler
void echo_isr(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        starttime = time_us_32();
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        endtime = time_us_32();
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(echoSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Initialize GPIO pins for the ultrasonic sensor
void init_ultrasonic_pins()
{
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_isr);

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
}

// Initialize GPIO pins for the IR sensor
void init_ir_pins()
{
    gpio_init(26); // Left sensor connected to GPIO 26
    gpio_init(27); // Right sensor connected to GPIO 27
    gpio_set_dir(26, GPIO_IN);
    gpio_set_dir(27, GPIO_IN);
}

// Ultrasonic sensor task
void ultrasonic_interrupt_handler_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        gpio_put(trigPin, 1);
        sleep_us(10);
        gpio_put(trigPin, 0);

        if (xSemaphoreTake(echoSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            uint32_t pulseduration = endtime - starttime;

            uint32_t distance_cm = (pulseduration * speedofsound) / 2;

            if (distance_cm <= maxdistance)
            {
                distance = (float)distance_cm;
                printf("Distance: %0.2f cm\n", distance);
            }
            else
            {
                printf("Out of range\n");
            }

            xMessageBufferSend(distanceBuffer, &distance, sizeof(float), portMAX_DELAY);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

int main()
{
    echoSemaphore = xSemaphoreCreateBinary();
    stdio_init_all();
    init_ultrasonic_pins();
    init_ir_pins();
    init_motor(NULL);

    distanceBuffer = xMessageBufferCreate(sizeof(float) * 2);

    sleep_ms(2000);

    // Create the ultrasonic task with highest priority
    xTaskCreate(ultrasonic_interrupt_handler_task, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);

    // Create the motor control task with second highest priority
    xTaskCreate(motor_control_task, "MotorControlTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);

    
    // Create the wheel encoder task with lowest priority
    //xTaskCreate(encoder_task, "EncoderTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Code should never reach here if FreeRTOS is set up correctly
    }
    return 0;
}