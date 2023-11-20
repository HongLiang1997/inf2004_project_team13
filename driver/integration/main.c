#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"

#define TRIG_PIN 0
#define ECHO_PIN 1
#define maxdistance 40      // maximum range of ultrasonic sensor
#define speedofsound 0.0343 // Speed of sound in cm

volatile uint32_t starttime = 0;    // timestamp for start of echo
volatile uint32_t endtime = 0;      // timestamp for end of echo
volatile bool detectedecho = false; // check if echo detected

uint trigPin = TRIG_PIN;
uint echoPin = ECHO_PIN;

// Message buffer to share distance data between tasks
MessageBufferHandle_t distanceBuffer;

// semaphore for ultrasonic
SemaphoreHandle_t echoSemaphore;

// Motor control pins
#define LEFT_WHEEL 14
#define RIGHT_WHEEL 15
#define RIGHT_WHEEL_FORWARD 18
#define RIGHT_WHEEL_BACKWARD 19
#define LEFT_WHEEL_FORWARD 20
#define LEFT_WHEEL_BACKWARD 21
#define CLK_DIV 100
#define WRAP 12500

uint slice_num_left;
uint slice_num_right;

// WHEELENCODER WHEELENCODER WHEELENCODER
//  Constants for distance and speed calculation
const float wheel_circumference = 21;     // Circumference of the wheel in centimeters
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

// Define a constant for the minimum time between prints (in microseconds)
#define MIN_PRINT_INTERVAL_US 2000000 // 2 seconds

// Define a variable to track the last time the function printed
volatile uint32_t last_print_time_1 = 0;
volatile uint32_t last_print_time_2 = 0;

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
  
        uint32_t pulse_end_time = (uint32_t)time_us_32(); // Convert to uint32_t
        uint32_t pulse_duration_us = pulse_end_time - pulse_start_time_1;
        float speed_mps = ((wheel_circumference / pulses_per_revolution) / 100) / (pulse_duration_us / 1e6); // Convert to meters per second, distance over time
        pulse_count_1++;
        last_pulse_time_1 = (uint32_t)time_us_32(); // Update the last pulse time

        // Print only if the time condition is met
        printf("Sensor 1 - Pulse detected - Distance: %.2f m, Speed: %.2f m/s, Events: %s\n", (pulse_count_1 * (wheel_circumference / pulses_per_revolution)) / 100, speed_mps, event_str);

       
    
}

// Function to handle pulse detection and printing for second sensor
void handle_pulse_detection_2(const char *event_str)
{

        uint32_t pulse_end_time = (uint32_t)time_us_32(); // Convert to uint32_t
        uint32_t pulse_duration_us = pulse_end_time - pulse_start_time_2;
        float speed_mps = ((wheel_circumference / pulses_per_revolution) / 100) / (pulse_duration_us / 1e6); // Convert to meters per second, distance over time
        pulse_count_2++;
        last_pulse_time_2 = (uint32_t)time_us_32(); // Update the last pulse time
        print_flag_2 = true;                        // Set the flag to indicate that a print is needed
        printf("Sensor 2 - Pulse detected - Distance: %.2f m, Speed: %.2f m/s, Events: %s\n", (pulse_count_2 * (wheel_circumference / pulses_per_revolution)) / 100, speed_mps, event_str);

     
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
    pwm_set_clkdiv(slice_num_left, CLK_DIV);
    pwm_set_wrap(slice_num_left, WRAP);
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, WRAP * speed);
    pwm_set_enabled(slice_num_left, true);
}

void set_right_speed(float speed)
{
    pwm_set_clkdiv(slice_num_right, CLK_DIV);
    pwm_set_wrap(slice_num_right, WRAP);
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, WRAP * speed);
    pwm_set_enabled(slice_num_right, true);
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

void turn_right(void *params)
{
    gpio_put(RIGHT_WHEEL_FORWARD, 0);
    gpio_put(RIGHT_WHEEL_BACKWARD, 1);
    gpio_put(LEFT_WHEEL_FORWARD, 1);
    gpio_put(LEFT_WHEEL_BACKWARD, 0);
}

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
    float distance;
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

// IR Sensor control task
void ir_sensor_control_task(void *pvParameters)
{
    while (1)
    {
        bool left_result = gpio_get(26);  // Read from the left sensor (GPIO 26)
        bool right_result = gpio_get(27); // Read from the right sensor (GPIO 27)

        // Default State IR Sensor will return 1
        printf("Left Sensor Reading: %d\n", left_result);
        printf("Right Sensor Reading: %d\n", right_result);

        if (left_result == 0 && right_result == 0)
        {
            // Both sensors detect a black line.
            // This indicates that the car is directly on top of both lines.
            // You can perform a specific action like stopping or making corrections.
            printf("left right black\n");
        }
        else if (left_result == 0 && right_result == 1)
        {
            // Left sensor detects a black line, but right sensor doesn't.
            // This means the car is deviating towards the left line.
            // You can steer the car to the right to get back on track. By making Left Wheel rotation higher
            printf("left black right white\n");
        }
        else if (left_result == 1 && right_result == 0)
        {
            // Right sensor detects a black line, but left sensor doesn't.
            // This means the car is deviating towards the right line.
            // You can steer the car to the left to get back on track. By making Right Wheel rotation higher
            printf("left white right black\n");
        }
        else if (left_result == 1 && right_result == 1)
        {
            // Both sensors do not detect a black line.
            // The car is off the lines.
            // You can perform a specific action, such as searching for the lines.
            printf("left right white\n");
        }

        // sleep_ms(1000); // Wait for a moment before taking the next reading
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust the delay as needed
    }
}

// Motor control task
void motor_control_task(void *pvParameters)
{
    float distance;

    while (1)
    {
        // Receive distance data from the message buffer
        xMessageBufferReceive(distanceBuffer, &distance, sizeof(float), portMAX_DELAY);

        if (distance <= 20.0)
        {
            stop(NULL);
        }
        else
        {
            // Adjust the motor speed based on the distance
            float speed = 0.8; // Set an initial speed (you can change this value)
            move_forward(NULL);
            set_left_speed(speed);
            set_right_speed(speed);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as needed
    }
}

// Ultrasonic sensor task using polling
void ultrasonic_polling_task(void *pvParameters) {
    float distance;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize the last wake time

    while (1) {
        gpio_put(trigPin, 1);
        sleep_us(10);
        gpio_put(trigPin, 0);

        // Start a timer to measure the echo pulse duration
        uint32_t start_time = time_us_32();
        uint32_t echo_timeout = 50000; // Set an appropriate timeout value
        uint32_t echo_duration = 0;

        while (gpio_get(ECHO_PIN) == 0) {
            if (time_us_32() - start_time > echo_timeout) {
                // Handle timeout condition
                break;
            }
        }

        start_time = time_us_32();

        while (gpio_get(ECHO_PIN) == 1) {
            echo_duration = time_us_32() - start_time;
        }

        uint32_t distance_cm = (echo_duration * speedofsound) / 2;

        if (distance_cm <= maxdistance) {
            distance = (float)distance_cm;
            printf("Distance: %0.2f cm\n", distance);
        } else {
            printf("Out of range\n");
        }

        // Send the distance data to the message buffer
        xMessageBufferSend(distanceBuffer, &distance, sizeof(float), portMAX_DELAY);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500)); // delay for 500 milliseconds. finetune for frequency of distance
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
    xTaskCreate(ultrasonic_polling_task, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);

    // Create the motor control task with second highest priority
    xTaskCreate(motor_control_task, "MotorControlTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);

    // Create the IR sensor task with third highest priority
    xTaskCreate(ir_sensor_control_task, "IRSensorControlTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);

    // Create the wheel encoder task with lowest priority
    xTaskCreate(encoder_task, "EncoderTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Code should never reach here if FreeRTOS is set up correctly
    }
    return 0;
}

// Integrated wheel encoder