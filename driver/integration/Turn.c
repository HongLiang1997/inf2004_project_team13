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

// Motor control
#define LEFT_WHEEL 15
#define RIGHT_WHEEL 14
#define RIGHT_WHEEL_FORWARD 20
#define RIGHT_WHEEL_BACKWARD 21
#define LEFT_WHEEL_FORWARD 18
#define LEFT_WHEEL_BACKWARD 19
#define CLK_DIV 100
#define WRAP 12500

// Variables
// Ultrasonic
volatile uint32_t starttime = 0;    // timestamp for start of echo
volatile uint32_t endtime = 0;      // timestamp for end of echo
volatile bool detectedecho = false; // check if echo detected
uint trigPin = TRIG_PIN;
uint echoPin = ECHO_PIN;
MessageBufferHandle_t distanceBuffer; // Message buffer to share distance data between tasks
SemaphoreHandle_t echoSemaphore;      // semaphore for ultrasonic

// Motor Control
uint slice_num_left;
uint slice_num_right;
float distance = 21.0;
float left_new = 0;
float right_new = 0;
volatile bool white_surface_detected = false;
absolute_time_t white_surface_start_time;
absolute_time_t black_surface_start_time;
uint32_t elapsed_seconds = 0;
float speed_right = 0.52; // Set an initial speed (LEFT)
float speed_left = 0.56;  // Set an initial speed (RIGHT)

// Functions
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

// Adjust left speed
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

// Adjust right speed
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
        sleep_ms(400);
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
        sleep_ms(600);
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
        }
        else if (left_result && right_result)
        {
            // stop(NULL);
            set_left_speed(0.8);
            set_right_speed(0.8);
            vTaskDelay(pdMS_TO_TICKS(100));
            turn_right_90();
            printf("turn\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            set_left_speed(speed_left);
            set_right_speed(speed_right);
        }
        else if (left_result)
        {

            // This is turn right
            set_left_speed(0.8);
            set_right_speed(0.8);
            vTaskDelay(pdMS_TO_TICKS(100));
            turn_right_90();
            printf("turn\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            set_left_speed(speed_left);
            set_right_speed(speed_right);
        }
        else if (right_result)
        {

            // decrease_right_speed();
            // increase_left_speed();
            // printf("adjust right, L: %f R: %f", left_new, right_new);

            set_left_speed(0.8);
            set_right_speed(0.8);
            vTaskDelay(pdMS_TO_TICKS(100));
            turn_right_90();
            printf("turn\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            set_left_speed(speed_left);
            set_right_speed(speed_right);
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

//Ultrasonic 
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

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Code should never reach here if FreeRTOS is set up correctly
    }
    return 0;
}