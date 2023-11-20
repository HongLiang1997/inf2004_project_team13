#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

#define TRIG_PIN 0
#define ECHO_PIN 1
#define maxdistance 40 // maximum range of ultrasonic sensor
#define speedofsound 0.0343 // Speed of sound in cm

volatile uint32_t starttime = 0; // timestamp for start of echo
volatile uint32_t endtime = 0;   // timestamp for end of echo
volatile bool detectedecho = false; // check if echo detected

uint trigPin = TRIG_PIN;
uint echoPin = ECHO_PIN;

// Message buffer to share distance data between tasks
MessageBufferHandle_t distanceBuffer;

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

// Motor control functions
void init_motor(void *params) {
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

void set_left_speed(float speed) {
    pwm_set_clkdiv(slice_num_left, CLK_DIV);
    pwm_set_wrap(slice_num_left, WRAP);
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, WRAP * speed);
    pwm_set_enabled(slice_num_left, true);
}

void set_right_speed(float speed) {
    pwm_set_clkdiv(slice_num_right, CLK_DIV);
    pwm_set_wrap(slice_num_right, WRAP);
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, WRAP * speed);
    pwm_set_enabled(slice_num_right, true);
}

void stop(void *params) {
    gpio_put(RIGHT_WHEEL_FORWARD, 0);
    gpio_put(RIGHT_WHEEL_BACKWARD, 0);
    gpio_put(LEFT_WHEEL_FORWARD, 0);
    gpio_put(LEFT_WHEEL_BACKWARD, 0);
}

void move_forward(void *params) {
    gpio_put(RIGHT_WHEEL_FORWARD, 1);
    gpio_put(RIGHT_WHEEL_BACKWARD, 0);
    gpio_put(LEFT_WHEEL_FORWARD, 1);
    gpio_put(LEFT_WHEEL_BACKWARD, 0);
}

void move_backward(void *params) {
    gpio_put(RIGHT_WHEEL_FORWARD, 0);
    gpio_put(RIGHT_WHEEL_BACKWARD, 1);
    gpio_put(LEFT_WHEEL_FORWARD, 0);
    gpio_put(LEFT_WHEEL_BACKWARD, 1);
}

void turn_left(void *params) {
    gpio_put(RIGHT_WHEEL_FORWARD, 1);
    gpio_put(RIGHT_WHEEL_BACKWARD, 0);
    gpio_put(LEFT_WHEEL_FORWARD, 0);
    gpio_put(LEFT_WHEEL_BACKWARD, 1);
}

void turn_right(void *params) {
    gpio_put(RIGHT_WHEEL_FORWARD, 0);
    gpio_put(RIGHT_WHEEL_BACKWARD, 1);
    gpio_put(LEFT_WHEEL_FORWARD, 1);
    gpio_put(LEFT_WHEEL_BACKWARD, 0);
}

// Ultrasonic sensor interrupt handler
void echo_isr(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        starttime = time_us_32(); // capture timestamp for the rising edge of the echo pulse
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        endtime = time_us_32(); // capture timestamp for the falling edge of the echo pulse
        detectedecho = true; // Set the detectedecho flag to true
    }
}

// Initialize GPIO pins for the ultrasonic sensor
void init_ultrasonic_pins() {
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_isr);

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
}

// Initialize GPIO pins for the IR sensor
void init_ir_pins(){
    gpio_init(26); // Left sensor connected to GPIO 26
    gpio_init(27); // Right sensor connected to GPIO 27
    gpio_set_dir(26, GPIO_IN);
    gpio_set_dir(27, GPIO_IN);
}

// Ultrasonic sensor task
void ultrasonic_interrupt_handler_task(void *pvParameters) {
    float distance;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize the last wake time

    while (1) {
        gpio_put(trigPin, 1);
        sleep_us(10);
        gpio_put(trigPin, 0);

        while (!detectedecho) {} // Wait for an echo to be detected

        uint32_t pulseduration = endtime - starttime;

        uint32_t distance_cm = (pulseduration * speedofsound) / 2;

        if (distance_cm <= maxdistance) {
            distance = (float)distance_cm;
            printf("Distance: %0.2f cm\n", distance);
        } else {
            printf("Out of range\n");
        }

        // Send the distance data to the message buffer
        xMessageBufferSend(distanceBuffer, &distance, sizeof(float), portMAX_DELAY);

        detectedecho = false;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500)); // delay for 500 milliseconds. finetune for frequency of distance
    }
}

// IR Sensor control task
void ir_sensor_control_task(void *pvParameters){
    while (1) {
        bool left_result = gpio_get(26);   // Read from the left sensor (GPIO 26)
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
void motor_control_task(void *pvParameters) {
    float distance;

    while (1) {
        // Receive distance data from the message buffer
        xMessageBufferReceive(distanceBuffer, &distance, sizeof(float), portMAX_DELAY);

        if (distance <= 20.0) {
            stop(NULL);
        } else {
            // Adjust the motor speed based on the distance
            float speed = 0.8; // Set an initial speed (you can change this value)
            move_forward(NULL);
            set_left_speed(speed);
            set_right_speed(speed);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as needed
    }
}

int main() {
    stdio_init_all();
    init_ultrasonic_pins();
    init_ir_pins();
    init_motor(NULL);

    distanceBuffer = xMessageBufferCreate(sizeof(float) * 2);

    sleep_ms(2000);

    // Create the ultrasonic_interrupt_handler_task task
    xTaskCreate(ultrasonic_interrupt_handler_task, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    // Create the ir_sensor_control_task
    xTaskCreate(ir_sensor_control_task, "IRSensorControlTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    // Create the motor_control_task
    xTaskCreate(motor_control_task, "MotorControlTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1) {
        // Code should never reach here if FreeRTOS is set up correctly
    }
     return 0;
}
