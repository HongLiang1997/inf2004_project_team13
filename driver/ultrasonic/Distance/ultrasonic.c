#include "pico/stdlib.h"
#include <stdio.h>
#include "ultrasonic.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

uint trigPin = 0;
uint echoPin = 1;

// Define a message buffer handle
static MessageBufferHandle_t xUltrasonicMessageBuffer;


// Define the handler for the ultrasonic sensor interrupt
void ultrasonic_interrupt_handler_task(void *pvParameters) {
    float distance;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize the last wake time

    while (1) {
        xMessageBufferReceive(xUltrasonicMessageBuffer, (void *)&distance, sizeof(float), portMAX_DELAY);
        if(distance<10.0)
        {
            printf("Object at %0.2f cm\n", distance); // Print the received distance only if < 10cm
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500)); // Wait for half second
    }
}

void distance_measurement_task(void *pvParameters) {
    setupUltrasonicPins(trigPin, echoPin);
    float distance;
    
    while (1) {
        distance = getCm(trigPin, echoPin);
        xMessageBufferSend(xUltrasonicMessageBuffer, (void *)&distance, sizeof(float), 0);
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
    }
}

int main() {
    stdio_init_all();

    xUltrasonicMessageBuffer = xMessageBufferCreate(sizeof(float) * 2); // Adjust the size as needed


    // Create the distance_measurement_task and ultrasonic_interrupt_task task
    xTaskCreate(distance_measurement_task, "DistanceTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(ultrasonic_interrupt_handler_task, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);


    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1) {
        // Code should never reach here if FreeRTOS is set up correctly
    }
}
