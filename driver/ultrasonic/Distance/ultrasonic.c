#include "pico/stdlib.h"
#include <stdio.h>
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


// define the handler for the ultrasonic sensor interrupt
void ultrasonic_interrupt_handler_task(void *pvParameters)
{
    float distance;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize the last wake time

    while (1)
    {
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

        detectedecho = false;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500)); // delay for 500 milliseconds. finetune for frequency of distance
    }
}


// ISR for the ECHO pin events
void echo_isr(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        starttime = time_us_32(); // capture timestamp for the rising edge of the echo pulse
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        endtime = time_us_32(); // capture timestamp for the falling edge of the echo pulse
        detectedecho = true; // Set the detectedecho flag to true
    }
}

// initialize GPIO pins for the ultrasonic sensor
void init_ultrasonic_pins() {
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_isr);

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
}



int main() {
    stdio_init_all(); // initialize standard I/O
    init_ultrasonic_pins(); // initialize GPIO pins for the ultrasonic sensor

    // delay for 2 seconds before starting ultrasonic task.
    // cannot be too fast or won't initialize properly.
    sleep_ms(2000);  

    // create the ultrasonic_interrupt_handler_task task
    xTaskCreate(ultrasonic_interrupt_handler_task, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    // start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1) {
        // code should never reach here if FreeRTOS is set up correctly
    }
    return 0;
}
