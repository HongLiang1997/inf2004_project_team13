
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/time.h"


#define THRESHOLD 1100
#define IR_SENSOR_PIN 26
#define INITIAL_SIZE 5

volatile bool white_surface_detected = false;
absolute_time_t white_surface_start_time;
absolute_time_t black_surface_start_time;
uint32_t elapsed_seconds = 0;

int *array;
int size, capacity;

// Define a struct to hold the Code 39 characters and their binary representations
struct Code39Character
{
    char character;
    const char *binary;
};

// Function to find the Code 39 character from a binary string
char findCode39Character(const char *barcodeString, const struct Code39Character *characters, int numCharacters)
{
    for (int i = 0; i < numCharacters; i++)
    {
        if (strcmp(barcodeString, characters[i].binary) == 0)
        {
            return characters[i].character;
        }
    }

    return '?'; // Character not found
}

// uint32_t blackrecord;
// uint32_t whiterecord;

// Function to initialize the dynamic array
void initializeDynamicArray(int **array, int *size, int *capacity)
{
    *size = 0;
    *capacity = INITIAL_SIZE;
    *array = (int *)malloc(INITIAL_SIZE * sizeof(int));
    if (*array == NULL)
    {
        fprintf(stderr, "Memory allocation failed.\n");
        exit(1);
    }
}

// Function to append a value to the dynamic array
void appendValue(int **array, int *size, int *capacity, int value)
{
    if (*size >= *capacity)
    {
        // If the current size exceeds the capacity, reallocate more space
        *capacity *= 2;
        *array = (int *)realloc(*array, *capacity * sizeof(int));
        if (*array == NULL)
        {
            fprintf(stderr, "Memory reallocation failed.\n");
            exit(1);
        }
    }

    (*array)[(*size)++] = value;
}

void freeDynamicArray(int *array)
{
    free(array);
}

// Function to concatenate values in the dynamic array into a string
char *concatenateArrayToString(int *array, int size)
{
    char *result = (char *)malloc((size * 4 + 1) * sizeof(char)); // +1 for the null-terminator
    if (result == NULL)
    {
        fprintf(stderr, "Memory allocation failed.\n");
        exit(1);
    }

    int offset = 0;
    for (int i = 0; i < size; i++)
    {
        int value = array[i];
        // Convert the integer to a string and append it to the result
        int written = snprintf(result + offset, (size * 4 + 1) - offset, "%d", value);
        if (written < 0 || written >= (size * 4 + 1) - offset)
        {
            fprintf(stderr, "Error converting integer to string.\n");
            free(result);
            exit(1);
        }
        offset += written;
    }
    result[offset] = '\0'; // Null-terminate the string

    return result;
}

void white_surface_detected_handler(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_FALL)
    {

        if (elapsed_seconds < THRESHOLD)
        {
            appendValue(&array, &size, &capacity, 1);
        }
        else if (elapsed_seconds < 5000 && elapsed_seconds > THRESHOLD)
        {
            appendValue(&array, &size, &capacity, 111);
        }

        white_surface_detected = true;
        white_surface_start_time = get_absolute_time();
        elapsed_seconds = absolute_time_diff_us(black_surface_start_time, white_surface_start_time) / 1000;
        printf("BLACK -- Time elapsed: %d seconds\n", elapsed_seconds);
    }
    else if (events & GPIO_IRQ_EDGE_RISE)
    {

        if (elapsed_seconds < THRESHOLD)
        {
            appendValue(&array, &size, &capacity, 2);
        }
        else if (elapsed_seconds < 5000 && elapsed_seconds > THRESHOLD)
        {
            appendValue(&array, &size, &capacity, 222);
        }

        white_surface_detected = false;
        black_surface_start_time = get_absolute_time();
        elapsed_seconds = absolute_time_diff_us(white_surface_start_time, black_surface_start_time) / 1000;
        printf("WHITE -- Time elapsed: %d seconds\n", elapsed_seconds);
    }
}

int main()
{
    stdio_init_all();
    adc_init();

    // Set the ADC channel for the IR sensor pin
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0); // Use ADC channel 0 (pin 26)

    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &white_surface_detected_handler);

    // Define the list of Code 39 characters and their binary representations
    struct Code39Character characters[] = {
        {'A', "12221211121112121112121222121112122212111211121"},
                
        {'B', "12221211121112121211121222121112122212111211121"},
        {'C', "12221211121112121112111212221212122212111211121"},
        {'D', "12221211121112121212111222121112122212111211121"},
        {'E', "12221211121112121112121112221212122212111211121"},
        {'F', "12221211121112121211121112221212122212111211121"},
        {'G', "12221211121112121212122211121112122212111211121"},
        {'H', "12221211121112121112121222111212122212111211121"},
        {'I', "12221211121112121211121222111212122212111211121"},
        {'J', "12221211121112121212111222111212122212111211121"},
        {'K', "12221211121112121112121212221112122212111211121"},
        {'L', "12221211121112121211121212221112122212111211121"},
        {'M', "12221211121112121112111212122212122212111211121"},
        {'N', "12221211121112121212111212221112122212111211121"},
        {'O', "12221211121112121112121112122212122212111211121"},
        {'P', "12221211121112121211121112122212122212111211121"},
        {'Q', "12221211121112121212121112221112122212111211121"},
        {'R', "12221211121112121112121211122212122212111211121"},
        {'S', "12221211121112121211121211122212122212111211121"},
        {'T', "12221211121112121212111211122212122212111211121"},
        {'U', "12221211121112121112221212121112122212111211121"},
        {'V', "12221211121112121222111212121112122212111211121"},
        {'W', "12221211121112121112221112121212122212111211121"},
        {'X', "12221211121112121222121112121112122212111211121"},
        {'Y', "12221211121112121112221211121212122212111211121"},
        {'Z', "12221211121112121222111211121212122212111211121"}};

    int numCharacters = sizeof(characters) / sizeof(characters[0]);

    initializeDynamicArray(&array, &size, &capacity);

    while (1)
    {

        // Print all values in the dynamic array
        printf("Dynamic Array Values: ");
        for (int i = 0; i < size; i++)
        {
            printf("%d ", array[i]);
        }
        printf("\n");

        // Concatenate the dynamic array into a string
        char *barcodeString = concatenateArrayToString(array, size);
        printf("Barcode String: %s\n", barcodeString);

        char character = findCode39Character(barcodeString, characters, numCharacters);
        printf("Code 39 Character: %c\n", character);

        // Add a delay to avoid rapid readings (adjust as needed)
        if (size >= 40)
        {
            // Free the dynamic array
            freeDynamicArray(array);
            break; // Exit the loop when the array size reaches 40
        }
        sleep_ms(50); // Sleep for 100 milliseconds
    }

    return 0;
}