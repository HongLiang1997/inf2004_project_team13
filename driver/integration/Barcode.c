#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <string.h>
#include "hardware/pwm.h"

// Define
// Barcode define
#define MAX_WHITE_BARS 14
#define MAX_BLACK_BARS 15
#define MAX_ENCODED_VALUES 11
// Motor control
#define LEFT_WHEEL 15
#define RIGHT_WHEEL 14
#define RIGHT_WHEEL_FORWARD 20
#define RIGHT_WHEEL_BACKWARD 21
#define LEFT_WHEEL_FORWARD 18
#define LEFT_WHEEL_BACKWARD 19
#define CLK_DIV 100
#define WRAP 12500

// Variable
//  Motor control variables
float left_new = 0;
float right_new = 0;
uint slice_num_left;
uint slice_num_right;

// Barcode variables
static int barCount_w = -1;
static int barCount_b = -1;
static uint64_t barTime_w[MAX_WHITE_BARS];
static uint64_t barTime_b[MAX_BLACK_BARS];
static uint64_t startIntervalTime = 0;
static uint64_t interval = 0;

typedef struct
{
    uint64_t startTime;
    int category;
} BarInfo;

typedef struct
{
    char letter;
    int arrayMap[MAX_ENCODED_VALUES];
} ArrayMapEntry;

ArrayMapEntry arrayMapDictionary[] = {
    {'A', {3, 0, 3, 1, 3, 1, 2, 1, 3, 0, 3}},
    {'B', {3, 1, 3, 0, 3, 1, 2, 1, 3, 0, 3}},
    {'C', {3, 0, 3, 0, 3, 1, 2, 1, 3, 1, 3}},
    {'D', {3, 1, 3, 1, 3, 0, 2, 1, 3, 0, 3}},
    {'E', {3, 0, 3, 1, 3, 0, 2, 1, 3, 1, 3}},
    {'F', {3, 1, 3, 0, 3, 0, 2, 1, 3, 1, 3}},
    {'G', {3, 1, 3, 1, 3, 1, 2, 0, 3, 0, 3}},
    {'H', {3, 0, 3, 1, 3, 1, 2, 0, 3, 1, 3}},
    {'I', {3, 1, 3, 0, 3, 1, 2, 0, 3, 1, 3}},
    {'J', {3, 1, 3, 1, 3, 0, 2, 0, 3, 1, 3}},
    {'K', {3, 0, 3, 1, 3, 1, 3, 1, 2, 0, 3}},
    {'L', {3, 1, 3, 0, 3, 1, 3, 1, 2, 0, 3}},
    {'M', {3, 0, 3, 0, 3, 1, 3, 1, 2, 1, 3}},
    {'N', {3, 1, 3, 1, 3, 0, 3, 1, 2, 0, 3}},
    {'O', {3, 0, 3, 1, 3, 0, 3, 1, 2, 1, 3}},
    {'P', {3, 1, 3, 0, 3, 0, 3, 1, 2, 1, 3}},
    {'Q', {3, 1, 3, 1, 3, 1, 3, 0, 2, 0, 3}},
    {'R', {3, 0, 3, 1, 3, 1, 3, 0, 2, 1, 3}},
    {'S', {3, 1, 3, 0, 3, 1, 3, 0, 2, 1, 3}},
    {'T', {3, 1, 3, 1, 3, 0, 3, 0, 2, 1, 3}},
    {'U', {3, 0, 2, 1, 3, 1, 3, 1, 3, 0, 3}},
    {'V', {3, 1, 2, 0, 3, 1, 3, 1, 3, 0, 3}},
    {'W', {3, 0, 2, 0, 3, 1, 3, 1, 3, 1, 3}},
    {'X', {3, 1, 2, 1, 3, 0, 3, 1, 3, 0, 3}},
    {'Y', {3, 0, 2, 1, 3, 0, 3, 1, 3, 1, 3}},
    {'Z', {3, 1, 2, 0, 3, 0, 3, 1, 3, 1, 3}}};

// Functions
//  Motor control functions
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

// Set left wheel right
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

// Set right wheel right
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
    left_new += 750;
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, left_new);
}
void decrease_left_speed(void)
{
    left_new -= 750;
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, left_new);
}

// Adjust right speed
void increase_right_speed(void)
{
    right_new += 750;
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, right_new);
}
void decrease_right_speed(void)
{
    right_new -= 750;
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

// Barcode Functions
char decodeBarInfo(int encodedString[MAX_ENCODED_VALUES])
{
    // Compare encoded string with arrayMapDictionary
    for (int i = 0; i < sizeof(arrayMapDictionary) / sizeof(arrayMapDictionary[0]); i++)
    {
        int matched = 1;
        for (int j = 0; j < MAX_ENCODED_VALUES; j++)
        {
            if (arrayMapDictionary[i].arrayMap[j] != encodedString[j])
            {
                matched = 0;
                break;
            }
        }
        if (matched)
        {
            return arrayMapDictionary[i].letter;
        }
    }

    // No match found
    return '\0';
}


int main(void)
{

    stdio_init_all();
    gpio_init(22); // Initialise pin 22 for barcode reader
    gpio_set_dir(22, GPIO_IN);

    init_motor(NULL);      // init motor
    set_left_speed(0.39);  // set left wheel speed
    set_right_speed(0.36); // set right wheel speed

    sleep_ms(2000); // sleep to allow proper initialisiation

    int previousState = gpio_get(22); // Get previous state of barcode sensor
    while (1)
    {
        move_forward(NULL); // motor move forward
        int currentState = gpio_get(22);

        if (currentState != previousState)
        {
            if (currentState == 0)
            {
                barCount_w++;
                startIntervalTime = to_ms_since_boot(get_absolute_time());
                
            }
            else
            {
                barCount_b++;
                startIntervalTime = to_ms_since_boot(get_absolute_time());
                
            }
        }
        else if (currentState == 0 && barCount_w >= 0)
        {
            // Update interval of white bar
            interval = to_ms_since_boot(get_absolute_time()) - startIntervalTime;
            barTime_w[barCount_w] = interval;
        }
        else if (currentState == 1 && barCount_b >= 0)
        {
            // Update interval of black bar
            interval = to_ms_since_boot(get_absolute_time()) - startIntervalTime;
            barTime_b[barCount_b] = interval;
            if (barCount_b == MAX_BLACK_BARS - 1)
            {
                if (currentState == 0)
                {
                    interval = to_ms_since_boot(get_absolute_time()) - startIntervalTime;
                    barTime_b[barCount_b] = interval;
                }
            }
        }
        if (barCount_w == MAX_WHITE_BARS)
        {
            break;
        }
        previousState = currentState;
    }

    // Calculate average of white bar length
    uint64_t total_BarTime_w = 0;
    for (int i = 0; i < MAX_WHITE_BARS; i++)
    {
        total_BarTime_w += barTime_w[i];
    }
    uint64_t averageWhiteBarLength = MAX_WHITE_BARS > 0 ? total_BarTime_w / MAX_WHITE_BARS : 0;

    // Determine each white bar width if is thin or thick based on the average of all detected Bars
    
    for (int i = 0; i <= MAX_WHITE_BARS; i++)
    {
        uint64_t barTime = barTime_w[i];
       
    }

    // Calculate average of black bar length
    uint64_t total_BarTime_b = 0;
    for (int i = 0; i < MAX_BLACK_BARS; i++)
    {
        total_BarTime_b += barTime_b[i];
    }
    uint64_t averageBlackBarLength = MAX_BLACK_BARS > 0 ? total_BarTime_b / MAX_BLACK_BARS : 0;

    // Determine each black bar width if is thin or thick based on the average of all detected Bars

    for (int i = 0; i < MAX_BLACK_BARS; i++)
    {
        uint64_t barTime = barTime_b[i];
    }

    // Store the start time and category(black&white) of each bar
    BarInfo barList[MAX_WHITE_BARS + MAX_BLACK_BARS];
    int barCount = 0;
    int whiteIndex = 0;
    int blackIndex = 0;

    for (int i = 0; i < MAX_WHITE_BARS + MAX_BLACK_BARS; i++)
    {
        if (i % 2 == 0)
        {
            // Add black bar
            if (blackIndex < MAX_BLACK_BARS)
            {
                barList[barCount].startTime = barTime_b[blackIndex];
                barList[barCount].category = (barTime_b[blackIndex] > averageBlackBarLength) ? 0 : 1;
                blackIndex++;
                barCount++;
            }
        }
        else
        {
            // Add white bar
            if (whiteIndex < MAX_WHITE_BARS)
            {
                barList[barCount].startTime = barTime_w[whiteIndex];
                barList[barCount].category = (barTime_w[whiteIndex] > averageWhiteBarLength) ? 2 : 3;
                whiteIndex++;
                barCount++;
            }
        }
    }
    // Print the stored bar in Array
    printf("Stored Bar Information:\n");
    for (int i = 0; i < barCount; i++)
    {
        printf("%d ", barList[i].category);
    }

    int encodedString[MAX_ENCODED_VALUES];
    for (int i = 0; i < MAX_ENCODED_VALUES; i++)
    {
        encodedString[i] = barList[i + 9].category;
    }

    printf("\nExtracted String: ");
    for (int i = 0; i < MAX_ENCODED_VALUES; i++)
    {
        printf("%d", encodedString[i]);
    }
    printf("\n");

    barCount_w = 0;
    barCount_b = 0;
    startIntervalTime = 0;
    interval = 0;

    char decodedLetter = decodeBarInfo(encodedString);

    if (decodedLetter != '\0')
    {
        printf("Decoded Letter: %c\n", decodedLetter);
    }
    else
    {
        printf("No letter matched the encoded string.\n");
    }

    return 0;
}
