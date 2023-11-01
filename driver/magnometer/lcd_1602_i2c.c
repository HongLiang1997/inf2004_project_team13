#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define ACCEL_CTRL_REG1_A 0x20      // Control register for accelerometer
#define MAGNETOMETER_ADDR 0x1E      // Magnetometer I2C Address
#define ACCELEROMETER_ADDR 0x19     // Accelerometer I2C Address
#define MAGNETOMETER_MR_REG_M 0x02  // Mode register for magnetometer
#define MAGNETOMETER_OUT_X_H_M 0x03 // Data output X high byte register for magnetometer

int16_t ax_offset = 0, ay_offset = 0, az_offset = 0; // Storing offset for accelerometer data calibration

void init_i2c()
{
    i2c_init(I2C_PORT, 400000); // Initialize I2C with 400KHz
    gpio_set_function(4, GPIO_FUNC_I2C); // Initialise GP4 pin
    gpio_set_function(5, GPIO_FUNC_I2C); // Initialise GP5 pin
    gpio_pull_up(4);
    gpio_pull_up(5);
}

// Function to read data from accelerometer
bool read_accel_data(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];

    // Set the register to start reading from
    uint8_t start_reg = 0x28 | 0x80; // 0x80 enables auto-increment
    if (i2c_write_blocking(I2C_PORT, ACCELEROMETER_ADDR, &start_reg, 1, true) != PICO_ERROR_GENERIC)
    {

        // Read 6 bytes of data
        if (i2c_read_blocking(I2C_PORT, ACCELEROMETER_ADDR, buffer, 6, false) == 6)
        {
            *x = (int16_t)(buffer[0] | (buffer[1] << 8));
            *y = (int16_t)(buffer[2] | (buffer[3] << 8));
            *z = (int16_t)(buffer[4] | (buffer[5] << 8));
            return true;
        }
    }
    return false;
}

// Function to read data from magnometer
bool read_mag_data(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];
    uint8_t start_reg = MAGNETOMETER_OUT_X_H_M;

    if (i2c_write_blocking(I2C_PORT, MAGNETOMETER_ADDR, &start_reg, 1, true) != PICO_ERROR_GENERIC)
    {
        if (i2c_read_blocking(I2C_PORT, MAGNETOMETER_ADDR, buffer, 6, false) == 6)
        {
            *x = (int16_t)(buffer[0] << 8 | buffer[1]);
            *z = (int16_t)(buffer[2] << 8 | buffer[3]);
            *y = (int16_t)(buffer[4] << 8 | buffer[5]);
            return true;
        }
    }
    return false;
}

// Calibrate magnometer by calculating offsets and averaging them
void calibrate_mag()
{
    int16_t x, y, z;
    int16_t x_offset = 0;
    int16_t y_offset = 0;
    int16_t z_offset = 0;
    for (int i = 0; i < 100; i++)
    {
        read_mag_data(&x, &y, &z);
        x_offset += x;
        y_offset += y;
        z_offset += z;
        sleep_ms(100);
    }
    x_offset /= 100;
    y_offset /= 100;
    z_offset /= 100;
    printf("Magnetometer offsets - X: %d, Y: %d, Z: %d\n", x_offset, y_offset, z_offset);
}

// Calibrate accelerometer by calculating offsets and averaging them
void calibrate_accelerometer()
{
    int16_t x, y, z;
    int sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < 100; i++)
    { // Taking 100 samples for calibration
        if (read_accel_data(&x, &y, &z))
        {
            sum_x += x;
            sum_y += y;
            sum_z += z;
        }
        sleep_ms(100);
    }
    ax_offset = sum_x / 100;
    ay_offset = sum_y / 100;
    az_offset = (sum_z / 100) - 1000; // Compensating for the effect of gravity
}

int main()
{
    stdio_init_all();

    // Initialize USB
    stdio_usb_init();

    // Initialize I2C
    init_i2c();

    // Initialize the accelerometer (set it to normal mode and enable all axes, 100Hz)
    uint8_t ctrl1_reg = 0x57;
    if (i2c_write_blocking(I2C_PORT, ACCELEROMETER_ADDR, (uint8_t[]){ACCEL_CTRL_REG1_A, ctrl1_reg}, 2, false) != PICO_ERROR_GENERIC)
    {

        // Initialize the magnetometer (set it to continuous conversion mode)
        uint8_t mr_reg = 0x00;
        if (i2c_write_blocking(I2C_PORT, MAGNETOMETER_ADDR, (uint8_t[]){MAGNETOMETER_MR_REG_M, mr_reg}, 2, false) != PICO_ERROR_GENERIC)
        {

            // Calibrate the magnetometer
            calibrate_mag();

            // Calibrate the accelerometer
            calibrate_accelerometer();

            while (true)
            {
                int16_t x, y, z;

                // Read accelerometer data
                if (read_accel_data(&x, &y, &z))
                {

                    // Compensate for the offset values obtained during calibration
                    x -= ax_offset;
                    y -= ay_offset;
                    z -= az_offset;

// Print the calibrated data
                    printf("Accelerometer - X: %d, Y: %d, Z: %d\n", x, y, z);
                }
                else
                {
                    printf("Error reading accelerometer data\n");
                }

                // Read magnetometer data
                if (read_mag_data(&x, &y, &z))
                {
                    printf("Magnetometer - X: %d, Y: %d, Z: %d\n", x, y, z);
                }
                else
                {
                    printf("Error reading magnetometer data\n");
                }

                sleep_ms(1000);
            }
        }
        else
        {
            printf("Error initializing magnetometer\n");
        }
    }
    else
    {
        printf("Error initializing accelerometer\n");
    }

    return 0;
}