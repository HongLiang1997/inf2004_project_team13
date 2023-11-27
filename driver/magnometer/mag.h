#ifndef MAG_H
#define MAG_H

#include <stdint.h>

void mag_init();
void write_byte(uint8_t address, uint8_t reg, uint8_t value);
uint8_t read_byte(uint8_t address, uint8_t reg);
int16_t read_data(uint8_t address, uint8_t reg_high, uint8_t reg_low);
void initialize_lsm303();
float getPitch(int16_t Ax, int16_t Ay, int16_t Az);
float getRoll(int16_t Ax, int16_t Az);
float getYaw(int16_t Hx, int16_t Hy, float pitch, float roll);
float calculate_heading(int16_t Hx, int16_t Hy);
float mag_get_x_angle();
float mag_get_y_angle();
float mag_get_heading();
float mag_measurement();
float mag_measurement();

#endif
