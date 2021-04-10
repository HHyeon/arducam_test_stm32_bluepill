
#ifndef ____ARDUCAM_2MP_MINI_DRIVER_____
#define ____ARDUCAM_2MP_MINI_DRIVER_____

#include "main.h"

void cam_spi_write_reg(uint8_t addr, uint8_t reg);
uint8_t cam_spi_read_reg(uint8_t addr);

void cam_i2c_write_regs(const struct sensor_reg reglist[]);
void cam_i2c_write_reg(uint8_t addr, uint8_t reg);
uint8_t cam_i2c_read_reg(uint8_t addr);

#endif