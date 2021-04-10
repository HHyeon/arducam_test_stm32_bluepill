

#include "arducam_2mp_mini_driver.h"
#include "stdio.h"

#define I2C_DEV_ADDR 0x60

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c2;

void cam_spi_write_reg(uint8_t addr, uint8_t reg)
{
  uint8_t pdatatmp[2];
  pdatatmp[0] = addr | 0x80;
  pdatatmp[1] = reg;
  
  CAM_CS_BEGIN;
  HAL_SPI_Transmit(&hspi1, pdatatmp, 2, 0xFFFF);
  CAM_CS_END;
}

uint8_t cam_spi_read_reg(uint8_t addr)
{
  uint8_t pdatatmp[2];
  pdatatmp[0] = addr & 0x7f;
  CAM_CS_BEGIN;
  HAL_SPI_Transmit(&hspi1, &pdatatmp[0], 1, 0xFFFF);
  pdatatmp[0] = 0x00;
  HAL_SPI_TransmitReceive(&hspi1, &pdatatmp[0], &pdatatmp[1], 1, 0xFFFF);
  CAM_CS_END;
  
  return pdatatmp[1];
}

void cam_i2c_write_reg(uint8_t addr, uint8_t reg)
{
  if( HAL_I2C_Mem_Write(&hi2c2, I2C_DEV_ADDR, addr, 1, &reg, 1, 0xFF) != HAL_OK)
  {
    printf("HAL_I2C_Mem_Write Error\n");
    Error_Handler();
  }
}

uint8_t cam_i2c_read_reg(uint8_t addr)
{
  uint8_t reg;
  if( HAL_I2C_Mem_Read(&hi2c2, I2C_DEV_ADDR, addr, 1, &reg, 1, 0xFF) != HAL_OK)
  {
    printf("HAL_I2C_Mem_Write Error\n");
    Error_Handler();
  }
  return reg;
}


void cam_i2c_write_regs(const struct sensor_reg reglist[])
{
//  if( HAL_I2C_Mem_Write(&hi2c2, I2C_DEV_ADDR, addr, 1, &reg, 1, 0xFF) != HAL_OK)
//  {
//    printf("HAL_I2C_Mem_Write Error\n");
//    Error_Handler();
//  }
  
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = next->reg;
    reg_val = next->val;
//    printf("0x%02x - 0x%02x\n",reg_addr, reg_val);
    cam_i2c_write_reg(reg_addr, reg_val);
    next++;
  }
  
}
