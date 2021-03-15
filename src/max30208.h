#pragma once
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>

#define I2C_DEV DT_LABEL(DT_ALIAS(i2c_0))


/******************************************************************************/
/*********** TEMPERATURE SENSOR REGISTER MAPPING  **************/
/******************************************************************************/
 
// status registers
#define MAX30208_INT_STATUS                 0x00
#define MAX30208_INT_ENABLE                 0x01
 
// FIFO registers
#define MAX30208_FIFO_W_POINTER             0x04
#define MAX30208_FIFO_R_POINTER             0x05
#define MAX30208_OVR_COUNTER                0x06
#define MAX30208_FIFO_DATA_COUNTER          0x07
#define MAX30208_FIFO_DATA_REG              0x08
 
// configuration registers
#define MAX30208_FIFO_CONFIG_1              0x09
#define MAX30208_FIFO_CONFIG_2              0x0A
#define MAX30208_SYSTEM_CONTROL             0x0C
#define MAX30208_TEMP_SETUP                 0x14
 

 
// PART ID registers
#define MAX30208_PART_ID                    0xFF
 
#define I_AM_MAX30208                       0x30

#define MAX30208_ADDRESS                    0x50
#define MPU6050_ADDRESS                     0x68   

#define MAX_OK                              0x00


u16_t init_max30208();
u16_t configure_max30208();

void configure_mpu6050();
static uint16_t mpu6050_i2c_write(uint8_t reg, uint8_t *data, uint16_t len);
static uint16_t mpu6050_i2c_read(u8_t reg, u8_t *data, u16_t len);
u16_t mpu6050_read_Acc(uint16_t *value);

static uint16_t max30208_i2c_write(uint8_t reg, uint8_t *data, uint16_t len);
static uint16_t max30208_i2c_read(u8_t reg, u8_t *data, u16_t len);
u16_t max30208_read_fifo(int *temp_data);
u16_t max30208_read_temp(float *value);