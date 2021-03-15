#include "max30208.h"

//#define logging 0

static int write_bytes(struct device *i2c_dev, u8_t addr, u8_t *data, u32_t num_bytes, u16_t slaveAddr)
{
    u8_t ret = 0x00;
#ifdef logging
    printk("Writing register - ox%x . Bytes =%d\n", addr, num_bytes);
#endif
    ret = i2c_reg_write_byte(i2c_dev, slaveAddr, addr, data);
#ifdef logging
    if (ret == MAX_OK)
        printk("Wrote Data\n");
    else
        printk("Failed to write\n");
#endif
    return ret;
}

static int read_bytes(struct device *i2c_dev, u8_t addr, u8_t *data, u32_t num_bytes, u16_t slaveAddr)
{
#ifdef logging
    printk("Reading register ox%x . Bytes =%d\n", addr, num_bytes);
#endif
    u8_t ret = i2c_reg_read_byte(i2c_dev, slaveAddr, addr, data);
#ifdef logging
    if (ret == MAX_OK)
        printk("Data read. Value = 0x%x\n", *(data));
#endif
    return ret;
}


/* MAX30102 i2c write function */
static uint16_t max30208_i2c_write(uint8_t reg, uint8_t *data, uint16_t len)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    return write_bytes(i2c_dev, reg, data, len, MAX30208_ADDRESS);
}

/* MAX30102 i2c read function */
static uint16_t max30208_i2c_read(u8_t reg, u8_t *data, u16_t len)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    return read_bytes(i2c_dev, reg, data, len, MAX30208_ADDRESS);
}

u16_t init_max30208()
{
    u8_t chipId;
    u16_t rslt = 0x00;
    rslt = max30208_i2c_read(MAX30208_PART_ID, &chipId, 1);
    if (rslt == MAX_OK)
    {
        if (chipId == I_AM_MAX30208)
        {
            printk("Communication with MAX30208 : OK\n");
        }
        else
        {
            printk("Communication with MAX30208 : FAILED!!!\n");
        }
    }
    else
    {
        printk("Communication with MAX30208 : FAILED!!!\n I2C error %d\n", rslt);
    }
    return rslt;
}

u16_t configure_max30208()
{
    u16_t rslt = MAX_OK;
    rslt |= max30208_i2c_write(MAX30208_INT_ENABLE, 0x00, 1);
    rslt |= max30208_i2c_write(MAX30208_FIFO_W_POINTER, 0x00, 1);
    rslt |= max30208_i2c_write(MAX30208_FIFO_R_POINTER, 0x00, 1);
    rslt |= max30208_i2c_write(MAX30208_OVR_COUNTER, 0x00, 1);
    rslt |= max30208_i2c_write(MAX30208_FIFO_CONFIG_1, 0x0F, 1);
    rslt |= max30208_i2c_write(MAX30208_FIFO_CONFIG_2, 0x1A, 1);
    rslt |= max30208_i2c_write(MAX30208_TEMP_SETUP, 0xC1, 1);
    if (rslt == MAX_OK)
        printk("CONFIGURING MAX30208 : OK\n");
    else
        printk("CONFIGURING MAX30208 : FAILED!!!\n");

    return (rslt);
}

u16_t max30208_read_fifo(int *data)
{
    u16_t rslt = MAX_OK;
    *data = 0;
    u16_t temp_dat[2] = {0, 0};
    int samples = 1, sum = 0;
    rslt |= max30208_i2c_write(MAX30208_TEMP_SETUP, 0xC1, 1);
    for (int j = 0; j < samples; j++)
    {
        for (int i = 0; i < 2; i++)
        {
            u8_t temp;
            max30208_i2c_read(MAX30208_FIFO_DATA_REG, &temp, 1);
            temp_dat[i] = temp;
        }
        sum = sum + (temp_dat[0] << 8) + temp_dat[1];
    }
    *data = sum / samples;
    u8_t temp;
    max30208_i2c_read(MAX30208_FIFO_R_POINTER, &temp, 1);
    //rslt |= max30208_i2c_write(MAX30208_FIFO_CONFIG_2, 0x1A, 1);
    *data = (temp_dat[0] << 8) + temp_dat[1];
    if (rslt != MAX_OK)
        // printk("READING FROM MAX30208 FIFO : FAILED\n");
    return rslt;
}

u16_t max30208_read_temp(float *value)
{
    int temp;
    u16_t rslt = 0x00;
    rslt |= max30208_read_fifo(&temp);
    if (temp & 0x8000)
    {
        temp = ~(temp - 1);
    }
    *value = (float)temp * 0.005;
    return rslt;
}



/* MPU6050 i2c write function */
static uint16_t mpu6050_i2c_write(uint8_t reg, uint8_t *data, uint16_t len)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    
    int ret;
    ret = write_bytes(i2c_dev, reg, data, 1,  MPU6050_ADDRESS);
    #ifdef logging
	if (ret) {
		printk("Error writing to MPU! error code (%d)\n", ret);
		return;
	} else {
		printk("Wrote 0xAE to address 0x00.\n");
	}
    #endif
    return ret;
}

/* MPU6050 i2c read function */
static uint16_t mpu6050_i2c_read(u8_t reg, u8_t *data, u16_t len)
{
    struct device *i2c_dev;
    int ret;
    i2c_dev = device_get_binding(I2C_DEV);
    ret =  read_bytes(i2c_dev, reg, data, len, MPU6050_ADDRESS);
    #ifdef log
    if (ret) {
		printk("Error reading from MPU6050! error code (%d)\n", ret);
		return;
	} else {
		printk("Read 0x%X from address %X.\n", *data, reg);
	}
    #endif
    return ret;
}

/*** Config MPU6050 ***/
void configure_mpu6050()
{
    int rslt;
	mpu6050_i2c_write(0x6B, 0x00, 1);	//Setting power
	mpu6050_i2c_write(0x1B, 0x00, 1);	//Setting Gyro
	mpu6050_i2c_write(0x38, 0x00, 1);	//Setting Accl
    
    mpu6050_i2c_read(0x75, &rslt, 1);   //Reading Who_AM_I register of MPU6050 
    if(rslt == 0x68)
        printk("CONFIGURING MPU6050 : OK\n");
    else
      printk("CONFIGURING MPU6050 : FAILED !!!\n");  
}

u16_t mpu6050_read_Acc(uint16_t *value)
{
    return mpu6050_i2c_read(0x3B, value, 2);
}