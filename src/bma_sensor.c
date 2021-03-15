#include "bma_sensor.h"
//#define logging 0

struct bma4_dev accel;
struct bma4_accel_config accel_conf;
struct bma4_dev bma456_device_global;
float devRange;

static int write_bytes(struct device *i2c_dev, u8_t addr, u8_t *data, u32_t num_bytes, u16_t slaveAddr)
{
    u8_t ret = 0x00;
#ifdef logging
    printk("Writing register - ox%x . Bytes =%d\n", addr, num_bytes);
#endif
    u8_t buffer[2];
    buffer[0] = &addr;
    buffer[1] = data;
    ret = i2c_burst_write(i2c_dev, slaveAddr, addr, data, num_bytes);
#ifdef logging
    if (ret == BMA4_OK)
        printk("Wrote Data\n");
    else
        printk("Failed to write\n");
#endif
    return ret;
}

static int read_bytes(struct device *i2c_dev, u8_t addr, u8_t *data, u32_t num_bytes, u16_t slaveAddr)
{
#ifdef logging
    printk("Reading register -ox%x . Bytes =%d\n", addr, num_bytes);
#endif
    u8_t ret = i2c_burst_read(i2c_dev, slaveAddr, addr, data, num_bytes);
#ifdef logging
    if (ret == 0x00)
        printk("Data read. Value = 0x%x\n", *(data));
#endif
    return ret;
}

/* BMA i2c write function */
static uint16_t bma_i2c_write(uint8_t reg, uint8_t *data, uint16_t len, void *intf_ptr)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    return write_bytes(i2c_dev, reg, data, len, BMA4_I2C_ADDR_SECONDARY);
}

/* BMA i2c read function */
static uint16_t bma_i2c_read(u8_t reg, u8_t *data, u16_t len, void *intf_ptr)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    return read_bytes(i2c_dev, reg, data, len, BMA4_I2C_ADDR_SECONDARY);
}

/* BMA delay function */
static void bma_delay_us(u32_t ms)
{
    k_busy_wait(ms);
}

/*Initialise BMA sensor */
void initialize_bma()
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    if (!i2c_dev)
    {
        printk("I2C: Device driver not found.\n");
        return;
    }
    uint16_t rslt = BMA4_OK;
    struct bma4_dev bma456_device;
    ////////// Device Settings /////////////
    static uint8_t dev_addr;
    dev_addr = BMA4_I2C_ADDR_SECONDARY;
    bma456_device.bus_read = bma_i2c_read;
    bma456_device.bus_write = bma_i2c_write;
    bma456_device.delay_us = bma_delay_us;
    bma456_device.read_write_len = 8;
    bma456_device.resolution = 12;
    bma456_device.feature_len = BMA456_FEATURE_SIZE;
    bma456_device.intf_ptr = &dev_addr;
    /////////////////////////////////////////
    rslt |= bma456_init(&bma456_device);
    if (rslt == BMA4_OK)
        printk("Communication with BMA : OK\n");
    else
        printk("Communication with BMA : FAILED!!! \n");

    // Soft Reset //
    bma4_set_command_register(0xB6, &bma456_device);
    ////////////////
    k_msleep(10);
    rslt = BMA4_OK;
    rslt |= bma456_write_config_file(&bma456_device);
    if (rslt == BMA4_OK)
        printk("Writing config file : OK\n");
    else
        printk("Writing config file : FAILED!!!\n");
    bma456_device_global = bma456_device;
}

u16_t init_accel()
{
    uint16_t rslt = BMA4_OK;
    struct bma4_dev bma456_device = bma456_device_global;
    /* Enable the accelerometer */
    printk("Enabling accelerometer\n");
    rslt = bma4_set_accel_enable(0x01, &bma456_device);
    if (rslt == BMA4_OK)
        printk("Enable accelerometer : OK\n");
    else
        printk("Enable accelerometer : Failed!!!\n");

    /* Declare an accelerometer configuration structure */
    //struct bma4_accel_config accel_conf;

    /* Assign the desired settings */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    accel_conf.range = BMA4_ACCEL_RANGE_2G;
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /* Set the configuration */
    rslt |= bma4_set_accel_config(&accel_conf, &bma456_device);
    if (rslt == BMA4_OK)
        printk("Accelerometer configuration : OK\n");
    else
        printk("Accelerometer configuration : FAILED!!!\n");
    return rslt;
}

struct BMA456_ACCEL_DATA read_accel()
{
    struct BMA456_ACCEL_DATA data;
    uint16_t rslt = BMA4_OK;
    struct bma4_accel sens_data;
    struct bma4_dev bma456_device = bma456_device_global;
    /* Read the sensor data into the sensor data instance */
    rslt |= bma4_read_accel_xyz(&sens_data, &bma456_device);

    /* Exit the program in case of a failure */
    if (rslt != BMA4_OK)
    {
        printk("Reading data from accelerometer : FAILED!!!");
        data.datax = 0;
        data.datay = 0;
        data.dataz = 0;
        data.result = rslt;
        return data;
    }

    /* Save the data */
    data.datax = sens_data.x;
    data.datay = sens_data.y;
    data.dataz = sens_data.z;
    data.result = rslt;
    bma456_device_global = bma456_device;
    return data;
}

u16_t stepCounterEnable()
{
    struct bma4_accel sens_data;
    struct bma4_dev bma456_device = bma456_device_global;
    uint16_t rslt = 0;
    // /* Soft-reset */
    rslt |= bma4_set_command_register(0xB6, &bma456_device);
    if (rslt == BMA4_OK)
        printk("Soft reset before enabling step counter : OK\n");
    else
        printk("Soft reset before enabling step counter : FAILED!!!\n");
    k_msleep(100);

    /* Initialize BMA456 */
    rslt |= bma456_init(&bma456_device);
    if (rslt != BMA4_OK)
    {
        printk("Testing communication with BMA : FAILED!!!\n");
        return rslt;
    }
    else
    {
        printk("Testing communication with BMA : OK\n");
    }

    /* Enable accelerometer */
    rslt = bma4_set_accel_enable(0x01, &bma456_device);
    if (rslt == BMA4_OK)
        printk("Enabling accelerometer : OK\n");
    else
    {
        printk("Enabling accelerometer : FAILED!!!\n");
        return rslt;
    }
    /* Load configuration file */
    rslt = bma456_write_config_file(&bma456_device);
    if (rslt != BMA4_OK)
    {
        printf("Load configuration fail\r\n");
        return rslt;
    }
    else
    {
        printf("Load configuration successful\r\n");
    }

    /* Enable step counter */
    rslt = bma456_feature_enable(BMA456_STEP_CNTR, 1, &bma456_device);
    if (rslt != BMA4_OK)
    {
        printf("Step counter not enabled\r\n");
        return rslt;
    }
    else
    {
        printf("Step counter enabled\r\n");
    }

    /* Map step counter interrupt */
    rslt = bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_STEP_CNTR_INT, 1, &bma456_device);
    if (rslt != BMA4_OK)
    {
        printf("Error code: %d\n", rslt);
        return rslt;
    }

    /* Set water-mark level 1 */
    rslt = bma456_step_counter_set_watermark(1, &bma456_device);
    if (rslt != BMA4_OK)
    {
        printf("Error code: %d\n", rslt);
        return rslt;
    }

    // /* Load configuration file */
    // rslt = bma456_reset_step_counter(&bma456_device);
    // if (rslt == BMA4_OK)
    //     printk("Resetting stepcounter : OK\n");
    // else
    // {
    //     printk("Resetting stepcounter : FAILED!!!\n");
    //     return rslt;
    // }
    // rslt = bma456_write_config_file(&bma456_device);
    // if (rslt != BMA4_OK)
    // {
    //     printk("Load configuration file : FAILED!!!\n");
    //     return rslt;
    // }
    // else
    // {
    //     printk("Load configuration file: OK\n");
    // }

    // /* Enable step counter */
    // rslt = bma456_feature_enable(BMA456_STEP_CNTR, 1, &bma456_device);
    // if (rslt != BMA4_OK)
    // {
    //     printk("Enabling step counter : FAILED!!!\n");
    //     return rslt;
    // }
    // else
    // {
    //     printk("Enabling step counter : OK\n");
    // }

    // /* Map step counter interrupt */
    // rslt = bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_STEP_CNTR_INT, 1, &bma456_device);
    // if (rslt != BMA4_OK)
    // {
    //     printk("Mapping Step counter interrupt : FAILED!!!\n Error code: %d\n", rslt);
    //     return rslt;
    // }
    // else
    //     printk("Mapping Step counter interrupt : OK\n");

    // /* Set water-mark level 1 */
    // rslt = bma456_step_counter_set_watermark(1, &bma456_device);
    // if (rslt != BMA4_OK)
    // {
    //     printk("Setting watermark : FAILED!!!\n Error code: %d\n", rslt);
    //     return rslt;
    // }
    // else
    //     printk("Setting watermark : OK\n");
    bma456_device_global = bma456_device;
    printk("Step counter init : OK\n");
    return rslt;
}

u32_t getStepCounterOutput()
{
    struct bma4_dev bma456_device = bma456_device_global;
    uint16_t rslt = 0;
    uint32_t step_out = 0;
    rslt = bma456_step_counter_output(&step_out, &bma456_device);
    if (rslt == BMA4_OK)
    {
        //printk("\nSteps counter output is %u\r\n", step_out);
    }
    else
    {
        printk("Reading step count : FAILED\n");
    }
    return step_out;
}