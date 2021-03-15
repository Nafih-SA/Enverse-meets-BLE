#pragma once
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include "bma456.h"

#define I2C_DEV DT_LABEL(DT_ALIAS(i2c_0))

struct BMA456_ACCEL_DATA
{
    int datax;
    int datay;
    int dataz;
    u8_t result;
};

typedef enum
{
    CIC_AVG = 0,
    CONTINUOUS,
} MA456_PERF_MODE;

typedef enum
{
    OSR4_AVG1 = 0,
    OSR2_AVG2,
    NORMAL_AVG4,
    CIC_AVG8,
    RES_AVG16,
    RES_AVG32,
    RES_AVG64,
    RES_AVG128,
} MA456_BW;

typedef enum
{
    ODR_0_78_HZ = 1,
    ODR_1_5_HZ,
    ODR_3_1_HZ,
    ODR_6_25_HZ,
    ODR_12_5_HZ,
    ODR_25_HZ,
    ODR_50_HZ,
    ODR_100_HZ,
    ODR_200_HZ,
    ODR_400_HZ,
    ODR_800_HZ,
    ODR_1600_HZ,
} MBA456_ODR;

typedef enum
{
    RANGE_2G = 0,
    RANGE_4G,
    RANGE_8G,
    RANGE_16G,
} MA456_RANGE;

typedef enum
{
    PHONE_CONFIG = 0,
    WRIST_CONFIG = 1,
} MA456_PLATFORM_CONF;

void initialize_bma();
u16_t init_accel();
struct BMA456_ACCEL_DATA read_accel();
u16_t stepCounterEnable();

void getAcceleration(float *x, float *y, float *z);

u32_t getTemperature(void);

u32_t getStepCounterOutput(void);



