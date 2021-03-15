#include "max30102.h"
#include "algorithm.h"
//#define logging 0

#define BUFFER_SIZE 500
#define DUMP_SAMPLES 10

int sum, avg;

static int write_bytes(struct device *i2c_dev, u8_t addr, u8_t *data, u32_t num_bytes, u16_t slaveAddr)
{
    u8_t ret = 0x00;
#ifdef logging
    printk("Writing register - ox%x . Bytes =%d\n", addr, num_bytes);
#endif
    u8_t buffer[2];
    buffer[0] = &addr;
    buffer[1] = data;
    //ret = i2c_burst_write(i2c_dev, slaveAddr, addr, data, num_bytes);
    ret = i2c_reg_write_byte(i2c_dev, slaveAddr, addr, data);
#ifdef logging
    if (ret == MAX3_OK)
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
    //u8_t ret = i2c_burst_read(i2c_dev, slaveAddr, addr, data, num_bytes);
    u8_t ret = i2c_reg_read_byte(i2c_dev, slaveAddr, addr, data);
#ifdef logging
    if (ret == MAX3_OK)
        printk("Data read. Value = 0x%x\n", *(data));
#endif
    return ret;
}

static int read_bytes_fifo(struct device *i2c_dev, u8_t addr, u8_t *data, u32_t num_bytes, u16_t slaveAddr)
{
#ifdef logging
    printk("Reading register ox%x . Bytes =%d\n", addr, num_bytes);
#endif
    u8_t ret = i2c_burst_read(i2c_dev, slaveAddr, addr, data, num_bytes);
    //u8_t ret = i2c_reg_read_byte(i2c_dev, slaveAddr, addr, data);
#ifdef logging
    if (ret == MAX3_OK)
        printk("Data read. Value = 0x%x\n", *(data));
#endif
    return ret;
}

/* MAX30102 i2c write function */
static uint16_t max30102_i2c_write(uint8_t reg, uint8_t *data, uint16_t len)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    return write_bytes(i2c_dev, reg, data, len, MAX30102_ADDRESS);
}

/* MAX30102 i2c read function */
static uint16_t max30102_i2c_read(u8_t reg, u8_t *data, u16_t len)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    return read_bytes(i2c_dev, reg, data, len, MAX30102_ADDRESS);
}

static uint16_t max30102_i2c_read_fifo(u8_t reg, u8_t *data, u16_t len)
{
    struct device *i2c_dev;
    i2c_dev = device_get_binding(I2C_DEV);
    return read_bytes_fifo(i2c_dev, reg, data, len, MAX30102_ADDRESS);
}

bool max30102_reset()
{
    u16_t rslt = MAX3_OK;
    rslt |= max30102_i2c_write(MAX30102_CONFIG, 0x40, 1);
    if (rslt == MAX3_OK)
    {
        printk("RESET MAX30102 : OK \n ");
        return true;
    }
    else
    {
        printk("RESET MAX30102 : FAILED !!!\n ");
        return false;
    }
}

u16_t init_max30102()
{
    max30102_reset();
    k_msleep(1000);
    u8_t dummyData;
    max30102_i2c_read(0x00, &dummyData, 1);
    u8_t chipId;
    u16_t rslt = 0x00;
    rslt = max30102_i2c_read(MAX30102_PART_ID, &chipId, 1);
    if (rslt == MAX3_OK)
    {
        if (chipId == I_AM_MAX30102)
        {
            printk("Communication with MAX30102 : OK\n");
        }
        else
        {
            printk("Communication with MAX30102 : FAILED!!!\n");
        }
    }
    else
    {
        printk("Communication with MAX30102 : FAILED!!!\n I2C error\n");
    }
    return rslt;
}

u16_t set_max30102_for_reading_data()
{
    u16_t rslt = MAX3_OK;
    rslt |= max30102_i2c_write(MAX30102_INT1_ENABLE, 0x40, 1);
    rslt |= max30102_i2c_write(MAX30102_INT2_ENABLE, 0x00, 1);
    rslt |= max30102_i2c_write(MAX30102_FIFO_W_POINTER, 0x00, 1);
    rslt |= max30102_i2c_write(MAX30102_OVR_COUNTER, 0x00, 1);
    rslt |= max30102_i2c_write(MAX30102_FIFO_R_POINTER, 0x00, 1);
    rslt |= max30102_i2c_write(MAX30102_FIFO_CONFIG, 0x0F, 1);
    rslt |= max30102_i2c_write(MAX30102_CONFIG, 0x03, 1);
    rslt |= max30102_i2c_write(MAX30102_SPO2_CONFIG, 0x27, 1);
    rslt |= max30102_i2c_write(MAX30102_LED_CONFIG_1, 0x24, 1);
    rslt |= max30102_i2c_write(MAX30102_LED_CONFIG_2, 0x24, 1);
    rslt |= max30102_i2c_write(MAX30102_REG_PILOT_PA, 0x7F, 1);
    return (rslt);
}

u16_t max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
    u16_t rslt = MAX3_OK;
    u32_t temp_dat[6] = {0, 0, 0, 0, 0, 0};
    u8_t temp_dat1[6] = {0, 0, 0, 0, 0, 0};
    *pun_ir_led = 0;
    *pun_red_led = 0;
    u8_t temp1;
    max30102_i2c_read(MAX30102_FIFO_W_POINTER, &temp1, 1);
    max30102_i2c_read(MAX30102_FIFO_R_POINTER, &temp1, 1);
    max30102_i2c_read_fifo(MAX30102_FIFO_DATA_REG, temp_dat1, 6);
    for (int i = 0; i < 6; i++)
        temp_dat[i] = temp_dat1[i];
    *pun_ir_led = (temp_dat[0] << 16) + (temp_dat[1] << 8) + temp_dat[2];
    *pun_red_led = (temp_dat[3] << 16) + (temp_dat[4] << 8) + temp_dat[5];
    *pun_red_led &= 0x03FFFF; //Mask MSB [23:18]
    *pun_ir_led &= 0x03FFFF;  //Mask MSB [23:18]
    return rslt;
}

bool read_interupt_pin()
{
    bool val;
    struct device *button;
    button = device_get_binding(SW1_GPIO_LABEL);
    if (button == NULL)
    {
        printk("Error: didn't find %s device\n", SW1_GPIO_LABEL);
        return;
    }
    val = gpio_pin_get(button, SW1_GPIO_PIN);
    return val;
}

uint32_t aun_ir_buffer[BUFFER_SIZE];  //IR LED sensor data
int32_t n_ir_buffer_length;           //data length
uint32_t aun_red_buffer[BUFFER_SIZE]; //Red LED sensor data
int32_t n_sp02;                       //SPO2 value
int8_t ch_spo2_valid;                 //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;                 //heart rate value
int8_t ch_hr_valid;                   //indicator to show if the heart rate calculation is valid

uint32_t un_min, un_max, un_prev_data;

void first_data_read()
{
    un_min = 0x3FFFF;
    un_max = 0;
    n_ir_buffer_length = BUFFER_SIZE; //*
    int i = 0;
    for (i = 0; i < n_ir_buffer_length; i++)
    {
        u8_t data = 0;
        while (data != 0x40)
        {
            max30102_i2c_read(MAX30102_INT1_STATUS, &data, 1);
            //printf("Data = 0x%x ", data);
        }
        max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i)); //read from MAX30102 FIFO
        if (un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i]; //update signal min
        if (un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i]; //update signal max
        // printf("red=");
        // printf("%d", aun_red_buffer[i]);
        // printf(",ir=");
        // printf("%d\n\r", aun_ir_buffer[i]);
    }
    un_prev_data = aun_red_buffer[i];
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
}

void read_heart_rate_spio2(int *rate, int *spo2, bool *valid_hr, bool *valid_spo2)
{
    int i = 0;
    un_min = 0x3FFFF;
    un_max = 0;

    //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
    for (i = DUMP_SAMPLES; i < BUFFER_SIZE; i++)
    {
        aun_red_buffer[i - DUMP_SAMPLES] = aun_red_buffer[i];
        aun_ir_buffer[i - DUMP_SAMPLES] = aun_ir_buffer[i];

        //update the signal min and max
        if (un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i];
        if (un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i];
    }

    //take 100 sets of samples before calculating the heart rate.
    for (i = (BUFFER_SIZE - DUMP_SAMPLES); i < BUFFER_SIZE; i++)
    {
        un_prev_data = aun_red_buffer[i - 1];
        u8_t data = 0;
        while (data != 0x40)
        {
            max30102_i2c_read(MAX30102_INT1_STATUS, &data, 1);
            //printf("Data = 0x%x ", data);
        }
        max30102_i2c_read(MAX30102_FIFO_W_POINTER, &data, 1);
        //printf("Write = %d ", data);
        max30102_i2c_read(MAX30102_FIFO_R_POINTER, &data, 1);
        //("Read = %d ", data);
        max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));
        // printf("red= ");
        // printf("%d", aun_red_buffer[i]);
        // printf(" ir= ");
        // printf("%d", aun_ir_buffer[i]);
        // printf(", HR=%d, ", n_heart_rate);
        // printf("HRvalid=%d, ", ch_hr_valid);
        // printf("SpO2=%d, ", n_sp02);
        // printf("SPO2Valid=%d\n\r", ch_spo2_valid);
        *rate = n_heart_rate;
        *spo2 = n_sp02;
        if (ch_hr_valid == 1)
            *valid_hr = true;
        else
            *valid_hr = false;
        if (ch_spo2_valid == 1)
            *valid_spo2 = true;
        else
            *valid_spo2 = false;
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
}