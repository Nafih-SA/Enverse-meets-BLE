#pragma once
#include <errno.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>

#define FLAGS_OR_ZERO(node)                          \
    COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags), \
                (DT_GPIO_FLAGS(node, gpios)),        \
                (0))

/*
 * Get button configuration from the devicetree sw0 alias.
 *
 * At least a GPIO device and pin number must be provided. The 'flags'
 * cell is optional.
 */

#define SW1_NODE DT_ALIAS(sw1)

#define SW1_GPIO_LABEL DT_GPIO_LABEL(SW1_NODE, gpios)
#define SW1_GPIO_PIN DT_GPIO_PIN(SW1_NODE, gpios)
#define SW1_GPIO_FLAGS (GPIO_INPUT | FLAGS_OR_ZERO(SW1_NODE))

#define I2C_DEV DT_LABEL(DT_ALIAS(i2c_0))

/******************************************************************************/
/*********** PULSE OXIMETER AND HEART RATE REGISTER MAPPING  **************/
/******************************************************************************/

// status registers
#define MAX30102_INT1_STATUS 0x00
#define MAX30102_INT2_STATUS 0x01
#define MAX30102_INT1_ENABLE 0x02
#define MAX30102_INT2_ENABLE 0x03

// FIFO registers
#define MAX30102_FIFO_W_POINTER 0x04
#define MAX30102_OVR_COUNTER 0x05
#define MAX30102_FIFO_R_POINTER 0x06
#define MAX30102_FIFO_DATA_REG 0x07

// configuration registers
#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_CONFIG 0x09
#define MAX30102_SPO2_CONFIG 0x0A
#define MAX30102_LED_CONFIG_1 0x0C
#define MAX30102_LED_CONFIG_2 0x0D
#define MAX30102_REG_PILOT_PA 0x10

// temperature registers
#define MAX30102_TEMP_INTEGER 0x1F
#define MAX30102_TEMP_FRACTION 0x20
#define MAX30102_TEMP_CONFIG 0x21

// PART ID registers
#define MAX30102_REVISION_ID 0xFE
#define MAX30102_PART_ID 0xFF

#define I_AM_MAX30102 0x15

/************************************** REGISTERS VALUE *******************************************/

#define MAX30102_ADDRESS 0x57

#define MAX3_OK 0x00

u16_t init_max30102();
bool max30102_reset();
u16_t set_max30102_for_reading_data();

static uint16_t max30102_i2c_write(uint8_t reg, uint8_t *data, uint16_t len);
static uint16_t max30102_i2c_read(u8_t reg, u8_t *data, u16_t len);
u16_t max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);
void read_heart_rate_spio2(int *rate, int *spo2, bool *valid_hr, bool *valid_spo2);
void first_data_read();
