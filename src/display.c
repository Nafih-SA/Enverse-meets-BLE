#include "display.h"

#define MAX_USER_DATA_LENGTH 1024

struct device *globalcs;
struct device *globalLcdRst;
struct device *globalVciEn;
struct device *globalDcx;                                                                

const int spi_freq = 8000000;

static uint8_t data[MAX_USER_DATA_LENGTH];

s16_t init_lcd_output()
{
    struct device *cs;
    int ret;
    cs = device_get_binding(CS_GPIO_LABEL);
    if (cs == NULL)
    {
        printk("Didn't find device %s\n", CS_GPIO_LABEL);
        return -1;
    }
    ret = gpio_pin_configure(cs, CS_GPIO_PIN, CS_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure device %s pin %d\n",
               ret, CS_GPIO_LABEL, CS_GPIO_PIN);
        return ret;
    }

    struct device *lcdRst;
    lcdRst = device_get_binding(LCD_RST_GPIO_LABEL);
    if (lcdRst == NULL)
    {
        printk("Didn't find device %s\n", LCD_RST_GPIO_LABEL);
        return -1;
    }
    ret = gpio_pin_configure(lcdRst, LCD_RST_GPIO_PIN, LCD_RST_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure device %s pin %d\n",
               ret, LCD_RST_GPIO_LABEL, LCD_RST_GPIO_PIN);
        return ret;
    }

    struct device *vciEn;
    vciEn = device_get_binding(VCI_EN_GPIO_LABEL);
    if (vciEn == NULL)
    {
        printk("Didn't find device %s\n", VCI_EN_GPIO_LABEL);
        return -1;
    }
    ret = gpio_pin_configure(lcdRst, VCI_EN_GPIO_PIN, VCI_EN_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure device %s pin %d\n",
               ret, VCI_EN_GPIO_LABEL, VCI_EN_GPIO_PIN);
        return ret;
    }

    struct device *dcx;
    dcx = device_get_binding(DCX_GPIO_LABEL);
    if (dcx == NULL)
    {
        printk("Didn't find device %s\n", DCX_GPIO_LABEL);
        return -1;
    }
    ret = gpio_pin_configure(lcdRst, DCX_GPIO_PIN, DCX_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure device %s pin %d\n",
               ret, DCX_GPIO_LABEL, DCX_GPIO_PIN);
        return ret;
    }

    printk("INITIALISING OUTPUTS : OK\n");

    globalcs = cs;
    globalLcdRst = lcdRst;
    globalVciEn = vciEn;
    globalDcx = dcx;

    return 0;
}

bool setOutputcs(bool val)
{
    int err = gpio_pin_set(globalcs, CS_GPIO_PIN, val);
    if (err != 0)
    {
        printk("Error in gpio cs");
        return 1;
    }
    return 0;
}

bool setOutputLcdRst(bool val)
{
    int err = gpio_pin_set(globalLcdRst, LCD_RST_GPIO_PIN, val);
    if (err != 0)
    {
        printk("Error in gpio reset");
        return 1;
    }
    return 0;
}

bool setOutputVciEn(bool val)
{
    int err = gpio_pin_set(globalVciEn, VCI_EN_GPIO_PIN, val);
    if (err != 0)
    {
        printk("Error in gpio vcien");
        return 1;
    }
    return 0;
}

bool setOutputDcx(bool val)
{
    int err = gpio_pin_set(globalDcx, DCX_GPIO_PIN, val);
    if (err != 0)
    {
        printk("Error in gpio dcx");
        return 1;
    }
    return 0;
}

static int spi_acess(const struct device *spi, struct spi_config *spi_cfg, uint8_t cmd, uint16_t addr, void *data, size_t len)
{
    uint8_t access[2];
    struct spi_buf bufs[] =
        {
            {.buf = data,
             .len = len}
        };
    struct spi_buf_set tx =
        {
            .buffers = bufs,
            .count = 1
        };
    return spi_write(spi, spi_cfg, &tx);
}

int write_bytes(const struct device *spi, struct spi_config *spi_cfg,
                       uint16_t addr, uint8_t *data, uint32_t num_bytes)
{
    int err;
    err = spi_acess(spi, spi_cfg, 0, addr, data, num_bytes);
    if (err)
    {
        printk("Error during SPI write\n");
        return -EIO;
    }
    return 0;
}

int read_bytes(const struct device *spi, struct spi_config *spi_cfg,
                      uint16_t addr, uint8_t *data, uint32_t num_bytes)
{
    int err;
    err = spi_acess(spi, spi_cfg, 1, addr, data, num_bytes);
    if (err)
    {
        printk("Error during SPI read\n");
        return -EIO;
    }
    return 0;
}