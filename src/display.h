#pragma once


#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <inttypes.h>

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),		\
		    (DT_GPIO_FLAGS(node, gpios)),			\
		    (0))

#define CS_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(CS_NODE, okay) && DT_NODE_HAS_PROP(CS_NODE, gpios)
#define CS_GPIO_LABEL DT_GPIO_LABEL(CS_NODE, gpios)
#define CS_GPIO_PIN DT_GPIO_PIN(CS_NODE, gpios)
#define CS_GPIO_FLAGS (GPIO_OUTPUT | FLAGS_OR_ZERO(CS_NODE))
#endif

#define LCD_RST_NODE DT_ALIAS(led1)

#if DT_NODE_HAS_STATUS(LCD_RST_NODE, okay) && DT_NODE_HAS_PROP(LCD_RST_NODE, gpios)
#define LCD_RST_GPIO_LABEL DT_GPIO_LABEL(LCD_RST_NODE, gpios)
#define LCD_RST_GPIO_PIN DT_GPIO_PIN(LCD_RST_NODE, gpios)
#define LCD_RST_GPIO_FLAGS (GPIO_OUTPUT | FLAGS_OR_ZERO(LCD_RST_NODE))
#endif

#define VCI_EN_NODE DT_ALIAS(led4)

#if DT_NODE_HAS_STATUS(VCI_EN_NODE, okay) && DT_NODE_HAS_PROP(VCI_EN_NODE, gpios)
#define VCI_EN_GPIO_LABEL DT_GPIO_LABEL(VCI_EN_NODE, gpios)
#define VCI_EN_GPIO_PIN DT_GPIO_PIN(VCI_EN_NODE, gpios)
#define VCI_EN_GPIO_FLAGS (GPIO_OUTPUT | FLAGS_OR_ZERO(VCI_EN_NODE))
#endif

#define DCX_NODE DT_ALIAS(led5)

#if DT_NODE_HAS_STATUS(DCX_NODE, okay) && DT_NODE_HAS_PROP(DCX_NODE, gpios)
#define DCX_GPIO_LABEL DT_GPIO_LABEL(DCX_NODE, gpios)
#define DCX_GPIO_PIN DT_GPIO_PIN(DCX_NODE, gpios)
#define DCX_GPIO_FLAGS (GPIO_OUTPUT | FLAGS_OR_ZERO(DCX_NODE))
#endif


s16_t init_lcd_output();
bool setOutputcs(bool val);
bool setOutputLcdRst(bool val);
bool setOutputVciEn(bool val);
bool setOutputDcx(bool val);

int write_bytes(const struct device *spi, struct spi_config *spi_cfg,
		       uint16_t addr, uint8_t *data, uint32_t num_bytes);
int read_bytes(const struct device *spi, struct spi_config *spi_cfg,
		      uint16_t addr, uint8_t *data, uint32_t num_bytes);
              

