#include "latchSensor.h"

/* LED helpers, which use the led0 devicetree alias if it's available. */
static struct device *initialize_led(void);

static struct gpio_callback button_cb_data;

void button_pressed(struct device *dev, struct gpio_callback *cb,
                    u32_t pins)
{
    printk("Device Detached\n");
}

struct device *globalButton;
struct device *globalLed;

void init_latch()
{
    struct device *button;
    struct device *led;
    int ret;

    button = device_get_binding(SW0_GPIO_LABEL);
    if (button == NULL)
    {
        printk("Error: didn't find %s device\n", SW0_GPIO_LABEL);
        return;
    }

    ret = gpio_pin_configure(button, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
        return;
    }

    ret = gpio_pin_interrupt_configure(button,
                                       SW0_GPIO_PIN,
                                       GPIO_INT_EDGE_RISING);
    if (ret != 0)
    {
        printk("Error %d: failed to configure interrupt on %s pin %d\n",
               ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
        return;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_GPIO_PIN));
    gpio_add_callback(button, &button_cb_data);
    //printk("Set up button at %s pin %d\n", SW0_GPIO_LABEL, SW0_GPIO_PIN);

    led = initialize_led();
    globalButton=button;
    globalLed=led;

}

bool detect_latch()
{   
    return match_led_to_button(globalButton,globalLed);
}

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */

#define LED0_NODE DT_ALIAS(led6)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay) && DT_NODE_HAS_PROP(LED0_NODE, gpios)
#define LED0_GPIO_LABEL DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_GPIO_PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_GPIO_FLAGS (GPIO_OUTPUT | FLAGS_OR_ZERO(LED0_NODE))
#endif

#ifdef LED0_GPIO_LABEL
static struct device *initialize_led(void)
{
    struct device *led;
    int ret;

    led = device_get_binding(LED0_GPIO_LABEL);
    if (led == NULL)
    {
        printk("Didn't find LED device %s\n", LED0_GPIO_LABEL);
        return NULL;
    }

    ret = gpio_pin_configure(led, LED0_GPIO_PIN, LED0_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure LED device %s pin %d\n",
               ret, LED0_GPIO_LABEL, LED0_GPIO_PIN);
        return NULL;
    }

    //printk("Set up LED at %s pin %d\n", LED0_GPIO_LABEL, LED0_GPIO_PIN);

    return led;
}

bool match_led_to_button(struct device *button, struct device *led)
{
    bool val;
    val = gpio_pin_get(button, SW0_GPIO_PIN);
    gpio_pin_set(led, LED0_GPIO_PIN, !val);
    return val;
}

#else  /* !defined(LED0_GPIO_LABEL) */
static struct device *initialize_led(void)
{
    printk("No LED device was defined\n");
    return NULL;
}

bool match_led_to_button(struct device *button, struct device *led)
{
    return;
}
#endif /* LED0_GPIO_LABEL */