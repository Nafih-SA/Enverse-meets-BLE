#include "digitalPins.h"

struct device *globalBuzzer;
struct device *globalMotor;

s16_t init_output()
{
    struct device *buzzer;
    int ret;
    buzzer = device_get_binding(BUZZER_GPIO_LABEL);
    if (buzzer == NULL)
    {
        printk("Didn't find device %s\n", BUZZER_GPIO_LABEL);
        return -1;
    }
    ret = gpio_pin_configure(buzzer, BUZZER_GPIO_PIN, BUZZER_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure device %s pin %d\n",
               ret, BUZZER_GPIO_LABEL, BUZZER_GPIO_PIN);
        return ret;
    }
    struct device *motor;
    motor = device_get_binding(MOTOR_GPIO_LABEL);
    if (motor == NULL)
    {
        printk("Didn't find device %s\n", MOTOR_GPIO_LABEL);
        return -1;
    }
    ret = gpio_pin_configure(motor, MOTOR_GPIO_PIN, MOTOR_GPIO_FLAGS);
    if (ret != 0)
    {
        printk("Error %d: failed to configure device %s pin %d\n",
               ret, MOTOR_GPIO_LABEL, MOTOR_GPIO_PIN);
        return ret;
    }
    printk("INITIALISING OUTPUTS : OK\n");
    globalBuzzer = buzzer;
    globalMotor = motor;
    return 0;
}

bool toggle_buzzer(bool val)
{
    if (val = 1)
    {
        for (int i = 0; i < 1000; i++)
        {
            gpio_pin_set(globalBuzzer, BUZZER_GPIO_PIN, 1);
            k_busy_wait(250);
            gpio_pin_set(globalBuzzer, BUZZER_GPIO_PIN, 0);
            k_busy_wait(250);
        }
    }
    else
        gpio_pin_set(globalBuzzer, BUZZER_GPIO_PIN, 1);
    return val;
}
bool toggle_motor(bool val)
{
    if (val)
    {
        gpio_pin_set(globalMotor, MOTOR_GPIO_PIN, 1);
        k_msleep(250);
        gpio_pin_set(globalMotor, MOTOR_GPIO_PIN, 0);
    }
    else
    {
        gpio_pin_set(globalMotor, MOTOR_GPIO_PIN, 1);
    }
}