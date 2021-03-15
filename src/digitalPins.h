#pragma once

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),		\
		    (DT_GPIO_FLAGS(node, gpios)),			\
		    (0))


#define BUZZER_PIN DT_ALIAS(led3)

#define BUZZER_GPIO_LABEL DT_GPIO_LABEL(BUZZER_PIN, gpios)
#define BUZZER_GPIO_PIN DT_GPIO_PIN(BUZZER_PIN, gpios)
#define BUZZER_GPIO_FLAGS (GPIO_OUTPUT|FLAGS_OR_ZERO(BUZZER_PIN))

#define MOTOR_PIN DT_ALIAS(led2)

#define MOTOR_GPIO_LABEL DT_GPIO_LABEL(MOTOR_PIN, gpios)
#define MOTOR_GPIO_PIN DT_GPIO_PIN(MOTOR_PIN, gpios)
#define MOTOR_GPIO_FLAGS (GPIO_OUTPUT |FLAGS_OR_ZERO(MOTOR_PIN))

s16_t init_output();

bool toggle_buzzer(bool val);
bool toggle_motor(bool val);
