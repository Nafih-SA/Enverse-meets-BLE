#include <zephyr.h>

#include <device.h>
#include <drivers/adc.h>
#include <sys/printk.h>
#include <string.h>

// Simple analog input method
// this just reads a sample then waits then returns it

// ADC Sampling Settings
// doc says that impedance of 800K == 40usec sample time
#define ADC_DEVICE_NAME		DT_LABEL(DT_ALIAS(adcctrl))
#define ADC_RESOLUTION		10
#define ADC_GAIN			ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define BUFFER_SIZE			6


static struct device* init_adc(int channel);
float AnalogRead(int channel);

