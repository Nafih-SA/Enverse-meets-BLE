#include "lightSensor.h"

static bool _IsInitialized = false;
static uint8_t _LastChannel = 250;
static s16_t m_sample_buffer[BUFFER_SIZE];

// the channel configuration with channel not yet filled in
static struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = 0, // gets set during init
	.differential	  = 0,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = 0, // gets set during init
#endif
};

// return device* for the adc
struct device* getAdcDevice(void)
{
	return device_get_binding(ADC_DEVICE_NAME);
}

// initialize the adc channel
static struct device* init_adc(int channel)
{
	int ret;
	struct device *adc_dev = getAdcDevice();
	if(_LastChannel != channel)
	{
		_IsInitialized = false;
		_LastChannel = channel;
	}

	if ( adc_dev != NULL && !_IsInitialized)
	{
		// strangely channel_id gets the channel id and input_positive gets id+1
		m_1st_channel_cfg.channel_id = channel;
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        m_1st_channel_cfg.input_positive = channel+1,
#endif
		ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
		if(ret != 0)
		{
			//LOG_INF("Setting up of the first channel failed with code %d", ret);
			adc_dev = NULL;
		}
		else
		{
			_IsInitialized = true;	// we don't have any other analog users
		}
	}
	memset(m_sample_buffer, 0, sizeof(m_sample_buffer));
	return adc_dev;
}

// ------------------------------------------------
// read one channel of adc
// ------------------------------------------------
static s16_t readOneChannel(int channel)
{
	const struct adc_sequence sequence = {
		.options     = NULL,				// extra samples and callback
		.channels    = BIT(channel),		// bit mask of channels to read
		.buffer      = m_sample_buffer,		// where to put samples read
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,		// desired resolution
		.oversampling = 0,					// don't oversample
		.calibrate = 0						// don't calibrate
	};

	int ret;
	s16_t sample_value = -1;

	struct device *adc_dev = init_adc(channel);

	if (adc_dev)
	{
		ret = adc_read(adc_dev, &sequence);
		if(ret == 0)
		{
			sample_value = m_sample_buffer[0];
		}
	}

	return sample_value;
}


// ------------------------------------------------
// high level read adc channel and convert to float voltage
// ------------------------------------------------
float AnalogRead(int channel)
{

	s16_t sv = readOneChannel(channel);
	if(sv == -1)
	{
		return sv;
	}

	// Convert the result to voltage
	// Result = [V(p) - V(n)] * GAIN/REFERENCE / 2^(RESOLUTION)
	int multip = 256;
	// find 2**adc_resolution
	switch(ADC_RESOLUTION)
	{
		default :
		case 8 :
			multip = 256;
			break;
		case 10 :
			multip = 1024;
			break;
		case 12 :
			multip = 4096;
			break;
		case 14 :
			multip = 16384;
			break;
	}
	
	// the 3.6 relates to the voltage divider being used in my circuit
	float fout = (sv * 3.6 / multip);
	return fout;
}