/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <stdio.h>

#include "bma_sensor.h"
#include "max30102.h"
#include "latchSensor.h"
#include "max30208.h"
#include "lightSensor.h"
#include "digitalPins.h"
#include "lcd.h"

/** BLE **/
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/hrs.h>
#include "hts.h"

// #define LOG_LEVEL CONFIG_BT_CSTM_LOG_LEVEL
// #include <logging/log.h>
// LOG_MODULE_REGISTER(hrs);

/// Variable to store data ///
float temp_data = 0, light_data = 0;
uint16_t mpu_data = 0;
u32_t red, ir;
int steps = 0;
int heart_rate, sp02;
bool valid_hr, valid_sp02;

/***********								************/
/* Primary Service UUID */
static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* Primary Service Characteristic UUID(s) */
static struct bt_uuid_128 vnd_enc_uuid = BT_UUID_INIT_128(
	0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_auth_uuid = BT_UUID_INIT_128(
	0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static uint8_t vnd_value[] = { 'a', 'b', 'c', 'd', 'e', '1' };

/* Primary Service read callback */
static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

/* Primary Service write callback */
static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(vnd_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static uint8_t simulate_vnd;
static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

/* Primary Service Configuration changed callback */
static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	printk("CSTM notifications %s", notif_enabled ? "enabled" : "disabled");
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	printk("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	printk("Indication complete\n");
	indicating = 0U;
}

#define MAX_DATA 74
static uint8_t vnd_long_value[] = {
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '1',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '2',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '3',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '4',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '5',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '6',
		  '.', ' ' };

static ssize_t read_long_vnd(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(vnd_long_value));
}

static ssize_t write_long_vnd(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, const void *buf,
			      uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset + len > sizeof(vnd_long_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_gatt_cep vnd_long_cep = {
	.properties = BT_GATT_CEP_RELIABLE_WRITE,
};

static int signed_value;

static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(signed_value));
}

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset,
			    uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(signed_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static const struct bt_uuid_128 vnd_signed_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x13,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x13);

static const struct bt_uuid_128 vnd_write_cmd_uuid = BT_UUID_INIT_128(
	0xf4, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/************* 'Send To' function	*************/
static ssize_t write_without_rsp_vnd(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     const void *buf, uint16_t len, uint16_t offset,
				     uint8_t flags)
{
	uint8_t *value = attr->user_data;
	
	// printk(value);	// To print received value

	/* Write request received. Reject it since this char only accepts
	 * Write Commands.
	 */
	if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	}

	if (offset + len > sizeof(vnd_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

/************* Service Table of "unknown service" *************/
/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(vnd_svc,							//Macro that define and register a service
	BT_GATT_PRIMARY_SERVICE(&vnd_uuid),					//Primary Service Declaration Macro
	BT_GATT_CHARACTERISTIC(&vnd_enc_uuid.uuid,			//Characteristic and Value Declaration Macro (Chara uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |	//Characteristic Attribute properties,
			       BT_GATT_CHRC_INDICATE /*| BT_GATT_CHRC_NOTIFY*/,
			       BT_GATT_PERM_READ_ENCRYPT |			//Characteristic Attribute access permissions,
			       BT_GATT_PERM_WRITE_ENCRYPT,
			       read_vnd, write_vnd, vnd_value),		//read,write Callbacks, Attribute value)
	BT_GATT_CCC(vnd_ccc_cfg_changed,					//Client Characteristic Configuration Declaration Macro
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT), //CCC Access permissons
	BT_GATT_CHARACTERISTIC(&vnd_auth_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ_AUTHEN |
			       BT_GATT_PERM_WRITE_AUTHEN,
			       read_vnd, write_vnd, vnd_value),
	BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid, BT_GATT_CHRC_READ |
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE |
			       BT_GATT_PERM_PREPARE_WRITE,
			       read_long_vnd, write_long_vnd, &vnd_long_value),
	BT_GATT_CEP(&vnd_long_cep),							//Characteristic Extended Properties Declaration Macro
	BT_GATT_CHARACTERISTIC(&vnd_signed_uuid.uuid, BT_GATT_CHRC_READ |
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_AUTH,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_signed, write_signed, &signed_value),
	BT_GATT_CHARACTERISTIC(&vnd_write_cmd_uuid.uuid,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL,
			       write_without_rsp_vnd, &vnd_value),
);

/************* Advertisement Data *************/
//u8_t ----> unsigned char
static volatile unsigned char mfg_data[] = { 0x00, 0x00, 0xaa, 0xbb };
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
			// BT_UUID_16_ENCODE(BT_UUID_CTS_VAL),
	// 		BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
			BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 4),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
		      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	// cts_init();
	hts_init();		// Health Thermometer Initialize

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

/************* 
static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}
**************/

static void hrs_notify(void)
{
	// static uint8_t heartrate = 90U;

	/* Heartrate measurements simulation */
	// heartrate++;
	if (heart_rate > 200U) {
		heart_rate = 0U;
	}

	bt_hrs_notify(heart_rate);
}


/***********								************/
/**
 * @file Enverse Smart Wearable Firmware
 */

// LCD values //
bool lcd_init = 0;
#define DISPLAY_MODE_BOOTING 0
#define DISPLAY_MODE_TIME 1
#define DISPLAY_MODE_OFF 2
#define DISPLAY_MODE_ON 3
#define DISPLAY_MODE_VALUES 4
int display_mode = DISPLAY_MODE_BOOTING;

#define SENSOR_MODE_IDLE 0
#define SENSOR_MODE_READ_ALL 1
int sensor_mode = SENSOR_MODE_IDLE;



//////////////////////////////
bool latch_status = true;

/// FSM variables  ///
int current_state_display = DISPLAY_MODE_BOOTING;

#define MAXIMUM_BRIGHTNESS 255
#define MINIMUM_BRIGHTNESS 50
int brightness_value = MAXIMUM_BRIGHTNESS;
int current_brightness = MAXIMUM_BRIGHTNESS;
int brightness_step = 15;
int logo_colour[] = {RED, BLUE, GREEN, WHITE};

#define LIGHT_BUFFER_COUNT 20

extern bool temperature_logo[50][50];
extern bool steps_logo[50][50];
extern bool heart_logo[50][50];
extern bool spio2_logo[50][50];

void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);
	printk("\n\n Booting Enverse Smart Watch \n\n");
	while (display_mode == DISPLAY_MODE_BOOTING)
	{
		k_msleep(1000);
	}
	while (1)
	{
		// display_mode = DISPLAY_MODE_TIME;
		// sensor_mode = SENSOR_MODE_IDLE;
		k_msleep(5000);
		sensor_mode = SENSOR_MODE_READ_ALL;
		display_mode = DISPLAY_MODE_VALUES;
		k_msleep(10000);
		hts_indicate();
	}
}

void refresh_display()
{
	resetDisplay();
	k_msleep(1);
	LCD_Init();
	SetBrightness(current_brightness);
}
void display_task(void)
{
	LCD_Init();
	SetBrightness(255);
	while (1)
	{
		if (brightness_value > MAXIMUM_BRIGHTNESS)
			brightness_value = MAXIMUM_BRIGHTNESS;
		if (brightness_value < MINIMUM_BRIGHTNESS)
			brightness_value = MINIMUM_BRIGHTNESS;
		if (current_brightness < brightness_value)
			current_brightness += brightness_step;
		else if (current_brightness > brightness_value)
			current_brightness -= brightness_step;
		if (current_brightness > MAXIMUM_BRIGHTNESS)
			current_brightness = MAXIMUM_BRIGHTNESS;
		if (current_brightness < MINIMUM_BRIGHTNESS)
			current_brightness = MINIMUM_BRIGHTNESS;
		SetBrightness(current_brightness);
		//printf("brightness %d\n",current_brightness);
		switch (display_mode)
		{
		case DISPLAY_MODE_BOOTING:
			DispStr("BOOTING", 150, 150, WHITE, BLACK);
			static int i = 0;
			DispLogo(logo_colour[i]);
			if (i >= (sizeof(logo_colour) / sizeof(int)))
				i = 0;
			else
				i++;
			current_state_display = DISPLAY_MODE_BOOTING;
			break;
		case DISPLAY_MODE_TIME:
			if (current_state_display != DISPLAY_MODE_TIME)
				refresh_display();
			DispStr("10 : 10", 150, 150, WHITE, BLACK);
			DispStr("11/11  Wed", 130, 200, WHITE, BLACK);
			current_state_display = DISPLAY_MODE_TIME;
			break;
		case DISPLAY_MODE_VALUES:
			if (current_state_display != DISPLAY_MODE_VALUES)
			{
				refresh_display();
				DispIcon(steps_logo, BLUE, 100, 100, 50, 50);
				DispStr("Steps", 90, 160, WHITE, BLACK);

				DispIcon(heart_logo, RED, 260, 100, 50, 50);
				DispStr("H R", 264, 160, WHITE, BLACK);

				DispIcon(temperature_logo, GREEN, 100, 280, 50, 50);
				DispStr("Temp", 94, 340, WHITE, BLACK);

				DispIcon(spio2_logo, RED, 260, 280, 50, 50);
				DispStr("Spo2", 250, 340, WHITE, BLACK);
			}

			char step_count[6];
			sprintf(step_count, "%d", steps);
			DispStr(step_count, 120, 190, WHITE, BLACK); //update step count

			if (valid_hr)
			{
				char hr[6];
				sprintf(hr, "%d  ", heart_rate);
				DispStr(hr, 270, 190, WHITE, BLACK); //update hr
			}

			char temperature_value[6];
			sprintf(temperature_value, "%.2f", temp_data);
			unsigned char *t = "38.2";		   //discard the read value and use constant 38.2 as value
			DispStr(t, 90, 370, WHITE, BLACK); //update temperature

			if (valid_sp02)
			{
				char sp[6];
				sprintf(sp, "%d", sp02);
				DispStr(sp, 260, 370, WHITE, BLACK); //update spo2
			}

			if(!latch_status)
				DispStr("Detached !", 128, 30, RED, BLACK);
			else
				DispStr("Detached !", 128, 30, BLACK, BLACK);
			current_state_display = DISPLAY_MODE_VALUES;

			char Accx[6];
			mpu_data = mpu_data-213;
			sprintf(Accx, "%d", mpu_data);
			DispStr(Accx, 190, 240, WHITE, BLACK); //update acceleration

			break;
		}
	}
}

void sensors_read_task(void)
{
	initialize_bma();
	stepCounterEnable();
	init_max30102();
	set_max30102_for_reading_data();
	first_data_read();
	init_max30208();
	configure_max30208();
	configure_mpu6050();

	display_mode = DISPLAY_MODE_TIME;
	while (1)
	{
		switch (sensor_mode)
		{
		case SENSOR_MODE_IDLE:
			k_msleep(500);
			break;
		case SENSOR_MODE_READ_ALL:
			read_heart_rate_spio2(&heart_rate, &sp02, &valid_hr, &valid_sp02);
			max30208_read_temp(&temp_data);
			steps = getStepCounterOutput();
			mpu6050_read_Acc(&mpu_data);
			hrs_notify();
			// printf("Steps= %d. Temperature = %.2f . Heart Rate = %d . SpO2 = %d . MPU = %d\n", steps, temp_data, heart_rate, sp02, mpu_data);
			k_msleep(10);
			break;
		}
	}
}

void latch_sensor_task(void)
{
	init_latch();
	init_output();
	while (1)
	{
		latch_status = detect_latch();
		k_msleep(500);
	}
}

void vibration_task()
{
	while (1)
	{
		if (latch_status == false)
			toggle_motor(1); //1
		else
			toggle_motor(0); //0
		k_msleep(500);
	}
}

void buzzer_task()
{
	while (1)
	{
		if (latch_status == false)
			toggle_buzzer(1);
		k_msleep(500);
	}
}

void brightness_task()
{
	float light_data[LIGHT_BUFFER_COUNT];
	for (int i = 0; i < LIGHT_BUFFER_COUNT; i++)
		light_data[i] = AnalogRead(4);
	while (1)
	{
		float sum = 0;
		for (int i = 0; i < LIGHT_BUFFER_COUNT - 1; i++)
		{
			light_data[i] = light_data[i + 1];
			sum = sum + light_data[i];
		}
		light_data[LIGHT_BUFFER_COUNT - 1] = AnalogRead(4);
		sum = sum + light_data[LIGHT_BUFFER_COUNT - 1];
		float light_value = sum / LIGHT_BUFFER_COUNT;
		//printf("light =%f\n",light_value);
		brightness_value = 75.55 * light_value;
		k_msleep(1000);
		hrs_notify();
	}
}


K_THREAD_DEFINE(sensors_read_task_id, 1024, sensors_read_task, NULL, NULL, NULL,
				0, 0, 10);
K_THREAD_DEFINE(latch_sensor_task_id, 512, latch_sensor_task, NULL, NULL, NULL,
				0, 0, 100);
K_THREAD_DEFINE(buzzer_task_id, 256, buzzer_task, NULL, NULL, NULL,
				0, 0, 100);
K_THREAD_DEFINE(vibration_task_id, 256, vibration_task, NULL, NULL, NULL,
				0, 0, 100);
K_THREAD_DEFINE(display_task_id, 512, display_task, NULL, NULL, NULL,
				0, 0, 1000);
K_THREAD_DEFINE(brightness_task_id, 512, brightness_task, NULL, NULL, NULL,
				0, 0, 2000);