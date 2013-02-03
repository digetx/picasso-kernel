/*
 * Battery and Power Management driver for acer a50x tablets
 *
 * based on nvidia's sbs and acer's ec drivers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/leds.h>

#include <linux/power/ec-battery.h>

#include <asm/system.h>

#include <media/yuv5_sensor.h>
#include <media/lsc_from_ec.h>

struct sensor_reg_copy EC_D65_patch_ram_table[LSC_LINE + 1];
struct sensor_reg_copy EC_CWF_patch_ram_table[LSC_LINE + 1];
int lsc_from_ec_status = 0;

#define DRV_NAME			"ec-control"
#define BATTERY_NAME			"ec-battery"
#define POLL_TIME_DEFAULT		360
#define POLL_TIME_CHARGING		60
#define POLL_TIME_MIN			25
#define POLL_TIME_THROTTLE		120
#define CAPACITY_LOW			7

struct ec_reg_data {
	u8  addr;
	u16 timeout;
};

#define EC_REG_DATA(_struct_name, _addr, _timeout) \
static struct ec_reg_data _struct_name = { \
	.addr = _addr, \
	.timeout = _timeout,\
};

/* Non-battery regs		addr	timeout */
EC_REG_DATA(BOARD_ID,		0x32,	10);
EC_REG_DATA(RESET_LED,		0x40,	100);
EC_REG_DATA(LEDS_OFF,		0x41,	100);
EC_REG_DATA(POWER_LED_ON,	0x42,	100);
EC_REG_DATA(CHARGE_LED_ON,	0x43,	100);
EC_REG_DATA(AUDIO_CTRL,		0x44,	0);
EC_REG_DATA(POWER_CTRL_3G,	0x45,	100);
EC_REG_DATA(GPS_POWER_OFF,	0x47,	0);
EC_REG_DATA(GPS_3G_STATUS_RD,	0x48,	0);
EC_REG_DATA(GPS_3G_STATUS_WR,	0x49,	0);
EC_REG_DATA(GPS_POWER_ON,	0x4A,	0);
EC_REG_DATA(MISC_CTRL_RD,	0x4C,	10);
EC_REG_DATA(MISC_CTRL_WR,	0x4D,	10);
EC_REG_DATA(SHUTDOWN,		0x52,	0);
EC_REG_DATA(WARM_REBOOT,	0x54,	0);
EC_REG_DATA(COLD_REBOOT,	0x55,	1000);
EC_REG_DATA(ANDROID_LEDS_OFF,	0x5A,	100);
EC_REG_DATA(BTMAC_RD,		0x62,	10);
EC_REG_DATA(BTMAC_WR,		0x63,	10);
EC_REG_DATA(WIFIMAC_RD,		0x64,	10);
EC_REG_DATA(WIFIMAC_WR,		0x65,	10);
EC_REG_DATA(LS_GAIN_RD,		0x71,	10);
EC_REG_DATA(LS_GAIN_WR,		0x72,	10);
EC_REG_DATA(GYRO_GAIN_RD,	0x73,	10);
EC_REG_DATA(GYRO_GAIN_WR,	0x74,	10);
EC_REG_DATA(SET_RAM_BANK,	0x75,	0);
EC_REG_DATA(WRITE_RAM_ADDR,	0x77,	20);
EC_REG_DATA(READ_RAM_ADDR,	0x78,	20);

enum {
	REG_VOLTAGE,
	REG_CAPACITY,
	REG_HEALTH,
	REG_DESIGN_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_TEMPERATURE,
	REG_CURRENT,
};

#define EC_DATA(_psp, _addr, _timeout) { \
	.psp = _psp, \
	.reg_data = { \
		.addr = _addr, \
		.timeout = _timeout\
	}, \
}

static const struct chip_data {
	enum power_supply_property psp;
	struct ec_reg_data reg_data;
} ec_data[] = {
	[REG_VOLTAGE] =
		EC_DATA(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0x01, 0),
	[REG_CAPACITY] =
		EC_DATA(POWER_SUPPLY_PROP_CAPACITY, 0x00, 0),
	[REG_CURRENT] =
		EC_DATA(POWER_SUPPLY_PROP_CURRENT_NOW, 0x03, 10),
	[REG_DESIGN_CAPACITY] =
		EC_DATA(POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN, 0x08, 0),
	[REG_HEALTH] =
		EC_DATA(POWER_SUPPLY_PROP_HEALTH, 0x09, 10),
	[REG_SERIAL_NUMBER] =
		EC_DATA(POWER_SUPPLY_PROP_SERIAL_NUMBER, 0x6a, 0),
	[REG_TEMPERATURE] =
		EC_DATA(POWER_SUPPLY_PROP_TEMP, 0x0A, 0),
};

static enum power_supply_property ec_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

struct ec_led {
	struct led_classdev		cdev;
	struct work_struct		work;
	u8				new_state;
};

struct ec_info {
	struct i2c_client		*client;
	struct power_supply		power_supply;
	struct ec_platform_data		*pdata;
	struct ec_led			power_white_led;
	struct ec_led			power_orange_led;
	bool				poll_disabled;
	bool				is_supplied;
	bool				is_panic;
	int				capacity;
	int				poll_interval;
	int				acoustic_table;
	int				power_state_3g;
	int				power_state_gps;
	struct delayed_work		work;
	struct delayed_work		lsc_work;
	struct notifier_block		panic_notifier;
	struct mutex			lock;
};

#define I2C_ERR_TIMEOUT	500
static int ec_read_word_data_locked(struct i2c_client *client,
				    struct ec_reg_data reg_data)
{
	struct ec_info *chip = i2c_get_clientdata(client);
	s32 ret = 0;
	int retries = 1;

	if (chip->pdata)
		retries = max(chip->pdata->i2c_retry_count + 1, 1);

	while (retries > 0) {
		ret = i2c_smbus_read_word_data(client, reg_data.addr);
		if (ret >= 0)
			break;
		msleep(I2C_ERR_TIMEOUT);
		retries--;
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c read at address 0x%x failed\n",
			__func__, reg_data.addr);
		return ret;
	}

	msleep(reg_data.timeout);

	return le16_to_cpu(ret);
}

static int ec_read_word_data(struct i2c_client *client,
			     struct ec_reg_data reg_data)
{
	struct ec_info *chip = i2c_get_clientdata(client);
	s32 ret;

	mutex_lock(&chip->lock);
	ret = ec_read_word_data_locked(client, reg_data);
	mutex_unlock(&chip->lock);

	return ret;
}

static int ec_write_word_data_locked(struct i2c_client *client,
				     struct ec_reg_data reg_data, u16 value)
{
	struct ec_info *chip = i2c_get_clientdata(client);
	s32 ret = 0;
	int retries = 1;

	if (chip->pdata)
		retries = max(chip->pdata->i2c_retry_count + 1, 1);

	while (retries > 0) {
		ret = i2c_smbus_write_word_data(client, reg_data.addr,
						le16_to_cpu(value));
		if (ret >= 0)
			break;
		msleep(I2C_ERR_TIMEOUT);
		retries--;
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c write to address 0x%x failed\n",
			__func__, reg_data.addr);
		return ret;
	}

	msleep(reg_data.timeout);

	return 0;
}

static int ec_write_word_data(struct i2c_client *client,
			      struct ec_reg_data reg_data, u16 value)
{
	struct ec_info *chip = i2c_get_clientdata(client);
	s32 ret;

	mutex_lock(&chip->lock);
	ret = ec_write_word_data_locked(client, reg_data, value);
	mutex_unlock(&chip->lock);

	return ret;
}

/*
 * Misc ec control
 */
#define PART_SZ	4
static void ec_read_multipart(struct i2c_client *client,
			      char *buf,
			      struct ec_reg_data reg_data, int parts_nb)
{
	struct ec_info *chip = i2c_get_clientdata(client);
	char *write_offset;
	int length, i;
	s32 ret;

	length = parts_nb * PART_SZ;
	write_offset = buf + length;

	mutex_lock(&chip->lock);
	for (i = 0; i < parts_nb; i++) {
		ret = ec_read_word_data_locked(client, reg_data);

		write_offset -= PART_SZ;

		snprintf(write_offset, length + 1, "%04x%s",
			 ret, (i == 0) ? "" : write_offset + PART_SZ);
	}
	mutex_unlock(&chip->lock);
}

static void ec_write_multipart(struct i2c_client *client,
			       const char *buf,
			       struct ec_reg_data reg_data, int parts_nb)
{
	struct ec_info *chip = i2c_get_clientdata(client);
	char part_buf[PART_SZ + 1];
	int read_offset, val, i;
	s32 ret;

	/* don't count trailing "\n" */
	ret = strlen(buf) - 1;

	if (ret != parts_nb * PART_SZ) {
		dev_err(&client->dev,
			"%s: length %d is not equal to required %d\n",
			  __func__, ret, parts_nb * PART_SZ);
		return;
	}

	mutex_lock(&chip->lock);
	for (i = 0; i < parts_nb; i++) {
		read_offset = (parts_nb - i + 1) * PART_SZ;

		snprintf(part_buf, ARRAY_SIZE(part_buf),
			 "%s", buf + read_offset);

		ret = kstrtoint(part_buf, 16, &val);
		if (ret < 0)
			dev_err(&client->dev,
				"%s: failed to convert hex str: %s\n",
				__func__, part_buf);

		ec_write_word_data_locked(client, reg_data, val);
	}
	mutex_unlock(&chip->lock);
}

#define GYRO_GAIN_PARTS_NB	18
static ssize_t gyro_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_read_multipart(client, buf, GYRO_GAIN_RD,
			  GYRO_GAIN_PARTS_NB);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t gyro_store(struct device *dev,
			  struct device_attribute *attr,
			  const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_multipart(client, buf, GYRO_GAIN_WR,
			   GYRO_GAIN_PARTS_NB);

	return n;
}

static ssize_t pwr_led_on_store(struct device *dev,
				struct device_attribute *attr,
				const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_word_data(client, POWER_LED_ON, 0);

	return n;
}

static ssize_t chrg_led_on_store(struct device *dev,
				 struct device_attribute *attr,
				 const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_word_data(client, CHARGE_LED_ON, 0);

	return n;
}

static ssize_t reset_led_store(struct device *dev,
			       struct device_attribute *attr,
			       const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_word_data(client, RESET_LED, 0);

	return n;
}

static ssize_t leds_off_store(struct device *dev,
			      struct device_attribute *attr,
			      const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_word_data(client, LEDS_OFF, 0);

	return n;
}

static ssize_t android_off_store(struct device *dev,
				 struct device_attribute *attr,
				 const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_word_data(client, ANDROID_LEDS_OFF, 0);

	return n;
}

static ssize_t ls_gain_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s32 ret;

	ret = ec_read_word_data(client, LS_GAIN_RD);

	return sprintf(buf, "%04x\n", ret);
}

static ssize_t ls_gain_store(struct device *dev,
			     struct device_attribute *attr,
			     const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;
	s32 ret;

	ret = kstrtoint(buf, 16, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert hex str: %s\n", __func__, buf);
		return n;
	}

	ec_write_word_data(client, LS_GAIN_WR, val);

	return n;
}

#define BTMAC_PARTS_NB	3
static ssize_t btmac_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_read_multipart(client, buf, BTMAC_RD,
			  BTMAC_PARTS_NB);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t btmac_store(struct device *dev,
			   struct device_attribute *attr,
			   const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_multipart(client, buf, BTMAC_WR,
			   BTMAC_PARTS_NB);

	return n;
}

#define WIFIMAC_PARTS_NB	3
static ssize_t wifimac_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_read_multipart(client, buf, WIFIMAC_RD,
			  WIFIMAC_PARTS_NB);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t wifimac_store(struct device *dev,
			     struct device_attribute *attr,
			     const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	ec_write_multipart(client, buf, WIFIMAC_WR,
			   WIFIMAC_PARTS_NB);

	return n;
}

static ssize_t device_status_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s32 ret;
	int i;

	ret = ec_read_word_data(client, GPS_3G_STATUS_RD);

	for (i = 15; i >= 0; i--)
		buf[i] = ret >> (15 - i) & 0x1 ? '1' : '0';

	return sprintf(buf, "%s\n", buf);
}

static ssize_t device_status_store(struct device *dev,
				   struct device_attribute *attr,
				   const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	ec_write_word_data(client, GPS_3G_STATUS_WR, val);

	return n;
}

static ssize_t status_3g_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ec_info *chip = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", chip->power_state_3g);
}

static ssize_t status_3g_store(struct device *dev,
			       struct device_attribute *attr,
			       const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ec_info *chip = i2c_get_clientdata(client);
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	chip->power_state_3g = val;
	ec_write_word_data(client, POWER_CTRL_3G, val);

	return n;
}

static ssize_t status_gps_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ec_info *chip = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", chip->power_state_gps);
}

static ssize_t status_gps_store(struct device *dev,
				struct device_attribute *attr,
				const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ec_info *chip = i2c_get_clientdata(client);
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	chip->power_state_gps = !!val;

	if (chip->power_state_gps)
		ec_write_word_data(client, GPS_POWER_ON, 0);
	else
		ec_write_word_data(client, GPS_POWER_OFF, 0);

	return n;
}

#define CABC_MASK	BIT(8)
static void ec_set_lcd_cabc(struct i2c_client *client, bool enable)
{
	s32 ret;

	ret = ec_read_word_data(client, MISC_CTRL_RD);
	if (ret < 0)
		return;

	if (enable)
		ret |= CABC_MASK;
	else
		ret &= (~CABC_MASK);

	ec_write_word_data(client, MISC_CTRL_WR, ret);
}

static ssize_t cabc_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	bool enabled;

	enabled = ec_read_word_data(client, MISC_CTRL_RD) & CABC_MASK;

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t cabc_store(struct device *dev,
			  struct device_attribute *attr,
			  const char * buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	ec_set_lcd_cabc(client, !!val);

	return n;
}

static DEVICE_ATTR(GyroGain, 0666, gyro_show, gyro_store);
static DEVICE_ATTR(PowerLED, 0222, NULL, pwr_led_on_store);
static DEVICE_ATTR(ChargeLED, 0222, NULL, chrg_led_on_store);
static DEVICE_ATTR(OriSts, 0222, NULL, reset_led_store);
static DEVICE_ATTR(OffLED, 0222, NULL, leds_off_store);
static DEVICE_ATTR(LEDAndroidOff, 0222, NULL, android_off_store);
static DEVICE_ATTR(AutoLSGain, 0666, ls_gain_show, ls_gain_store);
static DEVICE_ATTR(BTMAC, 0666, btmac_show, btmac_store);
static DEVICE_ATTR(WIFIMAC, 0666, wifimac_show, wifimac_store);
static DEVICE_ATTR(DeviceStatus, 0666, device_status_show, device_status_store);
static DEVICE_ATTR(ThreeGPower, 0666, status_3g_show, status_3g_store);
static DEVICE_ATTR(GPSPower, 0666, status_gps_show, status_gps_store);
static DEVICE_ATTR(Cabc, 0666, cabc_show, cabc_store);

static struct attribute *ec_control_attrs[] = {
	&dev_attr_GyroGain.attr,
	&dev_attr_PowerLED.attr,
	&dev_attr_ChargeLED.attr,
	&dev_attr_OriSts.attr,
	&dev_attr_OffLED.attr,
	&dev_attr_LEDAndroidOff.attr,
	&dev_attr_AutoLSGain.attr,
	&dev_attr_BTMAC.attr,
	&dev_attr_WIFIMAC.attr,
	&dev_attr_DeviceStatus.attr,
	&dev_attr_ThreeGPower.attr,
	&dev_attr_GPSPower.attr,
	&dev_attr_Cabc.attr,
	NULL,
};

static struct attribute_group ec_control_attr_group =
						{ .attrs = ec_control_attrs };

#define EC_LED_WORK(_color, _addr) \
static void ec_pwr_##_color##_led_work(struct work_struct *work) \
{ \
	struct ec_led *led = \
		container_of(work, struct ec_led, work); \
	struct ec_info *chip = \
		container_of(led, struct ec_info, power_##_color##_led); \
	struct i2c_client *client = chip->client; \
\
	if (led->new_state == LED_OFF) { \
		ec_write_word_data(client, RESET_LED, 0); \
		ec_write_word_data(client, ANDROID_LEDS_OFF, 0); \
	} else { \
		ec_write_word_data(client, _addr, 0); \
	} \
}

EC_LED_WORK(white, POWER_LED_ON);
EC_LED_WORK(orange, CHARGE_LED_ON);

static void ec_pwr_led_change_state(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	struct ec_led *led =
		container_of(led_cdev, struct ec_led, cdev);

	led->new_state = value;
	schedule_work(&led->work);
}

#define RAM_BANK	0x0001
#define PATCH_RAM_ADDR	0x098A
#define PATCH_D65	0x1718
#define PATCH_CWF	0x164C
#define MCU_ADDR	0x0990
#define UNDEFINED	0xFFFF
static void ec_lens_correction_preloading(struct work_struct *work)
{
#ifdef CONFIG_LSC_FROM_EC
	s32 ret;
	int i;
	u16 h_data;
	u16 l_data;
	u16 mcu_addr = MCU_ADDR;
	u16 ram_address = 0x0000;
	struct ec_info *chip;
	struct i2c_client *client;

	chip = container_of(work, struct ec_info, lsc_work.work);
	client = chip->client;

	/* initialize EC_D65_patch_ram_table, EC_CWF_patch_ram_table */
	for (i = 0; i < LSC_LINE + 1; i++) {
		EC_D65_patch_ram_table[i].val = UNDEFINED;
		EC_CWF_patch_ram_table[i].val = UNDEFINED;
	}

	mutex_lock(&chip->lock);
	for (i = 0; i < LSC_LINE - 2; i++) {
		if (i % 9 == 0) {
			/* set patch ram address 0x098A */
			u16 addr_offset = i / 9 * 0x0010;
			EC_D65_patch_ram_table[i].op   = WRITE_REG_DATA16;
			EC_D65_patch_ram_table[i].addr = PATCH_RAM_ADDR;
			EC_D65_patch_ram_table[i].val  = PATCH_D65 + addr_offset;

			EC_CWF_patch_ram_table[i].op   = WRITE_REG_DATA16;
			EC_CWF_patch_ram_table[i].addr = PATCH_RAM_ADDR;
			EC_CWF_patch_ram_table[i].val  = PATCH_CWF + addr_offset;

			/* reset mcu_addr */
			mcu_addr = MCU_ADDR;
		} else {
			/*
			 * set patch ram address 0x0990~0x099E
			 * write address for EC ROM read
			 */
			ec_write_word_data_locked(client, SET_RAM_BANK,
						  RAM_BANK);
			ec_write_word_data_locked(client, WRITE_RAM_ADDR,
						  ram_address);
			ret = ec_read_word_data_locked(client, READ_RAM_ADDR);
			h_data = ((s16) ret) << 8;
			ram_address++;

			ec_write_word_data_locked(client, SET_RAM_BANK,
						  RAM_BANK);
			ec_write_word_data_locked(client, WRITE_RAM_ADDR,
						  ram_address);
			ret = ec_read_word_data_locked(client, READ_RAM_ADDR);
			l_data = (s16) ret;
			ram_address++;

			EC_D65_patch_ram_table[i].op   = WRITE_REG_DATA16;
			EC_D65_patch_ram_table[i].addr = mcu_addr;
			EC_D65_patch_ram_table[i].val  = h_data + l_data;

			EC_CWF_patch_ram_table[i].op   = WRITE_REG_DATA16;
			EC_CWF_patch_ram_table[i].addr = mcu_addr;
			EC_CWF_patch_ram_table[i].val  = h_data + l_data;

			mcu_addr += 2;

			if (EC_D65_patch_ram_table[i].val == UNDEFINED) {
				dev_warn(&client->dev,
					"%s: EC might not contain LSC data\n",
					__func__);
				mutex_unlock(&chip->lock);
				return;
			}
		}
	}
	mutex_unlock(&chip->lock);

	/* the rest three entries of patch ram table */
	EC_D65_patch_ram_table[i].op   = WRITE_REG_DATA16;
	EC_D65_patch_ram_table[i].addr = mcu_addr;
	EC_D65_patch_ram_table[i].val  = 0x0000;
	EC_CWF_patch_ram_table[i].op   = WRITE_REG_DATA16;
	EC_CWF_patch_ram_table[i].addr = mcu_addr;
	EC_CWF_patch_ram_table[i].val  = 0x0000;
	i++;
	mcu_addr += 2;

	EC_D65_patch_ram_table[i].op   = WRITE_REG_DATA16;
	EC_D65_patch_ram_table[i].addr = mcu_addr;
	EC_D65_patch_ram_table[i].val  = 0x0000;
	EC_CWF_patch_ram_table[i].op   = WRITE_REG_DATA16;
	EC_CWF_patch_ram_table[i].addr = mcu_addr;
	EC_CWF_patch_ram_table[i].val  = 0x0000;
	i++;

	EC_D65_patch_ram_table[i].op   = SENSOR_5M_TABLE_END;
	EC_D65_patch_ram_table[i].addr = 0x0000;
	EC_D65_patch_ram_table[i].val  = 0x0000;
	EC_CWF_patch_ram_table[i].op   = SENSOR_5M_TABLE_END;
	EC_CWF_patch_ram_table[i].addr = 0x0000;
	EC_CWF_patch_ram_table[i].val  = 0x0000;

	lsc_from_ec_status = 1;
#endif
}

static void ec_poweroff(void)
{
	struct power_supply *ec_supply =
			power_supply_get_by_name(BATTERY_NAME);
	struct ec_info *chip;
	struct i2c_client *client;

	if (ec_supply != NULL) {
		chip = container_of(ec_supply,
				struct ec_info, power_supply);

		client = chip->client;

		dev_info(&client->dev, "poweroff ...\n");

		ec_write_word_data(client, SHUTDOWN, 0);
	}
}

static int panic_event(struct notifier_block *this,
		       unsigned long event, void *ptr)
{
	struct ec_info *chip =
			container_of(this,
				     struct ec_info, panic_notifier);
	chip->is_panic = true;

	return NOTIFY_DONE;
}

static void ec_reboot(char mode, const char *cmd)
{
	struct power_supply *ec_supply =
			power_supply_get_by_name(BATTERY_NAME);
	struct ec_info *chip;
	struct i2c_client *client;

	if (ec_supply != NULL) {
		chip = container_of(ec_supply,
				    struct ec_info, power_supply);

		client = chip->client;

		dev_info(&client->dev, "reboot ...\n");

		if (!chip->is_panic)
			ec_write_word_data(client, COLD_REBOOT, 1);
		else
			ec_write_word_data(client, WARM_REBOOT, 0);
	}

}

void ec_set_audio_table(int table_value)
{
	struct power_supply *ec_supply =
			power_supply_get_by_name(BATTERY_NAME);
	struct ec_info *chip;

	if (ec_supply != NULL) {
		chip = container_of(ec_supply,
				    struct ec_info, power_supply);

		chip->acoustic_table = table_value;

		ec_write_word_data(chip->client, AUDIO_CTRL, table_value);
	}
}

#define TEST_3G_WAKE_BIT(x)	(x & BIT(3))
int is3Gwakeup(void)
{
	struct power_supply *ec_supply =
			power_supply_get_by_name(BATTERY_NAME);
	struct ec_info *chip;
	s32 ret = 0;

	if (ec_supply != NULL) {
		chip = container_of(ec_supply,
				    struct ec_info, power_supply);

		ret = ec_read_word_data(chip->client, MISC_CTRL_RD);
	}

	return TEST_3G_WAKE_BIT(ret);
}

u16 get_board_id(void)
{
	struct power_supply *ec_supply =
			power_supply_get_by_name(BATTERY_NAME);
	struct ec_info *chip;
	static u16 board_id = 0;
	s32 ret;

	if (ec_supply != NULL && !board_id) {
		chip = container_of(ec_supply,
				    struct ec_info, power_supply);

		ret = ec_read_word_data(chip->client, BOARD_ID);

		if (ret >= 0)
			board_id = ret;
	}

	return board_id;
}
EXPORT_SYMBOL(get_board_id);

/*
 * EC battery fuctionality
 */
static int ec_get_battery_presence(struct i2c_client *client,
				   union power_supply_propval *val)
{
	s32 ret;

	ret = ec_read_word_data(client, ec_data[REG_DESIGN_CAPACITY].reg_data);
	if (ret <= 0)
		val->intval = 0;
	else
		val->intval = 1;

	return 0;
}

static int ec_get_battery_health(struct i2c_client *client,
				 union power_supply_propval *val)
{
	s32 ret;

	ret = ec_read_word_data(client, ec_data[REG_HEALTH].reg_data);
	if (ret < 0)
		return ret;

	if (ret > 50)
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	else
		val->intval = POWER_SUPPLY_HEALTH_DEAD;

	return 0;
}

static bool ec_get_battery_capacity(struct ec_info *chip)
{
	int capacity;
	s32 ret;

	ret = ec_read_word_data(chip->client, ec_data[REG_CAPACITY].reg_data);
	if (ret < 0)
		return false;

	/* sbs spec says that this can be >100 %
	* even if max value is 100 % */
	capacity = min(ret, 100);

	if (chip->capacity != capacity) {
		chip->capacity = capacity;
		return true;
	}

	return false;
}

static void ec_get_battery_status(struct ec_info *chip,
				  union power_supply_propval *val)
{
	if (chip->capacity < 100) {
		if (chip->is_supplied)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	else {
		if (chip->is_supplied)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
}

static int ec_get_battery_property(struct i2c_client *client,
				   int reg_offset,
				   union power_supply_propval *val)
{
	s32 ret;

	ret = ec_read_word_data(client, ec_data[reg_offset].reg_data);
	if (ret < 0)
		return ret;

	val->intval = ret;

	return 0;
}

static void  ec_unit_adjustment(struct i2c_client *client,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
#define BASE_UNIT_CONVERSION		1000
#define TEMP_KELVIN_TO_CELSIUS		2731
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = (s16) val->intval;
		break;

	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval *= BASE_UNIT_CONVERSION;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* sbs provides battery temperature in 0.1K
		 * so convert it to 0.1°C
		 */
		val->intval -= TEMP_KELVIN_TO_CELSIUS;
		break;

	default:
		dev_dbg(&client->dev,
			"%s: no need for unit conversion %d\n", __func__, psp);
	}
}

#define SERIAL_PARTS_NB	11
#define SERIAL_STRLEN	SERIAL_PARTS_NB * 2 + 1
static char ec_serial[SERIAL_STRLEN] = "";

static int ec_get_battery_serial_number(struct i2c_client *client,
					union power_supply_propval *val)
{
	struct ec_info *chip = i2c_get_clientdata(client);
	s32 ret;
	int i;

	mutex_lock(&chip->lock);
	for (i = 0; i < SERIAL_PARTS_NB; i++) {
		ret = ec_read_word_data_locked(client,
					ec_data[REG_SERIAL_NUMBER].reg_data);

		snprintf(ec_serial, SERIAL_STRLEN,
			 "%s%s", ec_serial, (char *)&ret);
	}
	mutex_unlock(&chip->lock);

	val->strval = ec_serial;

	return 0;
}

static int ec_get_property_index(struct i2c_client *client,
				 enum power_supply_property psp)
{
	int count;
	for (count = 0; count < ARRAY_SIZE(ec_data); count++)
		if (psp == ec_data[count].psp)
			return count;

	dev_warn(&client->dev,
		 "%s: Invalid Property - %d\n", __func__, psp);

	return -EINVAL;
}

static int ec_get_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	int ret = 0;
	struct ec_info *chip = container_of(psy,
				struct ec_info, power_supply);
	struct i2c_client *client = chip->client;

	switch (psp) {
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		ret = ec_get_battery_serial_number(client, val);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		ret = ec_get_battery_health(client, val);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		ret = ec_get_battery_presence(client, val);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		ec_get_battery_capacity(chip);
		ec_get_battery_status(chip, val);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ec_get_battery_capacity(chip);
		val->intval = chip->capacity;
		break;

	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_TEMP:
		ret = ec_get_property_index(client, psp);
		if (ret < 0)
			break;

		ret = ec_get_battery_property(client, ret, val);
		break;

	default:
		dev_err(&client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	if (!ret) {
		/* Convert units to match requirements for power supply class */
		ec_unit_adjustment(client, psp, val);
	}

	dev_dbg(&client->dev,
		"%s: property = %d, value = %x\n", __func__, psp, val->intval);

	/* battery not present, so return NODATA for properties */
	if (ret)
		return -ENODATA;

	return 0;
}

static void ec_external_power_changed(struct power_supply *psy)
{
	struct ec_info *chip;
	bool supplied_state;

	chip = container_of(psy, struct ec_info, power_supply);

	supplied_state= power_supply_am_i_supplied(psy);

	/* suppress bogus notifications */
	if (chip->is_supplied != supplied_state) {
		chip->is_supplied = supplied_state;
		/* notify android immediately */
		flush_delayed_work(&chip->work);
	}
}

static void ec_delayed_work(struct work_struct *work)
{
	struct ec_info *chip;
	bool capacity_changed;

	chip = container_of(work, struct ec_info, work.work);

	if (chip->poll_disabled)
		return;

	capacity_changed = ec_get_battery_capacity(chip);

	if (capacity_changed) {
		power_supply_changed(&chip->power_supply);

		/* reduce poll time on low battery */
		if (chip->capacity < CAPACITY_LOW) {
			chip->poll_interval -= POLL_TIME_THROTTLE;
			chip->poll_interval =
					max(chip->poll_interval, POLL_TIME_MIN);
		} else if (chip->is_supplied)
			chip->poll_interval = POLL_TIME_CHARGING;
		else
			chip->poll_interval = POLL_TIME_DEFAULT;
        }

	/* send continuous uevent notify to android */
	set_timer_slack(&chip->work.timer, chip->poll_interval * HZ / 4);
	schedule_delayed_work(&chip->work, chip->poll_interval * HZ);
}

#if defined(CONFIG_OF)

#include <linux/of_device.h>

static const struct of_device_id ec_dt_ids[] = {
	{ .compatible = "ec,ec-control" },
	{ }
};
MODULE_DEVICE_TABLE(of, ec_dt_ids);

static struct ec_platform_data *ec_of_populate_pdata(
		struct i2c_client *client)
{
	struct device_node *of_node = client->dev.of_node;
	struct ec_platform_data *pdata = client->dev.platform_data;
	int rc;
	u32 prop;

	/* if platform data is set, honor it */
	if (pdata)
		return pdata;

	/* verify this driver matches this device */
	if (!of_node)
		return NULL;

	pdata = devm_kzalloc(&client->dev, sizeof(struct ec_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		goto of_out;

	rc = of_property_read_u32(of_node, "ec,i2c-retry-count", &prop);

	pdata->i2c_retry_count = rc ? 5 : prop;

of_out:
	return pdata;
}
#else
#define ec_dt_ids NULL
static struct ec_platform_data *ec_of_populate_pdata(
	struct i2c_client *client)
{
	return client->dev.platform_data;
}
#endif

static int ec_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct ec_info *chip;
	struct ec_platform_data *pdata = ec_of_populate_pdata(client);
	int rc;

	if (!pdata)
		return -ENODATA;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->poll_interval = POLL_TIME_DEFAULT;
	chip->capacity = -1;

	chip->power_supply.name = BATTERY_NAME;
	chip->power_supply.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->power_supply.properties = ec_properties;
	chip->power_supply.num_properties = ARRAY_SIZE(ec_properties);
	chip->power_supply.get_property = ec_get_property;
	chip->power_supply.external_power_changed = ec_external_power_changed;

	/* 3g origin state */
	chip->power_state_3g = 2;

	chip->power_white_led.cdev.name = "power-button:white:";
	chip->power_white_led.cdev.dev = &client->dev;
	chip->power_white_led.cdev.brightness_set = ec_pwr_led_change_state;
	chip->power_white_led.cdev.max_brightness = 1;

	chip->power_orange_led.cdev.name = "power-button:orange:";
	chip->power_orange_led.cdev.dev = &client->dev;
	chip->power_orange_led.cdev.brightness_set = ec_pwr_led_change_state;
	chip->power_orange_led.cdev.max_brightness = 1;

	i2c_set_clientdata(client, chip);

	mutex_init(&chip->lock);

	rc = power_supply_register(&client->dev, &chip->power_supply);
	if (rc) {
		dev_err(&client->dev,
			"%s: Failed to register power supply\n", __func__);
		goto probe_fail1;
	}

	/* create dev sysfs for non-battery functions */
	rc = sysfs_create_group(&client->dev.kobj, &ec_control_attr_group);
	if (rc) {
		dev_err(&client->dev,
			"%s: Failed to create ec-control sysfs group\n",
			__func__);
		goto probe_fail2;
	}

	/* create legacy sysfs symlink for acer binary blobs */
	rc = sysfs_create_link(NULL, &client->dev.kobj, "EcControl");
	if (rc) {
		dev_err(&client->dev,
			"%s: Failed to create EcControl sysfs symlink\n",
			__func__);
		goto probe_fail2;
	}

	/* register leds */
	rc = led_classdev_register(&client->dev, &chip->power_white_led.cdev);
	if (rc) {
		dev_err(&client->dev,
			"%s: Failed to register white led\n", __func__);
		goto probe_fail2;
	}

	rc = led_classdev_register(&client->dev, &chip->power_orange_led.cdev);
	if (rc) {
		dev_err(&client->dev,
			"%s: Failed to register orange led\n", __func__);
		goto probe_fail3;
	}

	INIT_WORK(&chip->power_white_led.work, ec_pwr_white_led_work);
	INIT_WORK(&chip->power_orange_led.work, ec_pwr_orange_led_work);

	INIT_DELAYED_WORK(&chip->work, ec_delayed_work);
	INIT_DELAYED_WORK(&chip->lsc_work, ec_lens_correction_preloading);

	/*
	 * enable content adaptive backlight control
	 */
	ec_set_lcd_cabc(client, true);

	/* set pm functions */
	pm_power_off   = ec_poweroff;
	arm_pm_restart = ec_reboot;

	/* register panic notifier */
	chip->panic_notifier.notifier_call = panic_event;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &chip->panic_notifier);

	schedule_work(&chip->work.work);
	schedule_delayed_work(&chip->lsc_work, HZ);

	dev_info(&client->dev, "device registered\n");

	return 0;

probe_fail3:
	led_classdev_unregister(&chip->power_white_led.cdev);
probe_fail2:
	power_supply_unregister(&chip->power_supply);
probe_fail1:
	mutex_destroy(&chip->lock);

	return rc;
}

static int ec_remove(struct i2c_client *client)
{
	struct ec_info *chip = i2c_get_clientdata(client);

	led_classdev_unregister(&chip->power_white_led.cdev);
	led_classdev_unregister(&chip->power_orange_led.cdev);

	chip->poll_disabled = true;
	cancel_delayed_work_sync(&chip->lsc_work);
	cancel_delayed_work_sync(&chip->work);

	power_supply_unregister(&chip->power_supply);

	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &chip->panic_notifier);

	mutex_destroy(&chip->lock);

	return 0;
}

static void ec_shutdown(struct i2c_client *client)
{
	struct ec_info *chip = i2c_get_clientdata(client);

	chip->poll_disabled = true;
	cancel_delayed_work_sync(&chip->lsc_work);
	cancel_delayed_work_sync(&chip->work);
}

#ifdef CONFIG_PM_SLEEP
static int ec_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ec_info *chip = i2c_get_clientdata(client);

	chip->poll_disabled = true;
	cancel_delayed_work_sync(&chip->work);

	return 0;
}

static int ec_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ec_info *chip = i2c_get_clientdata(client);

	chip->poll_disabled = false;
	schedule_delayed_work(&chip->work, HZ);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ec_control_pm_ops, ec_suspend, ec_resume);

static const struct i2c_device_id ec_id[] = {
	{ DRV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ec_id);

static struct i2c_driver ec_control_driver = {
	.probe		= ec_probe,
	.remove		= ec_remove,
	.id_table	= ec_id,
	.shutdown	= ec_shutdown,
	.driver = {
		.name	= DRV_NAME,
		.of_match_table	= of_match_ptr(ec_dt_ids),
		.pm	= &ec_control_pm_ops,
	},
};
module_i2c_driver(ec_control_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dmitry Osipenko");
MODULE_DESCRIPTION("EC control driver");
