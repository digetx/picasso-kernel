/*
 * MFD driver for acer a50x embedded controller
 *
 * based on nvidia's sbs and acer's ec drivers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of_device.h>

#include <linux/mfd/core.h>
#include <linux/mfd/ec-control.h>

#include <asm/system.h>

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

static DEFINE_MUTEX(ec_mutex);

static struct ec_info {
	struct i2c_client		*client;
	int				i2c_retry_count;
	bool				is_panic;
	int				acoustic_table;
	int				power_state_3g;
	int				power_state_gps;
	struct delayed_work		lsc_work;
	struct notifier_block		panic_notifier;
	void (*arm_reboot) (char mode, const char *cmd);
} *ec_chip;

void inline ec_lock(void)
{
	mutex_lock(&ec_mutex);
}
EXPORT_SYMBOL_GPL(ec_lock);

void inline ec_unlock(void)
{
	mutex_unlock(&ec_mutex);
}
EXPORT_SYMBOL_GPL(ec_unlock);

#define I2C_ERR_TIMEOUT	500
int ec_read_word_data_locked(const struct ec_reg_data *reg_data)
{
	struct i2c_client *client = ec_chip->client;
	int retries = ec_chip->i2c_retry_count;
	s32 ret = 0;

	while (retries > 0) {
		ret = i2c_smbus_read_word_data(client, reg_data->addr);
		if (ret >= 0)
			break;
		msleep(I2C_ERR_TIMEOUT);
		retries--;
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c read at address 0x%x failed\n",
			__func__, reg_data->addr);
		return ret;
	}

	msleep(reg_data->timeout);

	return le16_to_cpu(ret);
}
EXPORT_SYMBOL_GPL(ec_read_word_data_locked);

int ec_read_word_data(const struct ec_reg_data *reg_data)
{
	s32 ret;

	ec_lock();
	ret = ec_read_word_data_locked(reg_data);
	ec_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(ec_read_word_data);

int ec_write_word_data_locked(const struct ec_reg_data *reg_data, u16 value)
{
	struct i2c_client *client = ec_chip->client;
	int retries = ec_chip->i2c_retry_count;
	s32 ret = 0;

	while (retries > 0) {
		ret = i2c_smbus_write_word_data(client, reg_data->addr,
						le16_to_cpu(value));
		if (ret >= 0)
			break;
		msleep(I2C_ERR_TIMEOUT);
		retries--;
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c write to address 0x%x failed\n",
			__func__, reg_data->addr);
		return ret;
	}

	msleep(reg_data->timeout);

	return 0;
}
EXPORT_SYMBOL_GPL(ec_write_word_data_locked);

int ec_write_word_data(const struct ec_reg_data *reg_data, u16 value)
{
	s32 ret;

	ec_lock();
	ret = ec_write_word_data_locked(reg_data, value);
	ec_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(ec_write_word_data);

/*
 * Misc ec control
 */
#define PART_SZ	4
static void ec_read_multipart(char *buf, const struct ec_reg_data *reg_data,
			      int parts_nb)
{
	char *write_offset;
	int length, i;
	s32 ret;

	length = parts_nb * PART_SZ;
	write_offset = buf + length;

	ec_lock();
	for (i = 0; i < parts_nb; i++) {
		ret = ec_read_word_data_locked(reg_data);

		write_offset -= PART_SZ;

		snprintf(write_offset, length + 1, "%04x%s",
			 ret, (i == 0) ? "" : write_offset + PART_SZ);
	}
	ec_unlock();
}

static void ec_write_multipart(const char *buf,
			       const struct ec_reg_data *reg_data, int parts_nb)
{
	char part_buf[PART_SZ + 1];
	int read_offset, val, i;
	s32 ret;

	/* don't count trailing "\n" */
	ret = strlen(buf) - 1;

	if (ret != parts_nb * PART_SZ) {
		dev_err(&ec_chip->client->dev,
			"%s: length %d is not equal to required %d\n",
			  __func__, ret, parts_nb * PART_SZ);
		return;
	}

	ec_lock();
	for (i = 0; i < parts_nb; i++) {
		read_offset = (parts_nb - i + 1) * PART_SZ;

		snprintf(part_buf, ARRAY_SIZE(part_buf),
			 "%s", buf + read_offset);

		ret = kstrtoint(part_buf, 16, &val);
		if (ret < 0)
			dev_err(&ec_chip->client->dev,
				"%s: failed to convert hex str: %s\n",
				__func__, part_buf);

		ec_write_word_data_locked(reg_data, val);
	}
	ec_unlock();
}

#define GYRO_GAIN_PARTS_NB	18
static ssize_t gyro_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	ec_read_multipart(buf, GYRO_GAIN_RD, GYRO_GAIN_PARTS_NB);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t gyro_store(struct device *dev,
			  struct device_attribute *attr,
			  const char * buf, size_t n)
{
	ec_write_multipart(buf, GYRO_GAIN_WR, GYRO_GAIN_PARTS_NB);

	return n;
}

static ssize_t pwr_led_on_store(struct device *dev,
				struct device_attribute *attr,
				const char * buf, size_t n)
{
	ec_write_word_data(POWER_LED_ON, 0);

	return n;
}

static ssize_t chrg_led_on_store(struct device *dev,
				 struct device_attribute *attr,
				 const char * buf, size_t n)
{
	ec_write_word_data(CHARGE_LED_ON, 0);

	return n;
}

static ssize_t reset_led_store(struct device *dev,
			       struct device_attribute *attr,
			       const char * buf, size_t n)
{
	ec_write_word_data(RESET_LED, 0);

	return n;
}

static ssize_t leds_off_store(struct device *dev,
			      struct device_attribute *attr,
			      const char * buf, size_t n)
{
	ec_write_word_data(LEDS_OFF, 0);

	return n;
}

static ssize_t android_off_store(struct device *dev,
				 struct device_attribute *attr,
				 const char * buf, size_t n)
{
	ec_write_word_data(ANDROID_LEDS_OFF, 0);

	return n;
}

static ssize_t ls_gain_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	s32 ret = ec_read_word_data(LS_GAIN_RD);

	return sprintf(buf, "%04x\n", ret);
}

static ssize_t ls_gain_store(struct device *dev,
			     struct device_attribute *attr,
			     const char * buf, size_t n)
{
	int val;
	s32 ret;

	ret = kstrtoint(buf, 16, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert hex str: %s\n", __func__, buf);
		return n;
	}

	ec_write_word_data(LS_GAIN_WR, val);

	return n;
}

#define BTMAC_PARTS_NB	3
static ssize_t btmac_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	ec_read_multipart(buf, BTMAC_RD, BTMAC_PARTS_NB);

	return sprintf(buf, "%c%c:%c%c:%c%c:%c%c:%c%c:%c%c\n",
			buf[0], buf[1], buf[2], buf[3], buf[4], 
			buf[5], buf[6], buf[7], buf[8], buf[9], 
			buf[10], buf[11]);
}

static ssize_t btmac_store(struct device *dev,
			   struct device_attribute *attr,
			   const char * buf, size_t n)
{
	ec_write_multipart(buf, BTMAC_WR, BTMAC_PARTS_NB);

	return n;
}

#define WIFIMAC_PARTS_NB	3
static ssize_t wifimac_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	ec_read_multipart(buf, WIFIMAC_RD, WIFIMAC_PARTS_NB);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t wifimac_store(struct device *dev,
			     struct device_attribute *attr,
			     const char * buf, size_t n)
{
	ec_write_multipart(buf, WIFIMAC_WR, WIFIMAC_PARTS_NB);

	return n;
}

static ssize_t device_status_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	s32 ret;
	int i;

	ret = ec_read_word_data(GPS_3G_STATUS_RD);

	for (i = 15; i >= 0; i--)
		buf[i] = ret >> (15 - i) & 0x1 ? '1' : '0';

	return sprintf(buf, "%s\n", buf);
}

static ssize_t device_status_store(struct device *dev,
				   struct device_attribute *attr,
				   const char * buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	ec_write_word_data(GPS_3G_STATUS_WR, val);

	return n;
}

static ssize_t status_3g_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->power_state_3g);
}

static ssize_t status_3g_store(struct device *dev,
			       struct device_attribute *attr,
			       const char * buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	ec_chip->power_state_3g = val;
	ec_write_word_data(POWER_CTRL_3G, val);

	return n;
}

static ssize_t status_gps_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->power_state_gps);
}

static ssize_t status_gps_store(struct device *dev,
				struct device_attribute *attr,
				const char * buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	ec_chip->power_state_gps = !!val;

	if (ec_chip->power_state_gps)
		ec_write_word_data(GPS_POWER_ON, 0);
	else
		ec_write_word_data(GPS_POWER_OFF, 0);

	return n;
}

#define CABC_MASK	BIT(8)
static void ec_set_lcd_cabc(bool enable)
{
	s32 ret;

	ret = ec_read_word_data(MISC_CTRL_RD);
	if (ret < 0)
		return;

	if (enable)
		ret |= CABC_MASK;
	else
		ret &= (~CABC_MASK);

	ec_write_word_data(MISC_CTRL_WR, ret);
}

static ssize_t cabc_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	bool enabled = ec_read_word_data(MISC_CTRL_RD) & CABC_MASK;

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t cabc_store(struct device *dev,
			  struct device_attribute *attr,
			  const char * buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}

	ec_set_lcd_cabc(!!val);

	return n;
}

#define SYSCONF_MASK	0x0000FFFF
static ssize_t sysconf_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	s32 ret = ec_read_word_data(MISC_CTRL_RD) & SYSCONF_MASK;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t sysconf_store(struct device *dev,
			     struct device_attribute *attr,
			     const char * buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}
	val &= SYSCONF_MASK;

	ec_write_word_data(MISC_CTRL_WR, val);

	return n;
}

static ssize_t audioconf_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	s32 ret = ec_read_word_data(AUDIO_CTRL) & SYSCONF_MASK;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t audioconf_store(struct device *dev,
			       struct device_attribute *attr,
			       const char * buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev,
			"%s: failed to convert str: %s\n", __func__, buf);
		return n;
	}
	val &= SYSCONF_MASK;

	ec_write_word_data(AUDIO_CTRL, val);

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
static DEVICE_ATTR(SystemConfig, 0666, sysconf_show, sysconf_store);
static DEVICE_ATTR(MicSwitch, 0666, audioconf_show, audioconf_store);

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
	&dev_attr_SystemConfig.attr,
	&dev_attr_MicSwitch.attr,
	NULL,
};

static struct attribute_group ec_control_attr_group = {
	.attrs = ec_control_attrs,
};

#ifdef CONFIG_LSC_FROM_EC
#include <media/yuv5_sensor.h>
#include <media/lsc_from_ec.h>

struct sensor_reg_copy EC_D65_patch_ram_table[LSC_LINE + 1];
struct sensor_reg_copy EC_CWF_patch_ram_table[LSC_LINE + 1];
int lsc_from_ec_status = 0;

EC_REG_DATA(SET_RAM_BANK,	0x75,	0);
EC_REG_DATA(WRITE_RAM_ADDR,	0x77,	20);
EC_REG_DATA(READ_RAM_ADDR,	0x78,	20);

#define RAM_BANK	0x0001
#define PATCH_RAM_ADDR	0x098A
#define PATCH_D65	0x1718
#define PATCH_CWF	0x164C
#define MCU_ADDR	0x0990
#define UNDEFINED	0xFFFF
static void ec_lens_correction_preloading(struct work_struct *work)
{
	s32 ret;
	int i;
	u16 h_data;
	u16 l_data;
	u16 mcu_addr = MCU_ADDR;
	u16 ram_address = 0x0000;

	/* initialize EC_D65_patch_ram_table, EC_CWF_patch_ram_table */
	for (i = 0; i < LSC_LINE + 1; i++) {
		EC_D65_patch_ram_table[i].val = UNDEFINED;
		EC_CWF_patch_ram_table[i].val = UNDEFINED;
	}

	ec_lock();
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
			ec_write_word_data_locked(SET_RAM_BANK, RAM_BANK);
			ec_write_word_data_locked(WRITE_RAM_ADDR, ram_address);
			ret = ec_read_word_data_locked(READ_RAM_ADDR);
			h_data = ((s16) ret) << 8;
			ram_address++;

			ec_write_word_data_locked(SET_RAM_BANK, RAM_BANK);
			ec_write_word_data_locked(WRITE_RAM_ADDR, ram_address);
			ret = ec_read_word_data_locked(READ_RAM_ADDR);
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
				dev_warn(&ec_chip->client->dev,
					"%s: EC might not contain LSC data\n",
					__func__);
				ec_unlock();
				return;
			}
		}
	}
	ec_unlock();

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
}
#else
static void ec_lens_correction_preloading(struct work_struct *work) {}
#endif

static void ec_poweroff(void)
{
	dev_info(&ec_chip->client->dev, "poweroff ...\n");

	ec_write_word_data(SHUTDOWN, 0);
}

static int panic_event(struct notifier_block *this,
		       unsigned long event, void *ptr)
{
	ec_chip->is_panic = true;

	return NOTIFY_DONE;
}

static void ec_reboot(char mode, const char *cmd)
{
	dev_info(&ec_chip->client->dev, "reboot ...\n");

	if (!ec_chip->is_panic)
		ec_write_word_data(COLD_REBOOT, 1);
	else {
		/* panic may run under spinlock, hence i2c can't be used */
		if (ec_chip->arm_reboot)
			ec_chip->arm_reboot(mode, cmd);
		else
			ec_write_word_data_locked(WARM_REBOOT, 0);
	}
}

void ec_set_audio_table(int table_value)
{
	ec_chip->acoustic_table = table_value;

	ec_write_word_data(AUDIO_CTRL, table_value);
}

#define TEST_3G_WAKE_BIT(x)	(x & BIT(3))
int is3Gwakeup(void)
{
	s32 ret = ec_read_word_data(MISC_CTRL_RD);

	return TEST_3G_WAKE_BIT(ret);
}

int get_board_id(void)
{
	static int board_id = -EINVAL;
	s32 ret;

	if (!ec_chip) {
		pr_warn("%s ec-contol not registered\n", __func__);
		return board_id;
	}

	if (board_id < 0) {
		ret = ec_read_word_data(BOARD_ID);
		if (ret >= 0)
			board_id = ret;
	}

	return board_id;
}
EXPORT_SYMBOL_GPL(get_board_id);

static struct mfd_cell ec_cell[] = {
	{
		.name = "ec-battery",
		.of_compatible = "ec,ec-battery",
	},
	{
		.name = "ec-leds",
		.of_compatible = "ec,ec-leds",
	},
};

static int ec_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	u32 lsc_work_delay = 0;
	int ret;

	ec_chip = devm_kzalloc(&client->dev, sizeof(*ec_chip), GFP_KERNEL);
	if (!ec_chip)
		return -ENOMEM;

	if (of_property_read_u32(np, "ec,i2c-retry-count",
				 &ec_chip->i2c_retry_count))
		ec_chip->i2c_retry_count = 5;

	ec_chip->client = client;

	/* create dev sysfs for non-battery functions */
	ret = sysfs_create_group(&client->dev.kobj, &ec_control_attr_group);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to create sysfs group\n",
			__func__);
		ec_chip = NULL;
		return ret;
	}

	/* create legacy sysfs symlink for acer binary blobs */
	ret = sysfs_create_link(NULL, &client->dev.kobj, "EcControl");
	if (ret) {
		dev_err(&client->dev, "%s: Failed to create sysfs symlink\n",
			__func__);
		sysfs_remove_group(&client->dev.kobj, &ec_control_attr_group);
		ec_chip = NULL;
		return ret;
	}

	/* register battery and leds */
	ret = mfd_add_devices(&client->dev, -1,
			      ec_cell, ARRAY_SIZE(ec_cell),
			      NULL, 0, NULL);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to add devices\n", __func__);
		sysfs_remove_link(NULL, "EcControl");
		sysfs_remove_group(&client->dev.kobj, &ec_control_attr_group);
		ec_chip = NULL;
		return ret;
	}

	/*
	 * enable content adaptive backlight control
	 */
	ec_set_lcd_cabc(true);

	/* set pm functions */
	if (of_property_read_bool(np, "system-power-controller")) {
		pm_power_off = ec_poweroff;

		if (arm_pm_restart)
			ec_chip->arm_reboot = arm_pm_restart;

		arm_pm_restart = ec_reboot;
	}

	/* register panic notifier */
	ec_chip->panic_notifier.notifier_call = panic_event;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &ec_chip->panic_notifier);

	if (IS_ENABLED(CONFIG_LSC_FROM_EC)) {
		INIT_DELAYED_WORK(&ec_chip->lsc_work,
				  ec_lens_correction_preloading);

		of_property_read_u32(np, "ec,lsc-start-delay", &lsc_work_delay);

		schedule_delayed_work(&ec_chip->lsc_work,
				      msecs_to_jiffies(lsc_work_delay));
	}

	dev_dbg(&client->dev, "device registered\n");

	return 0;
}

static int ec_remove(struct i2c_client *client)
{
	mfd_remove_devices(&client->dev);
	cancel_delayed_work_sync(&ec_chip->lsc_work);
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &ec_chip->panic_notifier);
	sysfs_remove_link(NULL, "EcControl");
	sysfs_remove_group(&client->dev.kobj, &ec_control_attr_group);
	ec_chip = NULL;

	return 0;
}

static const struct of_device_id ec_dt_ids[] = {
	{ .compatible = "ec,ec-control" },
	{ }
};
MODULE_DEVICE_TABLE(of, ec_dt_ids);

static const struct i2c_device_id ec_id[] = {
	{ "ec-control", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ec_id);

static struct i2c_driver ec_control_driver = {
	.probe		= ec_probe,
	.remove		= ec_remove,
	.id_table	= ec_id,
	.driver	= {
		.name	= "ec-control",
		.of_match_table	= ec_dt_ids,
	},
};
module_i2c_driver(ec_control_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dmitry Osipenko");
MODULE_DESCRIPTION("EC control driver");
