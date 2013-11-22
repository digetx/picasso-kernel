/*
 * Driver for MT9P111 from Aptina
 *
 * Copyright (C) 2013, Dmitry Osipenko <digetx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <media/soc_camera.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>

/*
 * Sensor core registers
 */
#define MT9P111_CHIP_ID				0x0000
#define MT9P111_PLL_DIVIDERS			0x0010
#define MT9P111_PLL_P_DIVIDERS			0x0012
#define MT9P111_PLL_CONTROL			0x0014
#define MT9P111_STANDBY_CONTROL_AND_STATUS	0x0018
#define MT9P111_RESET				0x001A
#define MT9P111_PAD_SLEW_PAD_CONFIG		0x001E
#define MT9P111_VDD_DIS_COUNTER			0x0022
#define MT9P111_PLL_P4_P5_P6_DIVIDERS		0x002A
#define MT9P111_PLL_P7_DIVIDER			0x002C
#define MT9P111_SENSOR_CLOCK_DIVIDER		0x002E
#define MT9P111_RESET_REGISTER			0x301A
#define MT9P111_MIPI_CONTROL			0x3400
#define MT9P111_TXSS_PARAMETERS			0x3CA0
#define MT9P111_TXC_PARAMETERS			0x3CA2
#define MT9P111_MIPI_STATUS			0x3402
#define MT9P111_LOGICAL_ADDRESS_ACCESS		0x098E

/*
 * Undocumented MCU registers
 */
#define CAM_CORE_Y_ADDR_START			0xC83A
#define CAM_CORE_X_ADDR_START			0xC83C
#define CAM_CORE_Y_ADDR_END			0xC83E
#define CAM_CORE_X_ADDR_END			0xC840
#define CAM_CORE_ROW_SPEED			0xC842
#define CAM_CORE_SKIP_X_CORE			0xC844
#define CAM_CORE_SKIP_Y_CORE			0xC846
#define CAM_CORE_SKIP_X_PIPE			0xC848
#define CAM_CORE_SKIP_Y_PIPE			0xC84A
#define CAM_CORE_POWER_MODE			0xC84C
#define CAM_CORE_BIN_MODE			0xC84E
#define CAM_CORE_ORIENTATION			0xC850
#define CAM_CORE_FINE_CORRECTION		0xC852
#define CAM_CORE_FINE_ITMIN			0xC854
#define CAM_CORE_COARSE_ITMIN			0xC858
#define CAM_CORE_COARSE_ITMAX_MARGIN		0xC85A
#define CAM_CORE_MIN_FRAME_LENGTH_LINES		0xC85C
#define CAM_CORE_MAX_FRAME_LENGTH_LINES		0xC85E
#define CAM_CORE_BASE_FRAME_LENGTH_LINES	0xC860
#define CAM_CORE_MIN_LINE_LENGTH_PCLK		0xC862
#define CAM_CORE_MAX_LINE_LENGTH_PCLK		0xC864
#define CAM_CORE_P4_5_6_DIVIDER			0xC866
#define CAM_CORE_FRAME_LENGTH_LINES		0xC868
#define CAM_CORE_LINE_LENGTH_PCK		0xC86A
#define CAM_CORE_OUTPUT_SIZE_WIDTH		0xC86C
#define CAM_CORE_OUTPUT_SIZE_HEIGHT		0xC86E
#define CAM_CORE_RX_FIFO_TRIGGER_MARK		0xC870
#define CAM_CORE_COARSE_ITMIN			0xC858
#define CAM_OUTPUT_A_IMAGE_WIDTH		0xC8AA
#define CAM_OUTPUT_A_IMAGE_HEIGHT		0xC8AC

#define SEQ_CMD					0x8404
#define FD_MAX_NUM_AUTOCOR_FUNC_VALUES_TO_CHECK	0xA00E
#define FD_50HZ_FLICKER_PERIOD_IN_CONTEXT_A	0xA018
#define FD_60HZ_FLICKER_PERIOD_IN_CONTEXT_A	0xA01C
#define FD_50HZ_FLICKER_PERIOD_IN_CONTEXT_B	0xA01A
#define FD_60HZ_FLICKER_PERIOD_IN_CONTEXT_B	0xA01E
#define FD_MIN_EXPECTED50HZ_FLICKER_PERIOD	0xA010
#define FD_MAX_EXPECTED50HZ_FLICKER_PERIOD	0xA012
#define FD_MIN_EXPECTED60HZ_FLICKER_PERIOD	0xA014
#define FD_MAX_EXPECTED60HZ_FLICKER_PERIOD	0xA016
#define FD_STATUS				0xA000
#define SEQ_STATE_CFG_1_FD			0x8417

#define AE_TRACK_MAX_DGAIN			0xA824
#define AE_RULE_BASE_TARGET			0xA409

#define CAM_OUTPUT_1_MIPICHANNEL		0xC8D4
#define JPEG_JPSS_CTRL_VAR			0xD822
#define SEQ_COMMON_CFG_CONT_TRACK_SPEED		0x843E
#define SEQ_COMMON_CFG_CONT_JUMP_DIV		0x843F

#define DEFAULT_HW_TIMEOUT			100

#define MAX_WIDTH				2592
#define MAX_HEIGHT				1944

enum {
	MODE_VIDEO,
	MODE_PHOTO,
};

static struct camera_mode {
	u16 skip_core_pipe;
	u16 power_mode;
	u16 bin_mode;
	u16 fine_correction;
	u16 fine_itmin;
	u16 frame_length_lines;
	u16 line_length_pck;
} camera_modes[] = {
	[MODE_VIDEO] = { 0x0303, 0x00F6, 0x0001, 0x019C,
			 0x0732, 0x0423, 0x03E8, },

	[MODE_PHOTO] = { 0x0101, 0x00F2, 0x0000, 0x009C,
			 0x034A, 0x07EF, 0x1E48, },
};

struct mt9p111 {
	struct v4l2_subdev subdev;
	struct v4l2_mbus_framefmt format;
	struct v4l2_ctrl_handler ctrls;
	struct camera_mode *mode;
	int is_streaming;
	int pwdn_gpio;
	int reset_gpio;
	u16 width;
	u16 height;
};

static struct mt9p111 *to_mt9p111(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct mt9p111, subdev);
}

static int mt9p111_read(struct i2c_client *client, u16 address)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	address = swab16(address);

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *)&address;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 2;
	msg[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		return ret;

	memcpy(&ret, buf, 2);
	return swab16(ret);
}

static int mt9p111_write(struct i2c_client *client, u16 address, u16 data)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	dev_dbg(&client->dev, "%s address: 0x%04x data: 0x%04x\n",
		__func__, address, data);

	address = swab16(address);
	data = swab16(data);

	memcpy(buf + 0, &address, 2);
	memcpy(buf + 2, &data,    2);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 4;
	msg.buf   = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int mt9p111_write_byte(struct i2c_client *client, u16 address, u8 data)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	dev_dbg(&client->dev, "%s address: 0x%04x data: 0x%02x\n",
		__func__, address, data);

	address = swab16(address);

	memcpy(buf, &address, 2);
	buf[2] = data;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static void mt9t111_set_pll_dividers(struct i2c_client *client, u8 n, u8 m,
				     u8 p1, u8 p2, u8 p3, u8 p4, u8 p5, u8 p6,
				     u8 p7, u8 w)
{
	mt9p111_write(client, MT9P111_PLL_DIVIDERS, (n << 8) | m);

	mt9p111_write(client, MT9P111_PLL_P_DIVIDERS,
		      (w << 12) |  (p3 << 8) | (p2 << 4) | p1);

	mt9p111_write(client, MT9P111_PLL_P4_P5_P6_DIVIDERS,
		      (!!p6 << 14) | (!!p5 << 13) | (!!p4 << 12) |
		      (p6 << 8)    | (p5 << 4)    | p4);

	mt9p111_write(client, MT9P111_PLL_P7_DIVIDER, (!!p7 << 12) | p7);

	mt9p111_write(client, MT9P111_PLL_CONTROL,
		      (0x1 << 0) | /* PLL bypass */
		      (0x1 << 2) | /* PLL lock detector mode selection */
		      (0x2 << 4) | /* Pfd tuning control */
		      (0x1 << 13)); /* Enable hysteresis on clock input pin */

	mt9p111_write(client, MT9P111_PAD_SLEW_PAD_CONFIG,
		      (0x7 << 0) | /* DOUT* pads slew rate */
		      (0x7 << 4) | /* GPIO pads slew rate */
		      (0x7 << 8)); /* PIXCLK pad slew rate */

	/* Control the delay of vdd_dis counter */
	mt9p111_write(client, MT9P111_VDD_DIS_COUNTER, 0x0030);

	mt9p111_write(client, MT9P111_SENSOR_CLOCK_DIVIDER, 0x0000);

	mt9p111_write(client, MT9P111_STANDBY_CONTROL_AND_STATUS,
		      (0x1 << 3) | /* Enable Interrupt request */
		      (0x1 << 14)); /* In Standby state */

	msleep(DEFAULT_HW_TIMEOUT);
}

static int mt9p111_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);

	dev_dbg(&client->dev, "%s %s\n", __func__, on ? "on" : "off");

	if (on) {
		gpio_direction_output(mt9p111->pwdn_gpio, 0);
		msleep(1);
		gpio_direction_output(mt9p111->reset_gpio, 1);
		msleep(20);
	} else {
		gpio_direction_output(mt9p111->reset_gpio, 0);
		msleep(1);
		gpio_direction_output(mt9p111->pwdn_gpio, 1);

		mt9p111->is_streaming = 0;
	}

	return 0;
}

// FIXME: row/column calc, align
static void mt9p111_set_context(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	struct camera_mode *mode = mt9p111->mode;
	u16 out_width, out_height;
	u16 row_startx = 0x001C;
	u16 row_starty = 0x0010;
	u16 row_endx;
	u16 row_endy;

	out_height = MAX_WIDTH * mt9p111->height / mt9p111->width;
	out_width = MAX_WIDTH;

	if (out_height > MAX_HEIGHT) {
		out_width = MAX_HEIGHT * mt9p111->width / mt9p111->height;
		out_height = MAX_HEIGHT;
	}

	row_startx += round_down((MAX_WIDTH - out_width) / 2, 2);
	row_starty += round_down((MAX_HEIGHT - out_height) / 2, 2);
	row_endx = out_width + row_startx + 7 - mode->bin_mode * 2;
	row_endy = out_height + row_starty + 7 - mode->bin_mode * 2;

	out_width = round_up((out_width + 1) / (mode->bin_mode + 1), 8);
	out_height = round_up((out_height + 1) / (mode->bin_mode + 1), 8);

	dev_dbg(&client->dev, "\trow_startx: %d row_starty: %d\n"
			      "\t\t\trow_endx: %d row_endy: %d\n"
			      "\t\t\tout_width: %d out_height: %d\n",
		row_startx, row_starty, row_endx, row_endy,
		out_width, out_height);

	mt9p111_write(client, MT9P111_LOGICAL_ADDRESS_ACCESS, 0x483A);
	mt9p111_write(client, CAM_CORE_OUTPUT_SIZE_WIDTH, out_width);
	mt9p111_write(client, CAM_CORE_OUTPUT_SIZE_HEIGHT, out_height);
	msleep(DEFAULT_HW_TIMEOUT);

	mt9p111_write(client, CAM_CORE_X_ADDR_START, row_startx);
	mt9p111_write(client, CAM_CORE_Y_ADDR_START, row_starty);
	mt9p111_write(client, CAM_CORE_X_ADDR_END, row_endx);
	mt9p111_write(client, CAM_CORE_Y_ADDR_END, row_endy);

	mt9p111_write(client, CAM_CORE_SKIP_X_CORE, mode->skip_core_pipe);
	mt9p111_write(client, CAM_CORE_SKIP_Y_CORE, mode->skip_core_pipe);
	mt9p111_write(client, CAM_CORE_SKIP_X_PIPE, mode->skip_core_pipe);
	mt9p111_write(client, CAM_CORE_SKIP_Y_PIPE, mode->skip_core_pipe);

	mt9p111_write(client, CAM_CORE_POWER_MODE, mode->power_mode);
	mt9p111_write(client, CAM_CORE_BIN_MODE, mode->bin_mode);

	mt9p111_write(client, CAM_CORE_FINE_CORRECTION, mode->fine_correction);
	mt9p111_write(client, CAM_CORE_FINE_ITMIN, mode->fine_itmin);

	mt9p111_write(client, CAM_CORE_MIN_FRAME_LENGTH_LINES,
		      mode->frame_length_lines);

	mt9p111_write(client, CAM_CORE_BASE_FRAME_LENGTH_LINES,
		      mode->frame_length_lines);

	mt9p111_write(client, CAM_CORE_FRAME_LENGTH_LINES,
		      mode->frame_length_lines);

	mt9p111_write(client, CAM_CORE_P4_5_6_DIVIDER, 0x7F7C);

	mt9p111_write(client, CAM_CORE_LINE_LENGTH_PCK, mode->line_length_pck);

	mt9p111_write(client, CAM_CORE_COARSE_ITMIN, 0x0002);
	mt9p111_write(client, CAM_CORE_COARSE_ITMAX_MARGIN, 0x0001);

	mt9p111_write(client, CAM_OUTPUT_A_IMAGE_WIDTH, mt9p111->width);
	mt9p111_write(client, CAM_OUTPUT_A_IMAGE_HEIGHT, mt9p111->height);

	/* test pattern */
	// mt9p111_write(client, 0x3070, 0x0002);
	// mt9p111_write(client, 0x3072, 0x00e2);
	// mt9p111_write(client, 0x3074, 0x00b2);
	// mt9p111_write(client, 0x3076, 0x00a2);
	// mt9p111_write(client, 0x3078, 0x0102);
	// mt9p111_write(client, 0x0100, 0x0003);
}

static void mt9p111_set_flicker(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	mt9p111_write(client, SEQ_CMD, 0x0600);
	mt9p111_write(client, MT9P111_LOGICAL_ADDRESS_ACCESS,
		      FD_MAX_NUM_AUTOCOR_FUNC_VALUES_TO_CHECK);
	mt9p111_write(client, FD_MAX_NUM_AUTOCOR_FUNC_VALUES_TO_CHECK, 0x3235);
	mt9p111_write(client, FD_50HZ_FLICKER_PERIOD_IN_CONTEXT_A, 0x0133);
	mt9p111_write(client, FD_60HZ_FLICKER_PERIOD_IN_CONTEXT_A, 0x00FF);
	mt9p111_write(client, FD_50HZ_FLICKER_PERIOD_IN_CONTEXT_B, 0x0113);
	mt9p111_write(client, FD_60HZ_FLICKER_PERIOD_IN_CONTEXT_B, 0x00E4);
	mt9p111_write(client, FD_MIN_EXPECTED50HZ_FLICKER_PERIOD, 0x0129);
	mt9p111_write(client, FD_MAX_EXPECTED50HZ_FLICKER_PERIOD, 0x013D);
	mt9p111_write(client, FD_MIN_EXPECTED60HZ_FLICKER_PERIOD, 0x00F5);
	mt9p111_write(client, FD_MAX_EXPECTED60HZ_FLICKER_PERIOD, 0x0109);
	mt9p111_write(client, FD_STATUS, 0x1801);
	mt9p111_write(client, SEQ_STATE_CFG_1_FD, 0x0402);
	mt9p111_write(client, SEQ_CMD, 0x0600);
	mt9p111_write(client, MT9P111_STANDBY_CONTROL_AND_STATUS, 0x2008);
	msleep(DEFAULT_HW_TIMEOUT);
}

static void mt9p111_set_auto_exposure(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	mt9p111_write(client, MT9P111_LOGICAL_ADDRESS_ACCESS, 0x281A);
	mt9p111_write(client, AE_TRACK_MAX_DGAIN, 0x0100);
	mt9p111_write(client, 0xA81C, 0x0040);
	mt9p111_write(client, 0xA81E, 0x0180);
	mt9p111_write(client, 0xA820, 0x0200);
	mt9p111_write(client, 0xA81A, 0x0848);
	mt9p111_write_byte(client, AE_RULE_BASE_TARGET, 0x40);
}

static void mt9p111_set_mipi_mode(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	mt9p111_write(client, MT9P111_RESET_REGISTER,
		      (0x1 << 12) | /* Disables SMIA high-speed serialiser and
				       differential output buffers */
		      (0x1 << 6) | /* reset_register_drive_pins */
		      (0x1 << 5) | /* Undocumented */
		      (0x1 << 4) | /* Transition to standby is synchronized to
				      the end of a frame */
		      (0x1 << 3) | /* Enable write for SMIA registers */
		      (0x1 << 2)); /* Place sensor in streaming mode */

	mt9p111_write(client, MT9P111_RESET,
		      (0x1 << 4) | /* GPIO pad input pd */
		      (0x1 << 3) | /* VGPIO pad input pd */
		      (0x1 << 2)); /* mipi transmitter enable */

	mt9p111_write(client, MT9P111_TXSS_PARAMETERS,
		      (0x1 << 0)); /* MIPI/CCP Output */

	mt9p111_write(client, MT9P111_TXC_PARAMETERS,
		      (0x1 << 7) | /* txc_mipi_enable_line_byte_cnt */
		      (0x1 << 1) | /* txc_po_enable_clk_betwn_lines */
		      (0x1 << 0)); /* txc_po_enable_clk_betwn_frames */

	mt9p111_write(client, MT9P111_MIPI_STATUS,
		      (0x1 << 4) | /* MIPI idle */
		      (0x1 << 0)); /* MIPI in standby */

	mt9p111_write(client, MT9P111_MIPI_CONTROL,
		      (0x1e << 10) | /* Data Format: YUV422 8-bit */
		      (0x1 << 9) | /* Enable MIPI Transmit */
		      (0x1 << 5)); /* Enable MIPI packing logic */

	mt9p111_write(client, SEQ_CMD, 0x0600);
	mt9p111_write_byte(client, SEQ_COMMON_CFG_CONT_TRACK_SPEED, 0x20);
	mt9p111_write_byte(client, SEQ_COMMON_CFG_CONT_JUMP_DIV, 0x01);
}

static struct v4l2_subdev_core_ops mt9p111_core_ops = {
	.s_power		= mt9p111_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= mt9p111_get_register,
	.s_register		= mt9p111_set_register,
#endif
};

static int mt9p111_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	int max_tries = 20;

	dev_dbg(&client->dev, "%s %s\n", __func__, enable ? "on" : "off");

	if (!enable || mt9p111->is_streaming)
		return 0;

	mt9t111_set_pll_dividers(client, 3, 64, 0, 7, 0, 12, 6, 15, 0, 0);
	mt9p111_set_context(sd);
	mt9p111_set_flicker(sd);
	mt9p111_set_auto_exposure(sd);
	mt9p111_set_mipi_mode(sd);

	do {
		s32 data = mt9p111_read(client, 0x8405);
		if (IS_ERR_VALUE(data))
			break;
		if (data == 0x0300) {
			mt9p111->is_streaming = 1;
			return 0;
		}
		msleep(DEFAULT_HW_TIMEOUT);

		dev_dbg(&client->dev, "%s polling %d...\n",
			__func__, max_tries);
	} while (max_tries--);

	dev_err(&client->dev, "capture start failed\n");

	return -EIO;
}

static enum v4l2_mbus_pixelcode mt9p111_codes[] = {
	V4L2_MBUS_FMT_UYVY8_2X8,
};

static int mt9p111_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(mt9p111_codes))
		return -EINVAL;

	*code = mt9p111_codes[index];

	return 0;
}

static int mt9p111_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s mf->width: %d mf->height: %d\n",
		__func__, mf->width, mf->height);

	if (mf->width > MAX_WIDTH || mf->height > MAX_HEIGHT)
		return -EINVAL;

	return 0;
}

static int mt9p111_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);

	if ((mf->width * mf->height) > (1280 * 720))
		mt9p111->mode = &camera_modes[MODE_PHOTO];
	else
		mt9p111->mode = &camera_modes[MODE_VIDEO];

	mt9p111->width = mf->width;
	mt9p111->height = mf->height;

	dev_dbg(&client->dev, "%s mf->width: %d mf->height: %d mode: %s\n",
		__func__, mf->width, mf->height,
		mt9p111->mode == &camera_modes[MODE_VIDEO] ? "video" : "photo");

	return 0;
}

static struct v4l2_subdev_video_ops mt9p111_video_ops = {
	.s_stream	= mt9p111_s_stream,
	.s_mbus_fmt	= mt9p111_s_fmt,
	.try_mbus_fmt	= mt9p111_try_fmt,
	.enum_mbus_fmt	= mt9p111_enum_fmt,
};

static int mt9p111_registered(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	s32 data;
	int ret = 0;

	mt9p111_s_power(&mt9p111->subdev, 1);

	/* Read out the chip version register */
	data = mt9p111_read(client, MT9P111_CHIP_ID);
	if (data != 0x2880) {
		dev_err(&client->dev, "MT9P111 not detected, wrong version "
			"0x%04x\n", data);
		ret = -ENODEV;
	} else
		dev_info(&client->dev, "MT9P111 detected at address 0x%02x\n",
			 client->addr);

	mt9p111_s_power(&mt9p111->subdev, 0);

	return ret;
}

static struct v4l2_subdev_ops mt9p111_subdev_ops = {
	.core	= &mt9p111_core_ops,
	.video	= &mt9p111_video_ops,
};

static const struct v4l2_subdev_internal_ops mt9p111_subdev_internal_ops = {
	.registered = mt9p111_registered,
};

static int mt9p111_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct mt9p111 *mt9p111;
	int ret;

	mt9p111 = devm_kzalloc(&client->dev, sizeof(*mt9p111), GFP_KERNEL);
	if (!mt9p111)
		return -ENOMEM;

	mt9p111->pwdn_gpio = of_get_named_gpio(np, "pwdn-gpio", 0);

	ret = devm_gpio_request_one(&client->dev, mt9p111->pwdn_gpio,
				    GPIOF_OUT_INIT_HIGH, "mt9p111_pwdn");
	if (ret) {
		dev_err(&client->dev, "cannot get pwdn gpio\n");
		return ret;
	}

	mt9p111->reset_gpio = of_get_named_gpio(np, "rst-gpio", 0);

	ret = devm_gpio_request_one(&client->dev, mt9p111->reset_gpio,
				    GPIOF_OUT_INIT_LOW, "mt9p111_rst");
	if (ret) {
		dev_err(&client->dev, "cannot get reset gpio\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&mt9p111->subdev, client, &mt9p111_subdev_ops);
	mt9p111->subdev.internal_ops = &mt9p111_subdev_internal_ops;

	return 0;
}

static int mt9p111_remove(struct i2c_client *client)
{
	struct mt9p111 *mt9p111 = to_mt9p111(client);

	v4l2_device_unregister_subdev(&mt9p111->subdev);

	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "mt9p111", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct of_device_id mt9p111_match[] = {
	{ .compatible = "aptina,mt9p111", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt9p111_match);

static struct i2c_driver mt9p111_i2c_driver = {
	.driver = {
		.name = "mt9p111",
		.of_match_table = mt9p111_match,
	},
	.probe = mt9p111_probe,
	.remove = mt9p111_remove,
	.id_table = sensor_id,
};

module_i2c_driver(mt9p111_i2c_driver);

MODULE_DESCRIPTION("Aptina MT9P111 Camera driver");
MODULE_AUTHOR("Dmitry Osipenko <digetx@gmail.com>");
MODULE_LICENSE("GPL v2");
