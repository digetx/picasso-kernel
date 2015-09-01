/*
 * Lens correction data for Acer A50x tablets
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/workqueue.h>

#include <media/yuv5_sensor.h>
#include <media/lsc_from_ec.h>

#include "ec.h"

#define RAM_BANK	0x0001
#define PATCH_RAM_ADDR	0x098A
#define PATCH_D65	0x1718
#define PATCH_CWF	0x164C
#define MCU_ADDR	0x0990
#define UNDEFINED	0xFFFF

struct sensor_reg_copy EC_D65_patch_ram_table[LSC_LINE + 1];
struct sensor_reg_copy EC_CWF_patch_ram_table[LSC_LINE + 1];
int lsc_from_ec_status = -1;

EC_REG_DATA(SET_RAM_BANK,	0x75,	0);
EC_REG_DATA(WRITE_RAM_ADDR,	0x77,	20);
EC_REG_DATA(READ_RAM_ADDR,	0x78,	20);

static void ec_lens_correction_preloading(struct work_struct *work)
{
	u16 ram_address = 0x0000;
	u16 mcu_addr = MCU_ADDR;
	u16 h_data;
	u16 l_data;
	s32 ret;
	int i;

	if (lsc_from_ec_status < 0)
		return;

	/* Initialize EC_D65_patch_ram_table, EC_CWF_patch_ram_table */
	for (i = 0; i < LSC_LINE + 1; i++) {
		EC_D65_patch_ram_table[i].val = UNDEFINED;
		EC_CWF_patch_ram_table[i].val = UNDEFINED;
	}

	for (i = 0; i < LSC_LINE - 2; i++) {
		if (i % 9 == 0) {
			/* Set patch ram address 0x098A */
			u16 addr_offset = i / 9 * 0x0010;
			EC_D65_patch_ram_table[i].op   = WRITE_REG_DATA16;
			EC_D65_patch_ram_table[i].addr = PATCH_RAM_ADDR;
			EC_D65_patch_ram_table[i].val  = PATCH_D65 + addr_offset;

			EC_CWF_patch_ram_table[i].op   = WRITE_REG_DATA16;
			EC_CWF_patch_ram_table[i].addr = PATCH_RAM_ADDR;
			EC_CWF_patch_ram_table[i].val  = PATCH_CWF + addr_offset;

			/* Reset mcu_addr */
			mcu_addr = MCU_ADDR;
		} else {
			/*
			 * Set patch ram address 0x0990~0x099E
			 * write address for EC ROM read
			 */
			ec_lock();
			ret = ec_write_word_data_locked(SET_RAM_BANK, RAM_BANK);
			if (ret < 0)
				goto rw_err;
			ret = ec_write_word_data_locked(WRITE_RAM_ADDR, ram_address);
			if (ret < 0)
				goto rw_err;
			ret = ec_read_word_data_locked(READ_RAM_ADDR);
			if (ret < 0)
				goto rw_err;
			ec_unlock();

			h_data = ((s16) ret) << 8;
			ram_address++;

			ec_lock();
			ret = ec_write_word_data_locked(SET_RAM_BANK, RAM_BANK);
			if (ret < 0)
				goto rw_err;
			ret = ec_write_word_data_locked(WRITE_RAM_ADDR, ram_address);
			if (ret < 0)
				goto rw_err;
			ret = ec_read_word_data_locked(READ_RAM_ADDR);
			if (ret < 0)
				goto rw_err;
			ec_unlock();

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
				pr_warn("EC might not contain LSC data\n");
				return;
			}
		}
	}

	/* The rest three entries of patch ram table */
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
	return;

rw_err:
	ec_unlock();
}

static DECLARE_DEFERRABLE_WORK(lsc_work, ec_lens_correction_preloading);

static int __init ec_lens_correction(void)
{
	schedule_delayed_work(&lsc_work, HZ * 5);

	return 0;
}
late_initcall(ec_lens_correction);
