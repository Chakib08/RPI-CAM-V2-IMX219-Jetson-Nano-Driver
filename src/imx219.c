/*
 * imx219.c - imx219 sensor driver
 *
 * Copyright (c) 2024, Mohamed Lamine KARTOBI.  All rights reserved.
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Linux headers */
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

/* NVIDIA Tegra camera headers */
#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include "../platform/tegra/camera/camera_gpio.h"


/* Specific headers for I2C imx219 registers*/
#include <media/imx219.h>
#include "imx219_mode_tbls.h"


/* struct array used to match driver with a device tree */
static const struct of_device_id imx219_of_match[] = {
	{ .compatible = "sony,imx219", },
	{ },
};
/* Macro that registers the Device Tree match table (imx219_of_match) with the kernel, 
 * so the driver can be automatically loaded when a compatible device is found 
 */
MODULE_DEVICE_TABLE(of, imx219_of_match);

/* IMX219 image sensor struct */
struct imx219
{
	struct i2c_client *i2c_client; 			/* Data used to manage and communicate with the i2c device (slave) on the i2c bus */
	struct v4l2_subdev *subdev; 			/* Data used to handle a subdevice in a camera module (image sensor imx219 in this driver) */
	struct regmap *regmap;					/* Data used to read and write registers to the i2c slave device */
	u16				fine_integ_time;		/* Fine integration time */
	u32				frame_length;			/* Frame lenght */
	struct camera_common_data	*s_data; 	/* Nvidia specific struct to manager camera common data (sensor properties and mode DT params)*/
	struct tegracam_device		*tc_dev;	/* Nvidia specific struct used to register a v4l2 camera device */
};

/* Regmap config struct to specify how the register map will interfact with the i2c device */
static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline int imx219_write_reg(struct device *dev, u16 addr, u8 val)
{
	struct imx219 *priv = dev_get_drvdata(dev); // Get private data
	int err = 0;

	dev_info(dev, "%s:i2c write, 0x%x = %x\n", __func__, addr, val);
	err = regmap_write(priv->regmap, addr, val);
	if(err) {
		dev_err(dev, "%s:i2c write, 0x%x = %x failed\n", __func__, addr, val);
	}

	return err;
}

static inline int imx219_read_reg(struct device *dev, u16 addr, u8 *val)
{
	struct imx219 *priv = dev_get_drvdata(dev); // Get private data
	int err = 0;
	// reg_val is used to store value read from the register address in 32 bits
	u32 reg_val = 0;

	// Read the register value and store it in reg_val
	err = regmap_read(priv->regmap, addr, &reg_val);
	if(err) {
		dev_err(dev, "%s: i2c read, 0x%x failed\n", __func__, addr);
		return err;
	}
	dev_info(dev, "%s:i2c read, 0x%x = %x\n", __func__, addr, reg_val);

	// Store only the least 8 significant bits in fact that the i2c sensor supports 8 bits values
	*val = reg_val & 0xff;

	return 0;
}