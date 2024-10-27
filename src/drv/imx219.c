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

/* imx219 - sensor parameter limits */
#define IMX219_MIN_GAIN				0x0000
#define IMX219_MAX_GAIN				0x00e8
#define IMX219_MIN_FRAME_LENGTH			0x0100
#define IMX219_MAX_FRAME_LENGTH			0xffff
#define IMX219_MIN_COARSE_EXPOSURE		0x0001
#define IMX219_MAX_COARSE_DIFF			0x0004

/* imx219 sensor register address */
#define IMX219_MODEL_ID_ADDR_MSB		0x0000
#define IMX219_MODEL_ID_ADDR_LSB		0x0001
#define IMX219_GAIN_ADDR			0x0157
#define IMX219_FRAME_LENGTH_ADDR_MSB		0x0160
#define IMX219_FRAME_LENGTH_ADDR_LSB		0x0161
#define IMX219_COARSE_INTEG_TIME_ADDR_MSB	0x015a
#define IMX219_COARSE_INTEG_TIME_ADDR_LSB	0x015b
#define IMX219_FINE_INTEG_TIME_ADDR_MSB		0x0388
#define IMX219_FINE_INTEG_TIME_ADDR_LSB		0x0389


/* struct array used to match driver with a device tree */
static const struct of_device_id imx219_of_match[] = {
	{ .compatible = "sony,imx219", },
	{ },
};
/* Macro that registers the Device Tree match table (imx219_of_match) with the kernel, 
 * so the driver can be automatically loaded when a compatible device is found 
 */
MODULE_DEVICE_TABLE(of, imx219_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

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

static inline void imx219_get_frame_length_regs(imx219_reg *regs,
	u32 frame_length)
{
	regs->addr = IMX219_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX219_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void imx219_get_coarse_integ_time_regs(imx219_reg *regs,
	u32 coarse_time)
{
	regs->addr = IMX219_COARSE_INTEG_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX219_COARSE_INTEG_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void imx219_get_gain_reg(imx219_reg *reg, u8 gain)
{
	reg->addr = IMX219_GAIN_ADDR;
	reg->val = gain & 0xff;
}

/**
 * @brief Reads an 8-bit register value from the IMX219 sensor.
 *
 * This function reads an 8-bit value from the specified register
 * address of the IMX219 sensor using the regmap API.
 *
 * @param s_data Pointer to the camera_common_data structure, which holds
 *               the regmap object for I2C communication.
 * @param addr The 16-bit register address to read from.
 * @param val Pointer to an 8-bit variable where the read value will be stored.
 *
 * @return 0 on success, or a negative error code if the read operation fails.
 */
static inline int imx219_read_reg(struct camera_common_data *s_data,
                                  u16 addr, u8 *val)
{
    int err = 0;
    u32 reg_val = 0;

    err = regmap_read(s_data->regmap, addr, &reg_val);
    *val = reg_val & 0xff;

    return err;
}

/**
 * @brief Writes a 8-bit value to a register of the IMX219 sensor.
 *
 * This function writes a 16-bit value to the specified register address
 * of the IMX219 sensor and verifies the write by reading back the register
 * value. Any error during the write is logged.
 *
 * @param s_data Pointer to the camera_common_data structure, which holds
 *               the regmap object for I2C communication.
 * @param addr The 16-bit register address to write to.
 * @param val The 8-bit value to write to the register.
 *
 * @return 0 on success, or a negative error code if the write operation fails.
 */
static int imx219_write_reg(struct camera_common_data *s_data,
                            u16 addr, u8 val)
{
    int err;
    struct device *dev = s_data->dev;
    unsigned int result;
    
    regmap_read(s_data->regmap, addr, &result);
    dev_info(dev, "i2c read camera, 0x%x = %x\n", addr, result);
    usleep_range(1000, 1010);
    err = regmap_write(s_data->regmap, addr, val);
    if (err)
        dev_err(dev, "%s: i2c write failed, 0x%x = %x\n", __func__, addr, val);
    usleep_range(1000, 1010);
    regmap_read(s_data->regmap, addr, &result);
    dev_info(dev, "i2c read after write camera, 0x%x = %x\n", addr, result);
    return err;
}

/**
 * @brief Writes a series of register values to the IMX219 sensor.
 *
 * This function iterates over a table of register values and writes them
 * to the IMX219 sensor via I2C. For each register, it retries up to
 * three times if a write fails, and uses a specific delay for wait commands.
 *
 * @param priv Pointer to the IMX219 device structure containing the
 *             necessary I2C client and data.
 * @param table Array of register-value pairs defining the table of
 *              registers to be written to the sensor. The table ends
 *              when the address `IMX219_TABLE_END` is reached.
 * 
 * @return 0 on success, or the last non-zero error code if any write
 *         operation failed.
 *
 * @note If a register in the table has the address `IMX219_TABLE_WAIT_MS`,
 *       the function pauses for the specified amount of milliseconds.
 */
static int imx219_write_table(struct imx219 *priv, const imx219_reg table[])
{
	int ret = 0;
	int i = 0;
	int retry;
	struct camera_common_data *s_data = priv->s_data;
	struct i2c_client *client = priv->i2c_client;
	
	dev_info(&client->dev, "writing registers table");
	
	while (table[i].addr != IMX219_TABLE_END) {
		dev_info(&client->dev, "writing register %x\n", table[i].addr);
		
		for (retry = 0; retry < 3; retry++) {
		    if (IMX219_TABLE_WAIT_MS == table[i].addr) {
			    msleep(table[i].val);
			    break;
		    }
		    
		    ret = imx219_write_reg(s_data, table[i].addr, table[i].val);
		    if (!ret) {
			    dev_info(&client->dev, "SUCCESS writing register %x\n", table[i].addr);
			    break;
		    }
		    usleep_range(1000, 1010);
		}
		i++;
	}
	
	dev_info(&client->dev, "Finished writing registers table");
	return ret;
}

static int imx219_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	/* imx219 does not support group hold */
	return 0;
}

static int imx219_get_fine_integ_time(struct imx219 *priv, u16 *fine_time)
{
	struct camera_common_data *s_data = priv->s_data;
	int err = 0;
	u8 reg_val[2];

	err = imx219_read_reg(s_data, IMX219_FINE_INTEG_TIME_ADDR_MSB,
		&reg_val[0]);
	if (err)
		goto done;

	err = imx219_read_reg(s_data, IMX219_FINE_INTEG_TIME_ADDR_LSB,
		&reg_val[1]);
	if (err)
		goto done;

	*fine_time = (reg_val[0] << 8) | reg_val[1];

done:
	return err;
}

static int imx219_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	int err = 0;
	imx219_reg gain_reg;
	s16 gain;

	if (val < mode->control_properties.min_gain_val)
		val = mode->control_properties.min_gain_val;
	else if (val > mode->control_properties.max_gain_val)
		val = mode->control_properties.max_gain_val;

	/* translate value (from normalized analog gain) */
	gain = (s16)((256 * mode->control_properties.gain_factor) / val);
	gain = 256 - gain;

	if (gain < IMX219_MIN_GAIN)
		gain = IMX219_MAX_GAIN;
	else if (gain > IMX219_MAX_GAIN)
		gain = IMX219_MAX_GAIN;

	dev_dbg(dev, "%s: val: %lld (/%d) [times], gain: %u\n",
		__func__, val, mode->control_properties.gain_factor, gain);

	imx219_get_gain_reg(&gain_reg, (u8)gain);
	err = imx219_write_reg(s_data, gain_reg.addr, gain_reg.val);
	if (err)
		dev_dbg(dev, "%s: gain control error\n", __func__);

	return 0;
}

static int imx219_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx219 *priv = (struct imx219 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	int err = 0;
	imx219_reg fl_regs[2];
	u32 frame_length;
	int i;

	frame_length = (u32)(mode->signal_properties.pixel_clock.val *
		(u64)mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val);

	if (frame_length < IMX219_MIN_FRAME_LENGTH)
		frame_length = IMX219_MIN_FRAME_LENGTH;
	else if (frame_length > IMX219_MAX_FRAME_LENGTH)
		frame_length = IMX219_MAX_FRAME_LENGTH;

	dev_dbg(dev,
		"%s: val: %llde-6 [fps], frame_length: %u [lines]\n",
		__func__, val, frame_length);

	imx219_get_frame_length_regs(fl_regs, frame_length);
	for (i = 0; i < 2; i++) {
		err = imx219_write_reg(s_data, fl_regs[i].addr, fl_regs[i].val);
		if (err) {
			dev_dbg(dev,
				"%s: frame_length control error\n", __func__);
			return err;
		}
	}

	priv->frame_length = frame_length;

	return 0;
}

static int imx219_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx219 *priv = (struct imx219 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	int err = 0;
	imx219_reg ct_regs[2];
	const s32 max_coarse_time = priv->frame_length - IMX219_MAX_COARSE_DIFF;
	const s32 fine_integ_time_factor = priv->fine_integ_time *
		mode->control_properties.exposure_factor /
		mode->signal_properties.pixel_clock.val;
	u32 coarse_time;
	int i;

	coarse_time = (val - fine_integ_time_factor)
		* mode->signal_properties.pixel_clock.val
		/ mode->control_properties.exposure_factor
		/ mode->image_properties.line_length;

	if (coarse_time < IMX219_MIN_COARSE_EXPOSURE)
		coarse_time = IMX219_MIN_COARSE_EXPOSURE;
	else if (coarse_time > max_coarse_time) {
		coarse_time = max_coarse_time;
		dev_dbg(dev,
			"%s: exposure limited by frame_length: %d [lines]\n",
			__func__, max_coarse_time);
	}

	dev_dbg(dev, "%s: val: %lld [us], coarse_time: %d [lines]\n",
		__func__, val, coarse_time);

	imx219_get_coarse_integ_time_regs(ct_regs, coarse_time);

	for (i = 0; i < 2; i++) {
		err = imx219_write_reg(s_data, ct_regs[i].addr, ct_regs[i].val);
		if (err) {
			dev_dbg(dev,
				"%s: coarse_time control error\n", __func__);
			return err;
		}
	}

	return 0;
}

/* Struct for camera control operations (Gain, exposure, frame rate etc.) through the NVIDIA Tegra Camera Framework */
static struct tegracam_ctrl_ops imx219_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx219_set_gain,
	.set_exposure = imx219_set_exposure,
	.set_frame_rate = imx219_set_frame_rate,
	.set_group_hold = imx219_set_group_hold,
};

/*
 * imx219_power_on - Powers on the IMX219 camera sensor
 * @s_data: Pointer to the camera_common_data structure, which holds device-specific information.
 *
 * This function performs the power-on sequence for the IMX219 camera sensor. It enables 
 * the necessary power rails (AVDD, IOVDD, DVDD) and toggles the reset GPIO if required.
 * The function ensures that the power sequence follows the timing requirements specified 
 * in the sensor datasheet (including required delays). It also allows for custom power-on 
 * procedures if a platform-specific power-on callback is provided.
 *
 * Return: 0 if the power-on sequence is successful, negative error code otherwise.
 */
static int imx219_power_on(struct camera_common_data *s_data)
{
    int err = 0;
    struct camera_common_power_rail *pw = s_data->power; // Power rail structure to handle regulators (regulator_put and regulator_get for releasing regulators, regulator_enable  and regulator_disable for power_on and power_down)
    struct camera_common_pdata *pdata = s_data->pdata;   // Platform data (can hold custom power-on callback)
    struct device *dev = s_data->dev;                    // Device structure for logging/debugging

    dev_dbg(dev, "%s: power on\n", __func__);            // Debug log: starting power-on sequence

    // If custom power-on callback is provided, execute it
    if (pdata && pdata->power_on) {
        err = pdata->power_on(pw);                       // Call platform-specific power-on function (power_on(struct camera_common_power_rail) not found in src)
        if (err)
		{
            dev_err(dev, "%s failed.\n", __func__);      // Log error if platform power-on fails
		}
        else
		{
			pw->state = SWITCH_ON;
			dev_info(dev, "power on success");
		}
                                   // Mark power state as ON if successful
        return err;                // Return result of custom power-on
    }


    // Handle reset GPIO if defined (That means GPIOs functions call specific arch functions, the functions called below are stubs)
    if (pw->reset_gpio) {
        if (gpio_cansleep(pw->reset_gpio))               // Check if the GPIO can sleep
            gpio_set_value_cansleep(pw->reset_gpio, 0);  // Set reset GPIO low (active reset)
        else
            gpio_set_value(pw->reset_gpio, 0);           // Set reset GPIO low without sleep
    }

    // Check if any of the main power rails (AVDD, IOVDD, DVDD) are missing
    if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
        goto skip_power_seqn;                            // Skip power sequence if no regulators are defined

    // Wait for a short time before enabling power rails
    usleep_range(10, 20);                                // Small delay before power-on

    // Enable AVDD regulator if present
    if (pw->avdd) {
        err = regulator_enable(pw->avdd);                // Enable AVDD power rail
        if (err)                                         // Handle AVDD enable failure
            goto imx219_avdd_fail;
    }

    // Enable IOVDD regulator if present
    if (pw->iovdd) {
        err = regulator_enable(pw->iovdd);               // Enable IOVDD power rail
        if (err)                                         // Handle IOVDD enable failure
            goto imx219_iovdd_fail;
    }

    // Enable DVDD regulator if present
    if (pw->dvdd) {
        err = regulator_enable(pw->dvdd);                // Enable DVDD power rail
        if (err)                                         // Handle DVDD enable failure
            goto imx219_dvdd_fail;
    }

    usleep_range(10, 20);                                // Small delay after enabling power rails

skip_power_seqn:
    // Release reset GPIO if defined (bring sensor out of reset)
    if (pw->reset_gpio) {
        if (gpio_cansleep(pw->reset_gpio))               // Check if the GPIO can sleep
            gpio_set_value_cansleep(pw->reset_gpio, 1);  // Set reset GPIO high (release reset)
        else
            gpio_set_value(pw->reset_gpio, 1);           // Set reset GPIO high without sleep
    }

    // Wait for t4 + t5 + t9 times as specified in the IMX219 datasheet
    // t4: 200us, t5: 21.2ms, t9: 1.2ms
    usleep_range(23000, 23100);                          // Total delay of ~23ms for proper sensor initialization

    pw->state = SWITCH_ON;                               // Update power state to ON

    return 0;                                            // Return success

// Error handling for regulator failures
imx219_dvdd_fail:
    regulator_disable(pw->iovdd);                        // Disable IOVDD if DVDD fails

imx219_iovdd_fail:
    regulator_disable(pw->avdd);                         // Disable AVDD if IOVDD fails

imx219_avdd_fail:
    dev_err(dev, "%s failed.\n", __func__);              // Log error if power sequence fails

    return -ENODEV;                                      // Return error code indicating device failure
}


/*
 * imx219_power_off - Powers off the IMX219 camera sensor
 * @s_data: Pointer to the camera_common_data structure, which holds device-specific information.
 *
 * This function performs the power-off sequence for the IMX219 camera sensor. It disables 
 * the necessary power rails (AVDD, IOVDD, DVDD) and toggles the reset GPIO if required.
 * The function ensures that the power sequence follows the timing requirements specified 
 * in the sensor datasheet (including required delays). It also allows for custom power-on 
 * procedures if a platform-specific power-on callback is provided.
 *
 * Return: 0 if the power-off sequence is successful, negative error code otherwise.
 */
static int imx219_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio) {
			if (gpio_cansleep(pw->reset_gpio))
				gpio_set_value_cansleep(pw->reset_gpio, 0);
			else
				gpio_set_value(pw->reset_gpio, 0);
		}

		usleep_range(10, 10);

		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->avdd)
			regulator_disable(pw->avdd);
	}

	pw->state = SWITCH_OFF;

	return 0;
}

/*
 * imx219_power_put - Release a reference to a regulator the IMX219 camera sensor
 * @tc_dev: Pointer to the tegracam_device structure, which holds tegra camera device information
 *
 * Return: 0 if the power-put sequence is successful, negative error code otherwise.
 */
static int imx219_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	printk("%s is called", __func__);
	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	return 0;
}

/*
 * imx219_power_get - Get a reference to a regulator the IMX219 camera sensor
 * @tc_dev: Pointer to the tegracam_device structure, which holds tegra camera device information.
 * 
 * Return: 0 if the power-get sequence is successful, negative error code otherwise.
 */
static int imx219_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	int err = 0;


	printk("%s is called", __func__);
	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	/* Sensor MCLK (aka. INCK) */
	if (pdata->mclk_name) {
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(dev, "unable to get clock %s\n",
				pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent)) {
				dev_err(dev, "unable to get parent clock %s",
					pdata->parentclk_name);
			} else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
				&pw->dvdd, pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	/* Reset or ENABLE GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
			__func__, err);
		goto done;
	}

done:
	pw->state = SWITCH_OFF;

	return err;
}

/*
 * imx219_parse_dt - Parse device tree
 * @tc_dev: Pointer to the tegracam_device structure, which holds tegra camera device information.
 * 
 * Return: 0 if the imx219 parse dt sequence is successful, negative error code otherwise.
 */
static struct camera_common_pdata *imx219_parse_dt(
	struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err = 0;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(imx219_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
		sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found\n");
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_dbg(dev, "mclk name not present, "
			"assume sensor driven externally\n");

	err = of_property_read_string(np, "avdd-reg",
		&board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
		&board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
		&board_priv_pdata->regulators.dvdd);
	if (err)
		dev_dbg(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
			"assume sensor powered independently\n");

	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

/*
 * imx219_set_mode - Set the sensor mode
 * @tc_dev: Pointer to the tegracam_device structure, which holds tegra camera device information.
 * 
 * Return: 0 if the imx219 set mode sequence is successful, negative error code otherwise.
 */
static int imx219_set_mode(struct tegracam_device *tc_dev)
{
	struct imx219 *priv = (struct imx219 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;

	int err = 0;

	err = imx219_write_table(priv, mode_table[IMX219_MODE_COMMON]);
	if (err)
		return err;

	err = imx219_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	return 0;
}

/**
 * @brief Starts the video streaming for the IMX219 camera sensor.
 *
 * This function initiates video streaming by sending the appropriate
 * commands to the sensor through the I2C interface.
 *
 * @param tc_dev Pointer to the tegracam device structure, which represents
 *               the camera device in the Tegra camera framework.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int imx219_start_streaming(struct tegracam_device *tc_dev)
{
    struct imx219 *priv = (struct imx219 *)tegracam_get_privdata(tc_dev);

    return imx219_write_table(priv, mode_table[IMX219_START_STREAM]);
}

/**
 * @brief Stops the video streaming for the IMX219 camera sensor.
 *
 * This function stops video streaming by sending the appropriate
 * commands to the sensor through the I2C interface and ensures
 * any necessary delay for a safe shutdown.
 *
 * @param tc_dev Pointer to the tegracam device structure, representing
 *               the camera device in the Tegra camera framework.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int imx219_stop_streaming(struct tegracam_device *tc_dev)
{
    int err;
    struct imx219 *priv = (struct imx219 *)tegracam_get_privdata(tc_dev);

    err = imx219_write_table(priv, mode_table[IMX219_STOP_STREAM]);

    // Delay to ensure safe shutdown of streaming
    usleep_range(50000, 51000);

    return err;
}

/* IMX219 Sensor Operations */
static struct camera_common_sensor_ops imx219_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx219_frmfmt),
	.frmfmt_table = imx219_frmfmt,
	.power_on = imx219_power_on,
	.power_off = imx219_power_off,
	.write_reg = imx219_write_reg,
	.read_reg = imx219_read_reg,
	.parse_dt = imx219_parse_dt,
	.power_get = imx219_power_get,
	.power_put = imx219_power_put,
	.set_mode = imx219_set_mode,
	.start_streaming = imx219_start_streaming,
	.stop_streaming = imx219_stop_streaming,
};

static int imx219_board_setup(struct imx219 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	u8 reg_val[2];
	int err = 0;

	if (pdata->mclk_name) {
		err = camera_common_mclk_enable(s_data);
		if (err) {
			dev_err(dev, "error turning on mclk (%d)\n", err);
			goto done;
		}
	}

	err = imx219_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto err_power_on;
	}

	/* Probe sensor model id registers */
	err = imx219_read_reg(s_data, IMX219_MODEL_ID_ADDR_MSB, &reg_val[0]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
	err = imx219_read_reg(s_data, IMX219_MODEL_ID_ADDR_LSB, &reg_val[1]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
	if (!((reg_val[0] == 0x02) && reg_val[1] == 0x19))
		dev_err(dev, "%s: invalid sensor model id: %x%x\n",
			__func__, reg_val[0], reg_val[1]);

	/* Sensor fine integration time */
	err = imx219_get_fine_integ_time(priv, &priv->fine_integ_time);
	if (err)
		dev_err(dev, "%s: error querying sensor fine integ. time\n",
			__func__);

err_reg_probe:
	imx219_power_off(s_data);

err_power_on:
	if (pdata->mclk_name)
		camera_common_mclk_disable(s_data);

done:
	return err;
}

static int imx219_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx219_subdev_internal_ops = {
	.open = imx219_open,
};

static int imx219_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx219 *priv;
	int err;

	dev_info(dev, "probing imx219 v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx219), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx219", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx219_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx219_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx219_ctrl_ops;
	
	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx219_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_dbg(dev, "detected imx219 sensor\n");

	return 0;
}

static int imx219_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx219 *priv = (struct imx219 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

/**
 * @brief Defines the supported device IDs for the IMX219 I2C driver.
 *
 * The imx219_id table provides a list of compatible devices for the IMX219 driver,
 * allowing the Linux kernel to automatically match this driver with any I2C device
 * named "imx219".
 * 
 * The `MODULE_DEVICE_TABLE` macro exposes this table to the kernel, enabling 
 * automatic loading and binding of this driver with the IMX219 sensor device.
 * 
 * @code
 * static const struct i2c_device_id imx219_id[] = {
 *     { "imx219", 0 },
 *     { }
 * };
 * MODULE_DEVICE_TABLE(i2c, imx219_id);
 * @endcode
 * 
 * @note
 * - `imx219_id` is an array of `i2c_device_id` structs.
 * - Each entry in the table consists of:
 *     - A string representing the device name ("imx219").
 *     - An associated data field, set to `0` in this case.
 *
 * @file imx219.c
 */
static const struct i2c_device_id imx219_id[] = {
    { "imx219", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, imx219_id);

static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.name = "imx219",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx219_of_match),
	},
	.probe = imx219_probe,
	.remove = imx219_remove,
	.id_table = imx219_id,
};
module_i2c_driver(imx219_i2c_driver);


MODULE_DESCRIPTION("Media Controller driver for Sony IMX219");
MODULE_AUTHOR("Mohamed Lamine KARTOBI");
MODULE_LICENSE("GPL v2");
