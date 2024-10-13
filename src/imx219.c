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

/**
 * @brief Writes an 8-bit value to a specified register address of the IMX219 sensor.
 *
 * This function writes an 8-bit value to a specific register address of the IMX219 sensor using
 * the I2C interface. It retrieves the private data of the device and utilizes `regmap_write` to
 * perform the write operation.
 *
 * @param dev The device structure pointer representing the IMX219 sensor.
 * @param addr The 16-bit register address where the data should be written.
 * @param val The 8-bit value to write to the specified register.
 * @return 0 on success, or a negative error code on failure.
 */
static inline int imx219_write_reg(struct device *dev, u16 addr, u8 val)
{
    struct imx219 *priv = dev_get_drvdata(dev); // Get private data
    int err = 0;

    dev_info(dev, "%s: i2c write, 0x%x = %x\n", __func__, addr, val);
    err = regmap_write(priv->regmap, addr, val);
    if (err) {
        dev_err(dev, "%s: i2c write, 0x%x = %x failed\n", __func__, addr, val);
    }

    return err;
}

/**
 * @brief Reads an 8-bit value from a specified register address of the IMX219 sensor.
 *
 * This function reads an 8-bit value from a specific register address of the IMX219 sensor using
 * the I2C interface. It retrieves the private data of the device and utilizes `regmap_read` to
 * perform the read operation. The result is stored in the provided `val` pointer.
 *
 * @param dev The device structure pointer representing the IMX219 sensor.
 * @param addr The 16-bit register address from where the data should be read.
 * @param val A pointer to a variable where the 8-bit value read from the register will be stored.
 * @return 0 on success, or a negative error code on failure.
 */
static inline int imx219_read_reg(struct device *dev, u16 addr, u8 *val)
{
    struct imx219 *priv = dev_get_drvdata(dev); // Get private data
    int err = 0;
    u32 reg_val = 0; // reg_val is used to store the value read from the register address in 32 bits

    // Read the register value and store it in reg_val
    err = regmap_read(priv->regmap, addr, &reg_val);
    if (err) {
        dev_err(dev, "%s: i2c read, 0x%x failed\n", __func__, addr);
        return err;
    }
    dev_info(dev, "%s: i2c read, 0x%x = %x\n", __func__, addr, reg_val);

    // Store only the least significant 8 bits in val
    *val = reg_val & 0xff;

    return 0;
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
    struct camera_common_power_rail *pw = s_data->power; // Power rail structure (holds regulator handles)
    struct camera_common_pdata *pdata = s_data->pdata;   // Platform data (can hold custom power-on callback)
    struct device *dev = s_data->dev;                    // Device structure for logging/debugging

    dev_dbg(dev, "%s: power on\n", __func__);            // Debug log: starting power-on sequence

    // If custom power-on callback is provided, execute it
    if (pdata && pdata->power_on) {
        err = pdata->power_on(pw);                       // Call platform-specific power-on function
        if (err)
            dev_err(dev, "%s failed.\n", __func__);      // Log error if platform power-on fails
        else
            pw->state = SWITCH_ON;                       // Mark power state as ON if successful
        return err;                                      // Return result of custom power-on
    }

    // Handle reset GPIO if defined
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
