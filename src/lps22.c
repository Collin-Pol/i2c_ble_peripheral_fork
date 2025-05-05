/*
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

// power management
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

#include "inc/lps22.h"

LOG_MODULE_REGISTER(lps22);

#define I2C_NODE DT_NODELABEL(lps22)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
const struct device *i2c_device = DEVICE_DT_GET(DT_NODELABEL(i2c22));

bool lps22_init(void)
{
    int err;
    if (!device_is_ready(dev_i2c.bus))
    {
        printk("I2C bus %s is not ready!", dev_i2c.bus->name);
        return false;
    }

    uint8_t cmd[] = {LPS22_RESET_CMD};
    err = i2c_write_dt(&dev_i2c, cmd, sizeof(cmd));
    if (err != 0)
    {
        LOG_WRN("Failed to write to i2c dev addr");
        return false;
    }

    // power management
    err = pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);

    return true;
}

float lps22_read_pressure(void)
{
    int err;
    float pressure;

    err = pm_device_action_run(i2c_device, PM_DEVICE_ACTION_RESUME);

    if (!device_is_ready(dev_i2c.bus))
    {
        LOG_WRN("I2C bus %s is not ready!", dev_i2c.bus->name);
        return 0;
    }

    // start conversion
    uint8_t cmd[] = {LPS22_PRESS_OUT_XL};
    uint8_t buf[3] = {0};
    err = i2c_write_dt(&dev_i2c, cmd, sizeof(cmd));
    if (err != 0)
    {
        LOG_WRN("Failed to write/read to i2c dev");
        return LPS22_ERRCODE;
    }
    k_msleep(20); // conversion time

    err = i2c_read_dt(&dev_i2c, buf, sizeof(buf));
    if (err != 0)
    {
        LOG_WRN("Failed to write/read to i2c dev");
        return LPS22_ERRCODE;
    }

    err = pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);

    pressure = ((buf[2] << 16) | (buf[1] << 8) | buf[0]) / 4096.0;
    return pressure;
}

float lps22_read_temperature(void)
{
    int err;
    float temperature_C;

    err = pm_device_action_run(i2c_device, PM_DEVICE_ACTION_RESUME);

    if (!device_is_ready(dev_i2c.bus))
    {
        printk("I2C bus %s is not ready!", dev_i2c.bus->name);
        return 0;
    }

    // start conversion
    uint8_t cmd[] = {LPS22_TEMP_OUT_L};
    uint8_t buf[2] = {0};
    err = i2c_write_dt(&dev_i2c, cmd, sizeof(cmd));
    if (err != 0)
    {
        LOG_WRN("Failed to write/read to i2c dev");
        return LPS22_ERRCODE;
    }

    k_msleep(20); // conversion time

    err = i2c_read_dt(&dev_i2c, buf, sizeof(buf));
    if (err != 0)
    {
        LOG_WRN("Failed to write/read to i2c dev");
        return LPS22_ERRCODE;
    }

    err = pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);

    // unit conversion
    temperature_C = ((buf[1] << 8) | buf[0]) / 100.0;
    return temperature_C;
}
*/






/*  
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

static void process_sample(const struct device *dev)
{
	static unsigned int obs;
	struct sensor_value pressure, temp;

	if (sensor_sample_fetch(dev) < 0) {
		LOG_INF("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		LOG_INF("Cannot read LPS22HB pressure channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		LOG_INF("Cannot read LPS22HB temperature channel\n");
		return;
	}

	++obs;
	LOG_INF("Observation:%u\n", obs);
    
	// display pressure 
	LOG_INF("Pressure:%.1f kPa\n", sensor_value_to_double(&pressure)); 

	// display temperature 
	LOG_INF("Temperature:%.1f C\n", sensor_value_to_double(&temp));

	// ^ YOU CAN USE THE FUNCTION RETURN VALUES HERE AND PIPE THAT TO YOUR BLE SERVICE! ^

}
*/



#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lps22);

static const struct device *lps22_dev;

bool lps22_init(void)
{
    lps22_dev = DEVICE_DT_GET(DT_NODELABEL(lps22));
    if (!device_is_ready(lps22_dev))
    {
        LOG_ERR("LPS22 device is not ready");
        return false;
    }

    LOG_INF("LPS22 initialized successfully");
    return true;
}

float lps22_read_pressure(void)
{
    struct sensor_value pressure;
    if (sensor_sample_fetch(lps22_dev) < 0)
    {
        LOG_ERR("Failed to fetch sensor data");
        return -1.0;
    }

    if (sensor_channel_get(lps22_dev, SENSOR_CHAN_PRESS, &pressure) < 0)
    {
        LOG_ERR("Failed to get pressure data");
        return -1.0;
    }

    // Convert kPa to hPa
    return sensor_value_to_double(&pressure) * 10.0;
}

float lps22_read_temperature(void)
{
    struct sensor_value temperature;
    if (sensor_sample_fetch(lps22_dev) < 0)
    {
        LOG_ERR("Failed to fetch sensor data");
        return -1.0;
    }

    if (sensor_channel_get(lps22_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature) < 0)
    {
        LOG_ERR("Failed to get temperature data");
        return -1.0;
    }

    return sensor_value_to_double(&temperature);
}
