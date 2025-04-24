
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
