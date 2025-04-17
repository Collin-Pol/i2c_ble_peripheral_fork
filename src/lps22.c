#include "lps22.h"
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

static const struct device *lps22_dev;

void lps22_init(void) {
    lps22_dev = device_get_binding("LPS22");
    if (!lps22_dev) {
        printk("Failed to get LPS22 device\n");
        return;
    }
}

float lps22_get_pressure(void) {
    struct sensor_value pressure;
    sensor_sample_fetch(lps22_dev);
    sensor_channel_get(lps22_dev, SENSOR_CHAN_PRESS, &pressure);
    return sensor_value_to_double(&pressure);
}
