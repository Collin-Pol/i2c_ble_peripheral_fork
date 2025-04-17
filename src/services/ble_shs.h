// sensor_hub_service.h

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

// Declaration of custom GATT service and characteristics UUIDs
#define SENSOR_HUB_SERVICE_UUID \
    BT_UUID_128_ENCODE(0xa5b46352, 0x9d13, 0x479f, 0x9fcb, 0x3dcdf0a13f4d)

#define PRESSURE_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0x506a55c4, 0xb5e7, 0x46fa, 0x8326, 0x8acaeb1189eb)

void sensor_hub_update_pressure(struct bt_conn *conn, const uint8_t *data, uint16_t len);
