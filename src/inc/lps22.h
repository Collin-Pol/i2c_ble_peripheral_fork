#ifndef LPS22_H_
#define LPS22_H_

#define LPS22_I2C_TIMEOUT 100 // 100ms timeout

/*!
 *  I2C ADDRESS/BITS
 */
#define LPS22_DEFAULT_ADDRESS 0x5C

#define LPS22_MEAS_PRESSURE_CMD \
    0x21 /**< Measure Pressure Command */
#define LPS22_RESET_CMD 0x11 /**< Reset Command */

#define LPS22_ERRCODE 0xBADC0DE

struct lps22_reading
{
    float pressure;
};

/* i2c dt fxns
i2c_write_dt(...)
i2c_read_dt(...)
i2c_burst_read_dt(...)
i2c_write_read_dt(...)
*/

bool lps22_init(void);
float lps22_get_pressure(void);

//! TODO: should totally implement these and stuff.
void lps22_read_revision(void);
void lps22_read_serial_num(void);

#endif // LPS22_H_
