
#ifndef LPS22_H_
#define LPS22_H_

#define LPS22_I2C_TIMEOUT 100 // 100ms timeout

/*!
 *  I2C ADDRESS/BITS
 */
#define LPS22_DEFAULT_ADDRESS 0x5C

#define LPS22_WHO_AM_I_REG 0x0F /**< Who am I register */
#define LPS22_CTRL_REG1 0x10    /**< Control register 1 */
#define LPS22_CTRL_REG2 0x11    /**< Control register 2 */
#define LPS22_STATUS_REG 0x27   /**< Status register */
#define LPS22_PRESS_OUT_XL 0x28 /**< Pressure output LSB */
#define LPS22_PRESS_OUT_L 0x29  /**< Pressure output middle byte */
#define LPS22_PRESS_OUT_H 0x2A  /**< Pressure output MSB */
#define LPS22_TEMP_OUT_L 0x2B   /**< Temperature output LSB */
#define LPS22_TEMP_OUT_H 0x2C   /**< Temperature output MSB */
#define LPS22_RESET_CMD 0x21    /**< Reset command */

#define LPS22_ERRCODE 0xBADC0DE

struct lps22_reading
{
    float pressure;
    float temperature;
};

/* I2C functions */
bool lps22_init(void);
float lps22_read_pressure(void);
float lps22_read_temperature(void);

#endif

