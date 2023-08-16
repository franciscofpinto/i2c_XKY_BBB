/**
 * @file
 * @author pfnf
 * @brief I2C Driver for BeagleBone Black
 */

#ifndef __I2C_DRIVER_H__
#define __I2C_DRIVER_H__


/** @brief I2C instance 0 (shared with power management)                    */
#define  I2C_INSTANCE_0                             (0x0u)
/** @brief I2C instance 1 (free)                                            */
#define  I2C_INSTANCE_1                             (0x1u)

/** @brief System clock fed to I2C module - 48Mhz                           */
#define  I2C_SYSTEM_CLOCK                           (48000000u)
/** @brief Internal clock used by I2C module - 12Mhz                        */
#define  I2C_INTERNAL_CLOCK                         (12000000u)
/** @brief I2C bus speed or frequency - 100Khz                              */
#define  I2C_OUTPUT_CLOCK                           (100000u)


/**
 * @brief Mode
 */
typedef enum {

    I2C_MASTER  = 0x00000400,
    I2C_SLAVE   = 0x00000000

} i2c_mode_t;

typedef enum {

    I2C_SLAVE_ACTION_NONE   = 0x0,
    I2C_SLAVE_ACTION_READ   = 0x1,
    I2C_SLAVE_ACTION_WRITE  = 0x2,

} i2c_slave_action_e;

/**
 * @brief
 */
typedef enum {

    I2C_OK          = 0x0,      //!< I2C_OK
    I2C_EBUSY       = 0x1,      //!< I2C_EBUSY
    I2C_EIO         = 0x2       //!< I2C_EIO

} i2c_rc_e;


#endif /* __I2C_DRIVER_H__ */
