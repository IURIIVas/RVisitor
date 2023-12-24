/// \file ina226.h
/// \brief ina226, current and voltage measurement module header file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef INA226_H_
#define INA226_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "i2c.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define INA226_CAL_VAL                         (1024)
#define INA226_CURRENTLSB_mA_TO_BIT            (0.5f)
#define INA226_CURRENTLSB_BIT_TO_mA            (1 / INA226_CURRENTLSB_mA_TO_BIT)
#define INA226_POWERLSB_INV_BIT_TO_mW          (1 / (INA226_CURRENTLSB_mA_TO_BIT * 25))
#define INA226_I2CTIMEOUT_ticks                (1000)

#define INA226_CONFIG_REG_RW                   (0x00)
#define INA226_SHUNTV_R                        (0x01)
#define INA226_BUSV_R                          (0x02)
#define INA226_POWER_R                         (0x03)
#define INA226_CURRENT_R                       (0x04)
#define INA226_CALIB_RW                        (0x05)
#define INA226_MASK_RW                         (0x06)
#define INA226_ALERTL_RW                       (0x07)
#define INA226_MANUF_ID_R                      (0xFE)
#define INA226_DIE_ID_R                        (0xFF)

#define INA226_MODE_POWER_DOWN                 (0 << 0)
#define INA226_MODE_TRIG_SHUNT_VOLTAGE         (1 << 0)
#define INA226_MODE_TRIG_BUS_VOLTAGE           (2 << 0)
#define INA226_MODE_TRIG_SHUNT_AND_BUS         (3 << 0)
#define INA226_MODE_POWER_DOWN2                (4 << 0)
#define INA226_MODE_CONT_SHUNT_VOLTAGE         (5 << 0)
#define INA226_MODE_CONT_BUS_VOLTAGE           (6 << 0)
#define INA226_MODE_CONT_SHUNT_AND_BUS         (7 << 0)

// Shunt Voltage Conversion Time
#define INA226_VSH_140uS                       (0 << 3)
#define INA226_VSH_204uS                       (1 << 3)
#define INA226_VSH_332uS                       (2 << 3)
#define INA226_VSH_588uS                       (3 << 3)
#define INA226_VSH_1100uS                      (4 << 3)
#define INA226_VSH_2116uS                      (5 << 3)
#define INA226_VSH_4156uS                      (6 << 3)
#define INA226_VSH_8244uS                      (7 << 3)

// Bus Voltage Conversion Time (VBUS CT Bit Settings[6-8])
#define INA226_VBUS_140uS                      (0 << 6)
#define INA226_VBUS_204uS                      (1 << 6)
#define INA226_VBUS_332uS                      (2 << 6)
#define INA226_VBUS_588uS                      (3 << 6)
#define INA226_VBUS_1100uS                     (4 << 6)
#define INA226_VBUS_2116uS                     (5 << 6)
#define INA226_VBUS_4156uS                     (6 << 6)
#define INA226_VBUS_8244uS                     (7 << 6)

// Averaging Mode (AVG Bit Settings[9-11])
#define INA226_AVG_1                           (0 << 9)
#define INA226_AVG_4                           (1 << 9)
#define INA226_AVG_16                          (2 << 9)
#define INA226_AVG_64                          (3 << 9)
#define INA226_AVG_128                         (4 << 9)
#define INA226_AVG_256                         (5 << 9)
#define INA226_AVG_512                         (6 << 9)
#define INA226_AVG_1024                        (7 << 9)

// Reset Bit (RST bit [15])
#define INA226_RESET_ACTIVE                    (1 << 15)
#define INA226_RESET_INACTIVE                  (0 << 15)

// Mask/Enable Register
#define INA226_MER_SOL                         (1 << 15) // Shunt Voltage Over-Voltage
#define INA226_MER_SUL                         (1 << 14) // Shunt Voltage Under-Voltage
#define INA226_MER_BOL                         (1 << 13) // Bus Voltagee Over-Voltage
#define INA226_MER_BUL                         (1 << 12) // Bus Voltage Under-Voltage
#define INA226_MER_POL                         (1 << 11) // Power Over-Limit
#define INA226_MER_CNVR                        (1 << 10) // Conversion Ready
#define INA226_MER_AFF                         (1 << 4)  // Alert Function Flag
#define INA226_MER_CVRF                        (1 << 3)  // Conversion Ready Flag
#define INA226_MER_OVF                         (1 << 2)  // Math Overflow Flag
#define INA226_MER_APOL                        (1 << 1)  // Alert Polarity Bit
#define INA226_MER_LEN                         (1 << 0)  // Alert Latch Enable

#define INA226_MANUF_ID_DEFAULT                (0x5449)
#define INA226_DIE_ID_DEFAULT                  (0x2260)

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------


double ina226_get_busv(i2c_s *I2CHandler, uint16_t dev_addr);
double ina226_get_current(i2c_s *I2CHandler, uint16_t dev_addr);
double ina226_get_power(i2c_s *I2CHandler, uint16_t dev_addr);

uint8_t ina226_set_config(i2c_s *I2CHandler, uint16_t dev_addr, uint16_t config_word);
uint16_t ina226_get_config(i2c_s *I2CHandler, uint16_t dev_addr);
uint16_t ina226_get_shunt_v(i2c_s *I2CHandler, uint16_t dev_addr);
uint16_t ina226_get_busv_reg(i2c_s *I2CHandler, uint16_t dev_addr);
uint16_t ina226_get_power_reg(i2c_s *I2CHandler, uint16_t dev_addr);
uint8_t ina226_set_calibration_reg(i2c_s *I2CHandler, uint16_t dev_addr, uint16_t config_word);
uint16_t ina226_get_calibration_reg(i2c_s *I2CHandler, uint16_t dev_addr);
uint16_t ina226_get_current_reg(i2c_s *I2CHandler, uint16_t dev_addr);
uint16_t ina226_get_manuf_id(i2c_s *I2CHandler, uint16_t dev_addr);
uint16_t ina226_get_die_id(i2c_s *I2CHandler, uint16_t dev_addr);
uint8_t ina226_set_mask_enable(i2c_s *I2CHandler, uint16_t dev_addr, uint16_t config_word);
uint16_t ina226_get_mask_enable(i2c_s *I2CHandler, uint16_t dev_addr);
uint8_t ina226_set_alert_limit(i2c_s *I2CHandler, uint16_t dev_addr, uint16_t config_word);
uint16_t ina226_get_alert_limit(i2c_s *I2CHandler, uint16_t dev_addr);

#endif /* W25Q32_H_ */
