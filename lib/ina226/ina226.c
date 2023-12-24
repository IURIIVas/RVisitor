/// \file ina226.h
/// \brief ina226, current and voltage measurement module source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "ina226.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

double ina226_get_busy(i2c_s *i2c, uint16_t dev_addr) {
    return (ina226_get_busv_reg(i2c, dev_addr));
}

double ina226_get_current(i2c_s *i2c, uint16_t dev_addr) {
    return (ina226_get_current_reg(i2c, dev_addr) * INA226_CURRENTLSB_BIT_TO_mA);
}

double ina226_get_power(i2c_s *i2c, uint16_t dev_addr) {
    return (ina226_get_power_reg(i2c, dev_addr) * INA226_POWERLSB_INV_BIT_TO_mW);
}

uint8_t ina226_set_config(i2c_s *i2c, uint16_t dev_addr, uint16_t config_word) {
    uint8_t send[3];
    send[0] = INA226_CONFIG_REG_RW;
    send[1] = (config_word & 0xFF00) >> 8;
    send[2] = (config_word & 0x00FF);
    return i2c_master_trancieve(i2c, dev_addr, send, 3, INA226_I2CTIMEOUT_ticks);
}

uint16_t ina226_get_config(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_CONFIG_REG_RW};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint16_t ina226_get_shunt_v(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_SHUNTV_R};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint16_t ina226_get_busv_reg(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_BUSV_R};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint8_t ina226_set_calibration_reg(i2c_s *i2c, uint16_t dev_addr, uint16_t config_word) {
    uint8_t send[3];
    send[0] = INA226_CALIB_RW;
    send[1] = (config_word & 0xFF00) >> 8;
    send[2] = (config_word & 0x00FF);
    return i2c_master_trancieve(i2c, dev_addr, send, 3, INA226_I2CTIMEOUT_ticks);
}

uint16_t ina226_get_calibration_reg(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_CALIB_RW};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint16_t ina226_get_power_reg(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_POWER_R};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint16_t ina226_get_current_reg(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_CURRENT_R};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint16_t ina226_get_manuf_id(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_MANUF_ID_R};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint16_t ina226_get_die_id(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_DIE_ID_R};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint8_t ina226_set_mask_enable(i2c_s *i2c, uint16_t dev_addr, uint16_t config_word) {
    uint8_t send[3];
    send[0] = INA226_MASK_RW;
    send[1] = (config_word & 0xFF00) >> 8;
    send[2] = (config_word & 0x00FF);
    return i2c_master_trancieve(i2c, dev_addr, send, 3, INA226_I2CTIMEOUT_ticks);
}

uint16_t ina226_get_mask_enable(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_MASK_RW};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}

uint8_t ina226_set_alert_limit(i2c_s *i2c, uint16_t dev_addr, uint16_t config_word) {
    uint8_t send[3];
    send[0] = INA226_ALERTL_RW;
    send[1] = (config_word & 0xFF00) >> 8;
    send[2] = (config_word & 0x00FF);
    return i2c_master_trancieve(i2c, dev_addr, send, 3, INA226_I2CTIMEOUT_ticks);
}

uint16_t ina226_get_alert_limit(i2c_s *i2c, uint16_t dev_addr) {
    uint8_t send[1] = {INA226_ALERTL_RW};
    uint8_t received[2];
    i2c_master_trancieve(i2c, dev_addr, send, 1, INA226_I2CTIMEOUT_ticks);
    if (i2c_master_receive(i2c, dev_addr, received, 2, INA226_I2CTIMEOUT_ticks) != OK) return 0xFF;
    else return ((uint16_t)received[0] << 8 | received[1]);
}
