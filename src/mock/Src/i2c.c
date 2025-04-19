#include i2c.h

__weak bool I2C_init() {}
__weak bool I2C_register_read(uint8_t bus, uint8_t devid, uint8_t regid, uint8_t* buf, uint8_t buflen) {}
__weak bool I2C_register_write(uint8_t bus, uint8_t devid, uint8_t regid, uint8_t* buf, uint8_t buflen {}
