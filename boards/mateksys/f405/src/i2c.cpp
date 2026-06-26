/**
 * MatekSys F405 I2C configuration
 * I2C1: SPL06-001 Barometer
 */

#include <px4_arch/i2c_hw_description.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusExternal(1),
};
