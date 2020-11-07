
#pragma once


#include "MPU9250.hpp"

#include <lib/drivers/device/spi.h>

#include "MPU9250_AK8963.hpp"


class MPU9250_SPI : public MPU9250, public device::SPI
{
public:
	MPU9250_SPI(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio, bool enable_magnetometer = false);
	~MPU9250_SPI() override;

private:

	const spi_drdy_gpio_t _drdy_gpio;

	// I2C AUX interface (slave 1 - 4)
	AKM_AK8963::MPU9250_AK8963 *_slave_ak8963_magnetometer{nullptr};
	friend class AKM_AK8963::MPU9250_AK8963;

	void I2CSlaveRegisterWrite(uint8_t slave_i2c_addr, uint8_t reg, uint8_t val);
	void I2CSlaveExternalSensorDataEnable(uint8_t slave_i2c_addr, uint8_t reg, uint8_t size);
	bool I2CSlaveExternalSensorDataRead(uint8_t *buffer, uint8_t length);



};
