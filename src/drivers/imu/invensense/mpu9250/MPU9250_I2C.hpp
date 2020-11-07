
#pragma once

#include "MPU9250.hpp"

#include <lib/drivers/device/spi.h>

class MPU9250_I2C : public MPU9250, public device::I2C
{
public:
	MPU9250_I2C(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency);
	~MPU9250_I2C() override;

private:



};
