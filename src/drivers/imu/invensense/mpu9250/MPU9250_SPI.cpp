
#include "MPU9250_SPI.hpp"

#include "AKM_AK8963_registers.hpp"

MPU9250_SPI::MPU9250_SPI(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		 spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio, bool enable_magnetometer) :
	MPU9250(rotation),
	SPI(DRV_IMU_DEVTYPE_MPU9250, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio)
{
	if (drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());

	if (enable_magnetometer) {
		_slave_ak8963_magnetometer = new AKM_AK8963::MPU9250_AK8963(*this, rotation);

		if (_slave_ak8963_magnetometer) {
			for (auto &r : _register_cfg) {
				if (r.reg == Register::I2C_SLV4_CTRL) {
					r.set_bits = I2C_SLV4_CTRL_BIT::I2C_MST_DLY;

				} else if (r.reg == Register::I2C_MST_CTRL) {
					r.set_bits = I2C_MST_CTRL_BIT::I2C_MST_P_NSR | I2C_MST_CTRL_BIT::I2C_MST_CLK_400_kHz;

				} else if (r.reg == Register::I2C_MST_DELAY_CTRL) {
					r.set_bits = I2C_MST_DELAY_CTRL_BIT::I2C_SLVX_DLY_EN;
				}
			}
		}
	}
}
