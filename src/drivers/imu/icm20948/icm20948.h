/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/conversions.h>
#include <lib/systemlib/px4_macros.h>

#include "ICM20948_mag.h"

#if defined(PX4_I2C_OBDEV_ICM20948) || defined(PX4_I2C_BUS_EXPANSION)
#  define USE_I2C
#endif

// Configuration bits ICM20948
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01
#define MPU_SLEEP			0b1000000

#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_INT_BYPASS_EN		0x02

#define BIT_I2C_READ_FLAG           0x80

#define BIT_I2C_SLV0_NACK           0x01
#define BIT_I2C_FIFO_EN             0x40
#define BIT_I2C_MST_EN              0x20
#define BIT_I2C_IF_DIS              0x10
#define BIT_FIFO_RST                0x04
#define BIT_I2C_MST_RST             0x02
#define BIT_SIG_COND_RST            0x01

#define BIT_I2C_SLV0_EN             0x80
#define BIT_I2C_SLV0_BYTE_SW        0x40
#define BIT_I2C_SLV0_REG_DIS        0x20
#define BIT_I2C_SLV0_REG_GRP        0x10

#define BIT_I2C_MST_MULT_MST_EN     0x80
#define BIT_I2C_MST_WAIT_FOR_ES     0x40
#define BIT_I2C_MST_SLV_3_FIFO_EN   0x20
#define BIT_I2C_MST_P_NSR           0x10
#define BITS_I2C_MST_CLOCK_258HZ    0x08
#define BITS_I2C_MST_CLOCK_400HZ    0x0D

#define BIT_I2C_SLV0_DLY_EN         0x01
#define BIT_I2C_SLV1_DLY_EN         0x02
#define BIT_I2C_SLV2_DLY_EN         0x04
#define BIT_I2C_SLV3_DLY_EN         0x08

#define ICM_WHOAMI_20948            0xEA


// ICM20948 registers and data

/*
 * ICM20948 I2C address LSB can be switched by the chip's AD0 pin, thus is device dependent.
 * Noting this down for now. Here GPS uses 0x69. To support a device implementing the second
 * address, probably an additional MPU_DEVICE_TYPE is the way to go.
 */
#define PX4_I2C_EXT_ICM20948_0			0x68
#define PX4_I2C_EXT_ICM20948_1			0x69

/*
 * ICM20948 uses register banks. Register 127 (0x7F) is used to switch between 4 banks.
 * There's room in the upper address byte below the port speed setting to code in the
 * used bank. This is a bit more efficient, already in use for the speed setting and more
 * in one place than a solution with a lookup table for address/bank pairs.
 */

#define BANK0	0x000
#define BANK1	0x100
#define BANK2	0x200
#define BANK3	0x300

#define BANK_REG_MASK	0x0300
#define REG_BANK(r) 			(((r) & BANK_REG_MASK)>>8)
#define REG_ADDRESS(r)			((r) & 0xFF)

#define ICMREG_20948_BANK_SEL 0x7F

#define	ICMREG_20948_WHOAMI				(0x00 | BANK0)
#define ICMREG_20948_USER_CTRL				(0x03 | BANK0)
#define ICMREG_20948_PWR_MGMT_1				(0x06 | BANK0)
#define ICMREG_20948_PWR_MGMT_2				(0x07 | BANK0)
#define ICMREG_20948_INT_PIN_CFG			(0x0F | BANK0)
#define ICMREG_20948_INT_ENABLE				(0x10 | BANK0)
#define ICMREG_20948_INT_ENABLE_1			(0x11 | BANK0)
#define ICMREG_20948_ACCEL_XOUT_H			(0x2D | BANK0)
#define ICMREG_20948_INT_ENABLE_2			(0x12 | BANK0)
#define ICMREG_20948_INT_ENABLE_3			(0x13 | BANK0)
#define ICMREG_20948_EXT_SLV_SENS_DATA_00	        (0x3B | BANK0)

#define ICMREG_20948_GYRO_SMPLRT_DIV		        (0x00 | BANK2)
#define ICMREG_20948_GYRO_CONFIG_1			(0x01 | BANK2)
#define ICMREG_20948_GYRO_CONFIG_2			(0x02 | BANK2)
#define ICMREG_20948_ACCEL_SMPLRT_DIV_1		        (0x10 | BANK2)
#define ICMREG_20948_ACCEL_SMPLRT_DIV_2		        (0x11 | BANK2)
#define ICMREG_20948_ACCEL_CONFIG			(0x14 | BANK2)
#define ICMREG_20948_ACCEL_CONFIG_2			(0x15 | BANK2)

#define ICMREG_20948_I2C_MST_CTRL			(0x01 | BANK3)
#define ICMREG_20948_I2C_SLV0_ADDR			(0x03 | BANK3)
#define ICMREG_20948_I2C_SLV0_REG			(0x04 | BANK3)
#define ICMREG_20948_I2C_SLV0_CTRL			(0x05 | BANK3)
#define ICMREG_20948_I2C_SLV0_DO			(0x06 | BANK3)



/*
* ICM20948 register bits
* Most of the regiser set values from ICM20948 have the same
* meaning on ICM20948. The exceptions and values not already
* defined for ICM20948 are defined below
*/
#define ICM_BIT_PWR_MGMT_1_ENABLE       	0x00
#define ICM_BIT_USER_CTRL_I2C_MST_DISABLE   0x00

#define ICM_BITS_GYRO_DLPF_CFG_119HZ		0b10000
#define ICM_BITS_GYRO_FS_SEL_2000DPS		0b110

#define ICM_BITS_ACCEL_DLPF_CFG_111HZ		0b10000
#define ICM_BITS_ACCEL_FS_SEL_16G		0b110

#define ICM_BITS_DEC3_CFG_32			0b11

#define ICM_BITS_I2C_MST_CLOCK_370KHZ    	0x00
#define ICM_BITS_I2C_MST_CLOCK_400HZ    	0x07	// recommended by datasheet for 400kHz target clock


#pragma pack(push, 1)
/**
 * Report conversation within the mpu, including command byte and
 * interrupt status.
 */
struct ICMReport {
	uint8_t cmd;
	uint8_t ACCEL_XOUT_H;
	uint8_t ACCEL_XOUT_L;
	uint8_t ACCEL_YOUT_H;
	uint8_t ACCEL_YOUT_L;
	uint8_t ACCEL_ZOUT_H;
	uint8_t ACCEL_ZOUT_L;
	uint8_t GYRO_XOUT_H;
	uint8_t GYRO_XOUT_L;
	uint8_t GYRO_YOUT_H;
	uint8_t GYRO_YOUT_L;
	uint8_t GYRO_ZOUT_H;
	uint8_t GYRO_ZOUT_L;
};
#pragma pack(pop)

/* interface factories */
extern device::Device *ICM20948_SPI_interface(int bus, uint32_t cs);
extern device::Device *ICM20948_I2C_interface(int bus, uint32_t address);
extern int ICM20948_probe(device::Device *dev);

typedef device::Device *(*ICM20948_constructor)(int, uint32_t);

class ICM20948_mag;

class ICM20948 : public px4::ScheduledWorkItem
{
public:
	ICM20948(device::Device *interface, device::Device *mag_interface, enum Rotation rotation);
	virtual ~ICM20948();

	virtual int		init();
	uint8_t			get_whoami() { return _whoami; }

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	device::Device *_interface;
	uint8_t			_whoami{0};	/** whoami result */

	virtual int		probe();

	friend class ICM20948_mag;

	void Run() override;

private:

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	ICM20948_mag		_mag;
	uint8_t 		_selected_bank{0};		/* Remember selected memory bank to avoid polling / setting on each read/write */

	unsigned		_call_interval{1000};

	unsigned		_dlpf_freq{0};
	unsigned		_dlpf_freq_icm_gyro{0};
	unsigned		_dlpf_freq_icm_accel{0};

	unsigned		_sample_rate{1000};

	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_duplicates;

	uint8_t			_register_wait{0};
	uint64_t		_reset_wait{0};

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset

	static constexpr int ICM20948_NUM_CHECKED_REGISTERS{4};
	/*
	list of registers that will be checked in check_registers(). Note
	that MPUREG_PRODUCT_ID must be first in the list.
	*/
	static constexpr uint16_t _checked_registers[ICM20948_NUM_CHECKED_REGISTERS] {
		ICMREG_20948_ACCEL_CONFIG,
		ICMREG_20948_ACCEL_CONFIG_2,
		ICMREG_20948_GYRO_CONFIG_1,
		ICMREG_20948_GYRO_CONFIG_2,
	};

	uint8_t					_checked_values[ICM20948_NUM_CHECKED_REGISTERS] {};
	uint8_t 				_checked_next{0};


	// last temperature reading for print_info()
	float			_last_temperature{0.0f};

	bool check_null_data(uint16_t *data, uint8_t size);
	bool check_duplicate(uint8_t *accel_data);

	// keep last accel reading for duplicate detection
	uint8_t			_last_accel_data[6] {};
	bool			_got_duplicate{false};

	void			start();
	void			stop();
	int			reset();

	/**
	 * Resets the main chip (excluding the magnetometer if any).
	 */
	int			reset_mpu();

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	/**
	 * Select a register bank in ICM20948
	 *
	 * Only actually switches if the remembered bank is different from the
	 * requested one
	 *
	 * @param		The index of the register bank to switch to (0-3)
	 * @return		Error code
	 */
	int			select_register_bank(uint8_t bank);

	/**
	 * Read a register from the mpu
	 *
	 * @param		The register to read.
	* @param       The bus speed to read with.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Read a register range from the mpu
	 *
	 * @param       The start address to read from.
	 * @param       The bus speed to read with.
	 * @param       The address of the target data buffer.
	 * @param       The count of bytes to be read.
	 * @return      The value that was read.
	 */
	uint8_t                 read_reg_range(unsigned start_reg, uint8_t *buf, uint8_t count);

	/**
	 * Write a register in the mpu
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the mpu, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a checked register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_checked_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Swap a 16-bit value read from the mpu to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return _interface->external(); }

	/*
	  check that key registers still have the right value
	 */
	void check_registers();
};
