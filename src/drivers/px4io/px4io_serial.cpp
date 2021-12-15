/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

/**
 * @file uploader.cpp
 * Firmware uploader for PX4IO
 */

#include "PX4IO_serial.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>

#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <sys/stat.h>
#include <nuttx/arch.h>

#include <crc32.h>

IOPacket PX4IO_serial::_io_buffer;

static PX4IO_serial *g_interface;

PX4IO_serial *PX4IO_serial_interface()
{
	if (g_interface == nullptr) {
		g_interface = new PX4IO_serial();

	}

	return g_interface;
}

PX4IO_serial::PX4IO_serial()
{
	g_interface = this;
}

PX4IO_serial::~PX4IO_serial()
{
	perf_free(_pc_txns);
	perf_free(_pc_retries);
	perf_free(_pc_timeouts);
	perf_free(_pc_crcerrs);
	perf_free(_pc_protoerrs);
	perf_free(_pc_uerrs);
	perf_free(_pc_idle);
	perf_free(_pc_badidle);

	if (g_interface == this) {
		g_interface = nullptr;
	}
}

int PX4IO_serial::init()
{
	if (_io_fd < 0) {

		_io_fd = open(PX4IO_SERIAL_DEVICE, O_RDWR | O_NONBLOCK);

		if (_io_fd < 0) {
			PX4_ERR("could not open interface");
			return -errno;
		}

		/* save initial uart configuration to reset after the update */
		struct termios t_original {};
		tcgetattr(_io_fd, &t_original);

		/* adjust line speed to match bootloader */
		struct termios t {};
		tcgetattr(_io_fd, &t);
		cfsetspeed(&t, 1500000);
		tcsetattr(_io_fd, TCSANOW, &t);
	}

	if (_io_fd < 0) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int PX4IO_serial::write_io(unsigned address, void *data, unsigned count)
{
	if (_io_fd < 0) {
		init();
	}

	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	const uint16_t *values = reinterpret_cast<const uint16_t *>(data);

	if (count > PKT_MAX_REGS) {
		return -EINVAL;
	}

	int result = -1;

	for (unsigned retries = 0; retries < 3; retries++) {
		_io_buffer.count_code = count | PKT_CODE_WRITE;
		_io_buffer.page = page;
		_io_buffer.offset = offset;
		memcpy((void *)&_io_buffer.regs[0], (void *)values, (2 * count));

		for (unsigned i = count; i < PKT_MAX_REGS; i++) {
			_io_buffer.regs[i] = 0x55aa;
		}

		_io_buffer.crc = crc_packet(&_io_buffer);

		/* start the transaction and wait for it to complete */
		result = ::write(_io_fd, (void *)&_io_buffer, sizeof(_io_buffer));

		/* successful transaction? */
		if (result == sizeof(_io_buffer)) {
			/* check result in packet */
			if (PKT_CODE(_io_buffer) == PKT_CODE_ERROR) {
				/* IO didn't like it - no point retrying */
				result = -EINVAL;
				perf_count(_pc_protoerrs);
			}

			break;
		}

		perf_count(_pc_retries);
	}

	if (result == OK) {
		result = count;
	}

	return result;
}

int PX4IO_serial::read_io(unsigned address, void *data, unsigned count)
{
	if (_io_fd < 0) {
		init();
	}

	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	uint16_t *values = reinterpret_cast<uint16_t *>(data);

	if (count > PKT_MAX_REGS) {
		return -EINVAL;
	}

	int result;

	_io_buffer.count_code = count | PKT_CODE_READ;
	_io_buffer.page = page;
	_io_buffer.offset = offset;
	_io_buffer.crc = crc_packet(&_io_buffer);

	result = ::write(_io_fd, (void *)&_io_buffer, sizeof(_io_buffer));

	if (result != sizeof(_io_buffer)) {
		PX4_ERR("write failed");
		return -1;
	}

	usleep(1000);

	// write, then read?
	_io_buffer.crc = 0;

	for (unsigned retries = 0; retries < 3; retries++) {

		int bytes_available = 0;
		int err = ::ioctl(_io_fd, FIONREAD, (unsigned long)&bytes_available);

		if (err != 0 || bytes_available >= (int)sizeof(_io_buffer)) {
			/* start the transaction and wait for it to complete */
			result = ::read(_io_fd, (void *)&_io_buffer, sizeof(_io_buffer));

			/* successful transaction? */
			if (result == sizeof(_io_buffer)) {
				if (_io_buffer.crc != crc_packet(&_io_buffer)) {
					PX4_ERR("crc error");
					result = -EIO;
					perf_count(_pc_crcerrs);

				} else if (PKT_CODE(_io_buffer) == PKT_CODE_ERROR) { // IO didn't like it - no point retrying
					PX4_ERR("PKT_CODE_ERROR");
					result = -EINVAL;
					perf_count(_pc_protoerrs);

				} else if (PKT_COUNT(_io_buffer) != count) { // compare the received count with the expected count
					/* IO returned the wrong number of registers - no point retrying */
					PX4_ERR("PKT_COUNT error");
					result = -EIO;
					perf_count(_pc_protoerrs);


				} else {
					/* successful read */
					/* copy back the result */
					memcpy(values, &_io_buffer.regs[0], (2 * count));
				}

				break;
			}
		}

		perf_count(_pc_retries);
		usleep(10000);
	}

	if (result == OK) {
		result = count;
	}

	return result;
}
