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

#include "UART.hpp"

namespace device
{

UART::UART(const char *path) :
	Device(path),
	_device_path(path)
{
	PX4_DEBUG("name = %s", path);

	// fill in _device_id fields for a UART device
	_device_id.devid_s.bus_type = DeviceBusType_UART;
}

UART::~UART()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int
UART::init()
{
	// Open the actual UART device
	_fd = ::open(_device_path, O_RDWR);

	if (_fd < 0) {
		PX4_ERR("could not open %s", _device_path);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
UART::read(unsigned address, void *data, unsigned count)
{
	int ret = ::read(_fd, data, count);
	return ret;
}

int
UART::write(unsigned address, void *data, unsigned count)
{
	int ret = ::write(_fd, data, count);
	return ret;
}

int
UART::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	int ret = PX4_ERROR;

	if (send_len > 0) {
		ret = ::write(_fd, send, send_len);
	}

	if (recv_len > 0) {
		ret = ::read(_fd, recv, recv_len);
	}

	return ret;
}

} // namespace device
