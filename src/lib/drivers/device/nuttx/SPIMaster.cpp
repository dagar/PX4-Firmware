/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file SPIMaster.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 * @todo A separate bus/device abstraction would allow for mixed interrupt-mode
 * and non-interrupt-mode clients to arbitrate for the bus.  As things stand,
 * a bus shared between clients of both kinds is vulnerable to races between
 * the two, where an interrupt-mode client will ignore the lock held by the
 * non-interrupt-mode client.
 */

#include "SPIMaster.hpp"

#include <px4_platform_common/px4_config.h>

namespace device
{

SPIMaster::SPIMaster(const px4::wq_config_t &config) :
	WorkQueue(config)
{
	if (config == px4::wq_configurations::SPI0) {
		_bus = 0;

	} else if (config == px4::wq_configurations::SPI1) {
		_bus = 1;

	} else if (config == px4::wq_configurations::SPI2) {
		_bus = 2;

	} else if (config == px4::wq_configurations::SPI3) {
		_bus = 3;

	} else if (config == px4::wq_configurations::SPI4) {
		_bus = 4;

	} else if (config == px4::wq_configurations::SPI5) {
		_bus = 5;

	} else if (config == px4::wq_configurations::SPI6) {
		_bus = 6;
	}

	_initialized = Init();
}

SPIMaster::~SPIMaster()
{
	// XXX no way to let go of the bus...
}

bool SPIMaster::Init()
{
	// attach to the spi bus
	if (_dev == nullptr) {
		if (!board_has_bus(BOARD_SPI_BUS, _bus)) {
			return false;
		}

		_dev = px4_spibus_initialize(_bus);
	}

	if (_dev == nullptr) {
		PX4_ERR("failed to init SPI: %d", _bus);
		return false;
	}

	_initialized = true;
	return true;
}

} // namespace device
