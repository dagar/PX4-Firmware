/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "NRA15.hpp"

#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>

static const uint16_t kTotalSizeBytes = 14;

NRA15::NRA15(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_NRA15;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
}

NRA15::~NRA15()
{
	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
NRA15::init()
{
	int32_t hw_model = 1; // only one model so far...

	switch (hw_model) {
	case 1: // NRA15 (12m, 100 Hz)
		// Note:
		// Sensor specification shows 0.3m as minimum, but in practice
		// 0.3 is too close to minimum so chattering of invalid sensor decision
		// is happening sometimes. this cause EKF to believe inconsistent range readings.
		// So we set 0.4 as valid minimum.
		_px4_rangefinder.set_min_distance(0.4f);
		_px4_rangefinder.set_max_distance(12.0f);
		_px4_rangefinder.set_fov(math::radians(1.15f));

		break;

	default:
		PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
		return -1;
	}

	// status
	int ret = 0;

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		// baudrate 115200, 8 bits, no parity, 1 stop bit
		unsigned speed = B115200;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		ret = tcsetattr(_fd, TCSANOW, &uart_config);

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while (0);

	// close the fd
	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

void NRA15::start()
{
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(20_ms);
}

void NRA15::stop()
{
	ScheduleClear();
}

void NRA15::Run()
{
	if (_fd < 0) {
		// open fd
		PX4_INFO("Opening port %s", _port);
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
		tcflush(_fd, TCIFLUSH);
	}



	// At each data cycle of NRA15 (20ms), the message for NRA15 systemstatusandtarget output status would be output

	// A complete data message of UART-TTL communication is 14 bytes.

	// Each byte of data is unsigned 8bit.

	// 0: Start Sequence (2 bytes)
	// 1:
	// 2: Message ID (2 bytes)
	// 3:
	// 4: Data Payload (8 x Uint8)
	// ...
	// 12: End Sequence (2 x Uint8)
	// 13:

	// Start Sequence is a constant value 0xAAAA
	// End Sequence is set to 0x5555

	// Message ID:
	// 0x200 Sensor Configuration
	// 0x400 Sensor Back
	// 0x60A Sensor Status
	// 0x70B Target Status
	// 0x70C Target Info




	// poll with 8ms timeout
	pollfd fds[1] {};
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
	int poll_ret = ::poll(fds, 1, 300);

	if (poll_ret == 0) {
		// timeout: this means none of our providers is giving us data
		perf_count(_poll_timeout_perf);
		ScheduleNow();
		return;

	} else if (poll_ret < 0) {
		// poll failed
		PX4_ERR("poll error: %d", poll_ret);
		perf_count(_poll_error_perf);
		tcflush(_fd, TCIFLUSH);
		ScheduleDelayed(5_ms);
		return;
	}


	// read data
	if (fds[0].revents & POLLIN) {
		// POLLIN







	} else {
		ScheduleDelayed(10_ms);
		return;
	}



	// clear buffer and reset if there's not enough room
	if (_linebuf_index >= sizeof(_linebuf) - 128) {
		// clear line buffer
		memset(_linebuf, 0, sizeof(_linebuf));
		_linebuf_index = 0;
	}

	perf_begin(_sample_perf);

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	// the buffer for read chars is buflen minus null termination
	uint8_t *readbuf = &_linebuf[_linebuf_index];

	// parse entire buffer
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	int bytes_available = 0;
	{
		int retval = ::ioctl(_fd, FIONREAD, &bytes_available);


		if (retval == 0) {
			if (bytes_available < 3) {
				// not enough data
				perf_end(_sample_perf);
				ScheduleDelayed(20_ms);
				return;
			}
		}
	}


	unsigned readlen = math::min(bytes_available,
				     kTotalSizeBytes * 2); //math::min(sizeof(_linebuf) - _linebuf_index - 1, (int)64);

	// read from the sensor (uart buffer)
	int ret = ::read(_fd, readbuf, readlen);

	if (ret < 0) {
		PX4_ERR("read err: %d errno: %d (%s)", ret, errno, strerror(errno));

		// clear line buffer
		memset(_linebuf, 0, sizeof(_linebuf));
		_linebuf_index = 0;

		perf_count(_read_error_perf);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		tcflush(_fd, TCIFLUSH);
		ScheduleDelayed(20_ms);
		return;// ret;

	} else if (ret > 0) {
		_linebuf_index += ret;
		_bytes_read += ret;

	} else {
		// no bytes to read
		ScheduleDelayed(20_ms);
		return;// ret;
	}

	_last_read = hrt_absolute_time();



	if (_print_debug.load()) {
		//fprintf(stderr, "\nread %d bytes: |%.*s| ", ret, ret, readbuf);
		fprintf(stderr, "read %d bytes: ", ret, ret, readbuf);

		for (int i = 0; i < ret; i++) {
			fprintf(stderr, "0x%02X ", readbuf[i]);
		}

		fprintf(stderr, "\n");


		// //fprintf(stderr, "|%.*s| (%d bytes total)\n", sizeof(_linebuf), _linebuf, _linebuf_index);


		// for (int c = 0; c < ret; c++) {
		// 	fprintf(stderr, "%X ", readbuf[c]);
		// }

		// fprintf(stderr, "\n");

		// for (int c = 0; c <= _linebuf_index; c++) {
		// 	fprintf(stderr, "%c", _linebuf[c]);
		// }

		if (_print_debug.load()) {
			fprintf(stderr, "full buffer %d bytes\n", _linebuf_index);

			for (int c = 0; c < _linebuf_index; c++) {
				fprintf(stderr, "%X ", _linebuf[c]);
			}

			fprintf(stderr, "\n");
		}
	}


	if (readbuf[0] == 0x48 && readlen >= 3) {


		//fprintf(stderr, "\n");

		uint8_t data_l = readbuf[1];
		uint8_t data_h = readbuf[2];

		float distance_m = static_cast<float>((data_h << 8) + data_l) * 0.01f;

		float distance_alt = static_cast<float>((data_h & 0x7F) * 128 + (data_l & 0x7F)) * 0.025f;


		if (_print_debug.load()) {
			fprintf(stderr, "read %d bytes: ", ret, ret, readbuf);

			for (int i = 0; i < ret; i++) {
				fprintf(stderr, "0x%02X ", readbuf[i]);
			}

			fprintf(stderr, " distance: %.3f (0x%02X 0x%02X) (alt: %.3f)\n", distance_m, data_l, data_h, distance_alt);
		}

		_px4_rangefinder.update(timestamp_sample, distance_m);

		_linebuf_index -= 3;
	}


	if (_linebuf_index >= kTotalSizeBytes) {

		for (int i = 0; i < _linebuf_index - kTotalSizeBytes + 1; i++) {

			if (ParseBuffer(&_linebuf[i], _linebuf_index - i)) {

				// shift buffer
				memmove(&_linebuf[0], &_linebuf[i + kTotalSizeBytes], _linebuf_index - kTotalSizeBytes - i);
				_linebuf_index -= kTotalSizeBytes;

				ScheduleDelayed(20_ms);
				return;
			}
		}
	}


	if (_print_debug.load()) {
		_print_debug.store(false);
	}



	perf_end(_sample_perf);

	ScheduleNow();
}

bool NRA15::ParseBuffer(const uint8_t *buf, const size_t len)
{
	hrt_abstime timestamp_sample = hrt_absolute_time();

	if ((buf[0] == 0xAA && buf[1] == 0xAA) // 0xAAAA : header bytes
	    && (buf[12] == 0x55 && buf[13] == 0x55) // 0x5555
	   ) {
		// found header byte

		uint16_t msg_id = (buf[2]) + (buf[3] << 8);

		// Target Info
		if (buf[2] == 0x0C && buf[3] == 0x07) {

			// NoOfTarget

			// RollCount


			// Rsvd1

			//uint8_t *payload = &buf[4];// 4 - 11


			uint8_t index = buf[4]; // Target ID

			uint8_t rcs = buf[5]; // The section of radar reflection
			uint8_t range_h = buf[6]; // bits 16..23
			uint8_t range_l = buf[7];

			uint16_t range = (range_h << 8) + range_l;
			float distance_m = range * 0.01; // 0.01m resolution


			uint8_t snr = buf[11] - 127;

			//_px4_rangefinder.update(timestamp_sample, distance_m);



			// uint8_t vel_rel_h = VrelH; // bits 40..42
			// uint8_t vel_rel_l = VrelL; // bits 48..55




			distance_sensor_s distance_sensor{};





			device::Device::DeviceId device_id;
			device_id.devid_s.devtype = DRV_DIST_DEVTYPE_NRA15;
			device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

			uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

			if (bus_num < 10) {
				device_id.devid_s.bus = bus_num;
			}

			device_id.devid_s.address = index;

			distance_sensor.device_id = device_id.devid;
			distance_sensor.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;

			distance_sensor.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

			distance_sensor.signal_quality = snr;

			distance_sensor.current_distance = distance_m;



			distance_sensor.timestamp = hrt_absolute_time();

			if (index >= 0 && index <= 7) {
				_distance_sensor_pubs[index].publish(distance_sensor);

			} else {
				PX4_ERR("invalid index %d", index);
			}

		} else {
			PX4_INFO("msg_id: 0x%04X", msg_id);
		}

		return true;
	}

	return false;
}


void NRA15::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	PX4_INFO_RAW("bytes read %d\n", _bytes_read);
	PX4_INFO_RAW("bytes written %d\n", _bytes_written);

	perf_print_counter(_poll_timeout_perf);
	perf_print_counter(_poll_error_perf);
	perf_print_counter(_read_error_perf);
	perf_print_counter(_write_error_perf);
	perf_print_counter(_checksum_good_perf);
	perf_print_counter(_checksum_bad_perf);


	_print_debug.store(true);
}
