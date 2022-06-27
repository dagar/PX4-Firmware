/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#include "GnssUblox.hpp"

GnssUblox::GnssUblox(const char *path, gps_driver_mode_t mode, GPSHelper::Interface interface,
		     unsigned configured_baudrate) :
	Device(MODULE_NAME),
	_configured_baudrate(configured_baudrate),
	_mode(mode),
	_interface(interface)
{
	/* store port name */
	strncpy(_port, path, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	_sensor_gps.heading = NAN;
	_sensor_gps.heading_offset = NAN;

	int32_t enable_sat_info = 0;
	param_get(param_find("UBX_SAT_INFO"), &enable_sat_info);

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_sat_info = new GPS_Sat_Info();
		_p_report_sat_info = &_sat_info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}

	set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);

	char c = _port[strlen(_port) - 1]; // last digit of path (eg /dev/ttyS2)
	set_device_bus(atoi(&c));

	decodeInit();
}

GnssUblox::~GnssUblox()
{
	delete _sat_info;
	delete _dump_to_device;
	delete _dump_from_device;
	delete _rtcm_parsing;
}

int GnssUblox::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	handleInjectDataTopic();

#if !defined(__PX4_QURT)

	/* For non QURT, use the usual polling. */

	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	//FIXME: add a unified poll() API
	const int max_timeout = 50;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

	if (ret > 0) {
		/* if we have new data from GPS, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If we have all requested data available, read it without waiting.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			const unsigned character_count = 32; // minimum bytes that we want to read
			unsigned baudrate = _baudrate == 0 ? 115200 : _baudrate;
			const unsigned sleeptime = character_count * 1000000 / (baudrate / 10);

#ifdef __PX4_NUTTX
			int err = 0;
			int bytes_available = 0;
			err = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

			if (err != 0 || bytes_available < (int)character_count) {
				px4_usleep(sleeptime);
			}

#else
			px4_usleep(sleeptime);
#endif

			ret = ::read(_serial_fd, buf, buf_length);

			if (ret > 0) {
				_num_bytes_read += ret;
			}

		} else {
			ret = -1;
		}
	}

	return ret;

#else
	/* For QURT, just use read for now, since this doesn't block, we need to slow it down
	 * just a bit. */
	px4_usleep(10000);
	return ::read(_serial_fd, buf, buf_length);
#endif
}

void GnssUblox::handleInjectDataTopic()
{
	if (!shouldInjectRTCM()) {
		return;
	}

	bool updated = false;

	// Limit maximum number of GPS injections to 6 since usually
	// GPS injections should consist of 1-4 packets (GPS, Glonass, BeiDou, Galileo).
	// Looking at 6 packets thus guarantees, that at least a full injection
	// data set is evaluated.
	const size_t max_num_injections = 6;
	size_t num_injections = 0;

	do {
		num_injections++;
		updated = _orb_inject_data_sub.updated();

		if (updated) {
			gps_inject_data_s msg;

			if (_orb_inject_data_sub.copy(&msg)) {

				// Prevent injection of data from self
				if (msg.device_id != get_device_id()) {
					/* Write the message to the gps device. Note that the message could be fragmented.
					* But as we don't write anywhere else to the device during operation, we don't
					* need to assemble the message first.
					*/
					injectData(msg.data, msg.len);

					++_last_rate_rtcm_injection_count;
				}
			}
		}
	} while (updated && num_injections < max_num_injections);
}

bool GnssUblox::injectData(uint8_t *data, size_t len)
{
	dumpGpsData(data, len, gps_dump_comm_mode_t::Full, true);

	size_t written = ::write(_serial_fd, data, len);
	::fsync(_serial_fd);
	return written == len;
}

int GnssUblox::setBaudrate(unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

#ifndef B460800
#define B460800 460800
#endif

	case 460800: speed = B460800; break;

#ifndef B921600
#define B921600 921600
#endif

	case 921600: speed = B921600; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

	return 0;
}

void GnssUblox::initializeCommunicationDump()
{
	param_t gps_dump_comm_ph = param_find("UBX_DUMP_COMM");
	int32_t param_dump_comm;

	if (gps_dump_comm_ph == PARAM_INVALID || param_get(gps_dump_comm_ph, &param_dump_comm) != 0) {
		return;
	}

	if (param_dump_comm < 1 || param_dump_comm > 2) {
		return; //dumping disabled
	}

	_dump_from_device = new gps_dump_s();
	_dump_to_device = new gps_dump_s();

	if (!_dump_from_device || !_dump_to_device) {
		PX4_ERR("failed to allocated dump data");
		return;
	}

	memset(_dump_to_device, 0, sizeof(gps_dump_s));
	memset(_dump_from_device, 0, sizeof(gps_dump_s));

	//make sure to use a large enough queue size, so that we don't lose messages. You may also want
	//to increase the logger rate for that.
	_dump_communication_pub.advertise();

	_dump_communication_mode = (gps_dump_comm_mode_t)param_dump_comm;
}

void GnssUblox::dumpGpsData(uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device)
{
	gps_dump_s *dump_data = msg_to_gps_device ? _dump_to_device : _dump_from_device;

	if (_dump_communication_mode != mode || !dump_data) {
		return;
	}

	while (len > 0) {
		size_t write_len = len;

		if (write_len > sizeof(dump_data->data) - dump_data->len) {
			write_len = sizeof(dump_data->data) - dump_data->len;
		}

		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= sizeof(dump_data->data)) {
			if (msg_to_gps_device) {
				dump_data->len |= 1 << 7;
			}

			dump_data->timestamp = hrt_absolute_time();
			_dump_communication_pub.publish(*dump_data);
			dump_data->len = 0;
		}
	}
}

void GnssUblox::run()
{
	param_t handle = param_find("UBX_YAW_OFFSET");
	float heading_offset = 0.f;

	if (handle != PARAM_INVALID) {
		param_get(handle, &heading_offset);
		heading_offset = matrix::wrap_pi(math::radians(heading_offset));
	}

	int32_t gps_ubx_dynmodel = 7; // default to 7: airborne with <2g acceleration
	handle = param_find("UBX_DYNMODEL");

	if (handle != PARAM_INVALID) {
		param_get(handle, &gps_ubx_dynmodel);
	}

	handle = param_find("UBX_MODE");

	GPSDriverUBX::UBXMode ubx_mode{GPSDriverUBX::UBXMode::Normal};

	if (handle != PARAM_INVALID) {
		int32_t gps_ubx_mode = 0;
		param_get(handle, &gps_ubx_mode);

		if (gps_ubx_mode == 1) { // heading
			ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBase;

		} else if (gps_ubx_mode == 2) {
			ubx_mode = GPSDriverUBX::UBXMode::MovingBase;

		} else if (gps_ubx_mode == 3) {
			ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBaseUART1;

		} else if (gps_ubx_mode == 4) {
			ubx_mode = GPSDriverUBX::UBXMode::MovingBaseUART1;
		}
	}

	int32_t gnssSystemsParam = static_cast<int32_t>(GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS);

	handle = param_find("UBX_GNSS");
	param_get(handle, &gnssSystemsParam);

	initializeCommunicationDump();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!should_exit()) {

		if (_serial_fd < 0) {
			/* open the serial port */
			_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

			if (_serial_fd < 0) {
				PX4_ERR("failed to open %s err: %d", _port, errno);
				px4_sleep(1);
				continue;
			}
		}

		//_helper = new GPSDriverUBX(_interface, &GnssUblox::callback, this, &_report_gps_pos, _p_report_sat_info, gps_ubx_dynmodel, heading_offset, ubx_mode);
		set_device_type(DRV_GPS_DEVTYPE_UBX);

		_baudrate = _configured_baudrate;
		GPSHelper::GPSConfig gpsConfig{};
		gpsConfig.gnss_systems = static_cast<GPSHelper::GNSSSystemsMask>(gnssSystemsParam);

		if (_dump_communication_mode == gps_dump_comm_mode_t::RTCM) {
			gpsConfig.output_mode = GPSHelper::OutputMode::GPSAndRTCM;

		} else {
			gpsConfig.output_mode = GPSHelper::OutputMode::GPS;
		}

		if (configure(_baudrate, gpsConfig) == 0) {

			/* reset report */
			_sensor_gps = {};
			_sensor_gps.heading = NAN;
			_sensor_gps.heading_offset = heading_offset;

			/* GPS is obviously detected successfully, reset statistics */
			resetUpdateRates();

			// populate specific ublox model
			switch (board()) {
			case GPSDriverUBX::Board::u_blox6:
				set_device_type(DRV_GPS_DEVTYPE_UBX_6);
				break;

			case GPSDriverUBX::Board::u_blox7:
				set_device_type(DRV_GPS_DEVTYPE_UBX_7);
				break;

			case GPSDriverUBX::Board::u_blox8:
				set_device_type(DRV_GPS_DEVTYPE_UBX_8);
				break;

			case GPSDriverUBX::Board::u_blox9:
				set_device_type(DRV_GPS_DEVTYPE_UBX_9);
				break;

			case GPSDriverUBX::Board::u_blox9_F9P:
				set_device_type(DRV_GPS_DEVTYPE_UBX_F9P);
				break;

			default:
				set_device_type(DRV_GPS_DEVTYPE_UBX);
				break;
			}

			int helper_ret;
			unsigned receive_timeout = TIMEOUT_5HZ;

			if ((ubx_mode == GPSDriverUBX::UBXMode::RoverWithMovingBase)
			    || (ubx_mode == GPSDriverUBX::UBXMode::RoverWithMovingBaseUART1)) {
				/* The MB rover will wait as long as possible to compute a navigation solution,
				 * possibly lowering the navigation rate all the way to 1 Hz while doing so. */
				receive_timeout = TIMEOUT_1HZ;
			}

			while ((helper_ret = receive(receive_timeout)) > 0 && !should_exit()) {

				if (helper_ret & 1) {
					publish();

					last_rate_count++;
				}

				if (_p_report_sat_info && (helper_ret & 2)) {
					publishSatelliteInfo();
				}

				reset_if_scheduled();

				/* measure update rate every 5 seconds */
				if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
					float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
					_rate = last_rate_count / dt;
					_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
					_rate_reading = _num_bytes_read / dt;
					last_rate_measurement = hrt_absolute_time();
					last_rate_count = 0;
					_last_rate_rtcm_injection_count = 0;
					_num_bytes_read = 0;

					_rate_vel = _rate_count_vel / (((float)(hrt_absolute_time() - _interval_rate_start)) / 1000000.0f);
					_rate_lat_lon = _rate_count_lat_lon / (((float)(hrt_absolute_time() - _interval_rate_start)) / 1000000.0f);

					resetUpdateRates();
				}

				if (!_healthy) {
					_healthy = true;
				}
			}

			if (_healthy) {
				_healthy = false;
				_rate = 0.0f;
				_rate_rtcm_injection = 0.0f;
			}
		}

		if (_serial_fd >= 0) {
			::close(_serial_fd);
			_serial_fd = -1;
		}
	}

	PX4_INFO("exiting");
}

int
GnssUblox::print_status()
{
	PX4_INFO("protocol: UBX");
	PX4_INFO("status: %s, port: %s, baudrate: %d", _healthy ? "OK" : "NOT OK", _port, _baudrate);
	PX4_INFO("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");
	PX4_INFO("rate reading: \t\t%6i B/s", _rate_reading);

	if (_sensor_gps.timestamp != 0) {
		PX4_INFO("rate position: \t\t%6.2f Hz", (double)getPositionUpdateRate());
		PX4_INFO("rate velocity: \t\t%6.2f Hz", (double)getVelocityUpdateRate());

		PX4_INFO("rate publication:\t\t%6.2f Hz", (double)_rate);
		PX4_INFO("rate RTCM injection:\t%6.2f Hz", (double)_rate_rtcm_injection);

		print_message(ORB_ID(sensor_gps), _sensor_gps);
	}

	return 0;
}

void GnssUblox::schedule_reset(GPSRestartType restart_type)
{
	_scheduled_reset.store((int)restart_type);
}

void GnssUblox::reset_if_scheduled()
{
	GPSRestartType restart_type = (GPSRestartType)_scheduled_reset.load();

	if (restart_type != GPSRestartType::None) {
		_scheduled_reset.store((int)GPSRestartType::None);
		int res = reset(restart_type);

		if (res == -1) {
			PX4_INFO("Reset is not supported on this device.");

		} else if (res < 0) {
			PX4_INFO("Reset failed.");

		} else {
			PX4_INFO("Reset succeeded.");
		}
	}
}

void GnssUblox::publish()
{
	_sensor_gps.device_id = get_device_id();

	_sensor_gps_pub.publish(_sensor_gps);
	// Heading/yaw data can be updated at a lower rate than the other navigation data.
	// The uORB message definition requires this data to be set to a NAN if no new valid data is available.
	_sensor_gps.heading = NAN;
}

void GnssUblox::publishSatelliteInfo()
{
	if (_p_report_sat_info != nullptr) {
		_report_sat_info_pub.publish(*_p_report_sat_info);
	}
}

void GnssUblox::publishRTCMCorrections(uint8_t *data, size_t len)
{
	gps_inject_data_s gps_inject_data{};

	gps_inject_data.timestamp = hrt_absolute_time();
	gps_inject_data.device_id = get_device_id();

	size_t capacity = (sizeof(gps_inject_data.data) / sizeof(gps_inject_data.data[0]));

	if (len > capacity) {
		gps_inject_data.flags = 1; //LSB: 1=fragmented

	} else {
		gps_inject_data.flags = 0;
	}

	size_t written = 0;

	while (written < len) {

		gps_inject_data.len = len - written;

		if (gps_inject_data.len > capacity) {
			gps_inject_data.len = capacity;
		}

		memcpy(gps_inject_data.data, &data[written], gps_inject_data.len);

		_gps_inject_data_pub.publish(gps_inject_data);

		written = written + gps_inject_data.len;
	}
}

void GnssUblox::publishRelativePosition(sensor_gnss_relative_s &gnss_relative)
{
	gnss_relative.device_id = get_device_id();
	gnss_relative.timestamp = hrt_absolute_time();
	_sensor_gnss_relative_pub.publish(gnss_relative);
}

int GnssUblox::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	GPS *_instance = get_instance();

	bool res = false;

	if (argc == 2 && !strcmp(argv[0], "reset")) {

		if (!strcmp(argv[1], "hot")) {
			res = true;
			_instance->schedule_reset(GPSRestartType::Hot);

		} else if (!strcmp(argv[1], "cold")) {
			res = true;
			_instance->schedule_reset(GPSRestartType::Cold);

		} else if (!strcmp(argv[1], "warm")) {
			res = true;
			_instance->schedule_reset(GPSRestartType::Warm);
		}
	}

	if (res) {
		PX4_INFO("Resetting GPS - %s", argv[1]);
		return 0;
	}

	return (res) ? 0 : print_usage("unknown command");
}

int GnssUblox::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPS driver module that handles the communication with the device and publishes the position via uORB.
It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published
on the second uORB topic instance, but it's currently not used by the rest of the system (however the
data will be logged, so that it can be used for comparisons).

### Implementation
There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks
so that they can be used in other projects as well (eg. QGroundControl uses them too).

### Examples

Initiate warm restart of GPS device
$ gps reset warm
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "GPS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset GPS device");
	PRINT_MODULE_USAGE_ARG("cold|warm|hot", "Specify reset type", false);

	return 0;
}

int GnssUblox::task_spawn(int argc, char *argv[])
{
	return task_spawn(argc, argv, Instance::Main);
}

int GnssUblox::task_spawn(int argc, char *argv[], Instance instance)
{
	px4_main_t entry_point;
	if (instance == Instance::Main) {
		entry_point = (px4_main_t)&run_trampoline;
	} else {
		entry_point = (px4_main_t)&run_trampoline_secondary;
	}

	int task_id = px4_task_spawn_cmd("gps", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, TASK_STACK_SIZE,
				   entry_point, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		return -errno;
	}

	if (instance == Instance::Main) {
		_task_id = task_id;
	}

	return 0;
}

GPS *GnssUblox::instantiate(int argc, char *argv[])
{
	return instantiate(argc, argv, Instance::Main);
}

GPS *GnssUblox::instantiate(int argc, char *argv[], Instance instance)
{
	const char *device_name = nullptr;
	const char *device_name_secondary = nullptr;
	int baudrate_main = 0;
	int baudrate_secondary = 0;
	GPSHelper::Interface interface = GPSHelper::Interface::UART;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate_main) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;
		case 'd':
			device_name = myoptarg;
			break;

		case 'i':
			if (!strcmp(myoptarg, "spi")) {
				interface = GPSHelper::Interface::SPI;

			} else if (!strcmp(myoptarg, "uart")) {
				interface = GPSHelper::Interface::UART;

			} else {
				PX4_ERR("unknown interface: %s", myoptarg);
				error_flag = true;
			}
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	GPS *gps = nullptr;
	if (instance == Instance::Main) {
		if (device_name && (access(device_name, R_OK|W_OK) == 0)) {
			gps = new GPS(device_name, mode, interface, instance, baudrate_main);

		} else {
			PX4_ERR("invalid device (-d) %s", device_name ? device_name  : "");
		}
	}

	return gps;
}


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[])
{
	return GnssUblox::main(argc, argv);
}
