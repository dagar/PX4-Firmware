
#include "MSGBridge.hpp"

#include <px4_platform_common/getopt.h>

#include <crc32.h>

#include <termios.h>

MSGBridge::MSGBridge(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

MSGBridge::~MSGBridge()
{
	//perf_free(_loop_perf);
}

bool MSGBridge::init()
{
	PX4_INFO("starting");

	_start_time = hrt_absolute_time();

	for (auto &sub : _subs) {
		sub.registerCallback();
	}

	ScheduleNow();

	return true;
}

void MSGBridge::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_uart_fd < 0) {
		_uart_fd = ::open(_device, O_RDWR | O_NONBLOCK);

		if (_uart_fd >= 0) {
			PX4_INFO("opening %s", _device);

			int speed = B921600;

			/* Try to set baud rate */
			struct termios uart_config {};
			int termios_state;

			/* Initialize the uart config */
			if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
				PX4_ERR("ERR GET CONF %s: %d\n", _device, termios_state);
			}

			/* Clear ONLCR flag (which appends a CR for every LF) */
			uart_config.c_oflag &= ~ONLCR;

			/* Set baud rate */
			if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
				PX4_ERR("SET BAUD %s: %d\n", _device, termios_state);
			}


#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
			/* Put in raw mode */
			cfmakeraw(&uart_config);
#endif

			if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
				PX4_WARN("SET CONF %s\n", _device);
			}

			tcgetattr(_uart_fd, &uart_config);
			uart_config.c_cflag |= CRTSCTS;
			tcsetattr(_uart_fd, TCSANOW, &uart_config);

		} else {
			PX4_ERR("opening %s failed", _device);
		}
	}










	ScheduleDelayed(100_ms);

	// serial port
	// TODO: encode
	// TODO: decode
	for (auto &sub : _subs) {
		uint8_t buffer[512];

		if (sub.update(&buffer[2])) {

			size_t sz = sub.get_topic_size();

			// 1st Byte - Sync Flag (Value: 0xff)
			// 2nd Byte - ID
			// 3rd Byte - instance
			// 3rd Byte - Message Data
			//
			// last Byte - Checksum over message ID and data
			buffer[0] = SYNC_FLAG;
			buffer[1] = static_cast<uint8_t>(sub.get_topic_enum());
			buffer[2] = sub.get_instance();

			size_t total_length = sz + 3;

			buffer[3 + sz + 1] = crc32(buffer, total_length - 1);

			_total_bytes_copied += total_length;

			ssize_t bytes_written = ::write(_uart_fd, buffer, total_length);

			if (bytes_written > 0) {
				_total_bytes_written += bytes_written;


				// TODO: track I/O rates per topic?
				// error rates? (missed generation, etc)
			}
		}
	}

}

int MSGBridge::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
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
		return -1;
	}



	MSGBridge *instance = new MSGBridge(device);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MSGBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MSGBridge::print_status()
{
	//perf_print_counter(_cycle_perf);

	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		//PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	const float time_s = hrt_elapsed_time(&_start_time) * 1e-6f;

	PX4_INFO("total bytes copied: %d (%.1f B/s)", _total_bytes_copied, (double) _total_bytes_copied / (double)time_s);
	PX4_INFO("total bytes written: %d (%.1f B/s)", _total_bytes_written, (double) _total_bytes_copied / (double)time_s);

	return 0;
}

int MSGBridge::print_usage(const char *reason)
{
	return 0;
}

extern "C" __EXPORT int msg_bridge_main(int argc, char *argv[])
{
	return MSGBridge::main(argc, argv);
}
