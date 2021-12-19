
#include "MSGBridge.hpp"

#include <px4_platform_common/getopt.h>

#include <termios.h>

static constexpr uint8_t crc8_tab[256] = {
	0x00, 0x3e, 0x7c, 0x42, 0xf8, 0xc6, 0x84, 0xba,
	0x95, 0xab, 0xe9, 0xd7, 0x6d, 0x53, 0x11, 0x2f,
	0x4f, 0x71, 0x33, 0x0d, 0xb7, 0x89, 0xcb, 0xf5,
	0xda, 0xe4, 0xa6, 0x98, 0x22, 0x1c, 0x5e, 0x60,
	0x9e, 0xa0, 0xe2, 0xdc, 0x66, 0x58, 0x1a, 0x24,
	0x0b, 0x35, 0x77, 0x49, 0xf3, 0xcd, 0x8f, 0xb1,
	0xd1, 0xef, 0xad, 0x93, 0x29, 0x17, 0x55, 0x6b,
	0x44, 0x7a, 0x38, 0x06, 0xbc, 0x82, 0xc0, 0xfe,
	0x59, 0x67, 0x25, 0x1b, 0xa1, 0x9f, 0xdd, 0xe3,
	0xcc, 0xf2, 0xb0, 0x8e, 0x34, 0x0a, 0x48, 0x76,
	0x16, 0x28, 0x6a, 0x54, 0xee, 0xd0, 0x92, 0xac,
	0x83, 0xbd, 0xff, 0xc1, 0x7b, 0x45, 0x07, 0x39,
	0xc7, 0xf9, 0xbb, 0x85, 0x3f, 0x01, 0x43, 0x7d,
	0x52, 0x6c, 0x2e, 0x10, 0xaa, 0x94, 0xd6, 0xe8,
	0x88, 0xb6, 0xf4, 0xca, 0x70, 0x4e, 0x0c, 0x32,
	0x1d, 0x23, 0x61, 0x5f, 0xe5, 0xdb, 0x99, 0xa7,
	0xb2, 0x8c, 0xce, 0xf0, 0x4a, 0x74, 0x36, 0x08,
	0x27, 0x19, 0x5b, 0x65, 0xdf, 0xe1, 0xa3, 0x9d,
	0xfd, 0xc3, 0x81, 0xbf, 0x05, 0x3b, 0x79, 0x47,
	0x68, 0x56, 0x14, 0x2a, 0x90, 0xae, 0xec, 0xd2,
	0x2c, 0x12, 0x50, 0x6e, 0xd4, 0xea, 0xa8, 0x96,
	0xb9, 0x87, 0xc5, 0xfb, 0x41, 0x7f, 0x3d, 0x03,
	0x63, 0x5d, 0x1f, 0x21, 0x9b, 0xa5, 0xe7, 0xd9,
	0xf6, 0xc8, 0x8a, 0xb4, 0x0e, 0x30, 0x72, 0x4c,
	0xeb, 0xd5, 0x97, 0xa9, 0x13, 0x2d, 0x6f, 0x51,
	0x7e, 0x40, 0x02, 0x3c, 0x86, 0xb8, 0xfa, 0xc4,
	0xa4, 0x9a, 0xd8, 0xe6, 0x5c, 0x62, 0x20, 0x1e,
	0x31, 0x0f, 0x4d, 0x73, 0xc9, 0xf7, 0xb5, 0x8b,
	0x75, 0x4b, 0x09, 0x37, 0x8d, 0xb3, 0xf1, 0xcf,
	0xe0, 0xde, 0x9c, 0xa2, 0x18, 0x26, 0x64, 0x5a,
	0x3a, 0x04, 0x46, 0x78, 0xc2, 0xfc, 0xbe, 0x80,
	0xaf, 0x91, 0xd3, 0xed, 0x57, 0x69, 0x2b, 0x15
};

static constexpr uint8_t crc8(FAR const uint8_t *src, size_t len)
{
	uint8_t crc8val = 0;
	crc8val ^= 0xff;

	for (size_t i = 0; i < len; i++) {
		crc8val = crc8_tab[crc8val ^ src[i]];
	}

	return crc8val ^ 0xff;
}

MSGBridge::MSGBridge(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
	//ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
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
		_uart_fd = ::open(_device, O_RDWR);

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

	ScheduleDelayed(10_ms);

	size_t buf_free = 0;
	ioctl(_uart_fd, FIONSPACE, (unsigned long)&buf_free);

	// serial port
	for (int i = 0; i < NUM_TOPICS; i++) {
		auto &sub = _subs[i];
		uint8_t buffer[512];

		size_t sz = sub.get_topic_size();
		size_t total_length = 3 + sz + 1; // (SYNC_FLAG + ORB_ID + INSTANCE) + sizeof(DATA) + (CRC8)

		if ((buf_free >= total_length) && sub.update(&buffer[3])) {

			if (sub.get_last_generation() != _generations[i] + 1) {
				//PX4_ERR("%s generation %d->%d", sub.get_topic()->o_name, _generations[i], sub.get_last_generation());
			}

			_generations[i] = sub.get_last_generation();

			// 1st Byte - Sync Flag (Value: 0xff)
			// 2nd Byte - ID
			// 3rd Byte - generation (uint8)
			// 3rd Byte - Message Data
			//
			// last Byte - Checksum over message ID and data
			buffer[0] = SYNC_FLAG;
			buffer[1] = static_cast<uint8_t>(sub.get_topic_enum());
			buffer[2] = sub.get_last_generation() % 256;
			// buffer[3]          start of data
			// buffer[3 + sz - 1] end of data
			buffer[3 + sz] = crc8(buffer, 3 + sz);

			_total_bytes_copied += total_length;

			ssize_t bytes_written = ::write(_uart_fd, buffer, total_length);

			// PX4_INFO("msg %d:%d crc:%X, sz=%d, bytes_written=%d", buffer[1], buffer[2], buffer[3 + sz], sz, bytes_written);

			if (bytes_written > 0) {
				_total_bytes_written += bytes_written;

				// TODO: track I/O rates per topic?
				// error rates? (missed generation, etc)

				// recompute free space
				ioctl(_uart_fd, FIONSPACE, (unsigned long)&buf_free);
			}

		} else {
			// PX4_ERR("buffer insufficient to send %s (%d/%d)", sub.get_topic()->o_name, buf_free, total_length);
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
