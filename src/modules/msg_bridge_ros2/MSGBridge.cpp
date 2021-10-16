
#include <functional>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "px4/msg/actuator_controls.hpp"
#include "px4/msg/battery_status.hpp"
#include "px4/msg/landing_gear.hpp"
#include "px4/msg/manual_control_setpoint.hpp"
#include "px4/msg/multirotor_motor_limits.hpp"
#include "px4/msg/parameter_update.hpp"
#include "px4/msg/rate_ctrl_status.hpp"
#include "px4/msg/vehicle_angular_acceleration.hpp"
#include "px4/msg/vehicle_angular_velocity.hpp"
#include "px4/msg/vehicle_control_mode.hpp"
#include "px4/msg/vehicle_land_detected.hpp"
#include "px4/msg/vehicle_rates_setpoint.hpp"
#include "px4/msg/vehicle_status.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>


#include <uORB/topics/vehicle_status.h>

#include <px4_platform_common/Node.hpp>
#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h> // TODO: ros time

#include <fcntl.h>
#include <termios.h>

#include <uORB/topics/uORBTopics.hpp>

// modules start/stop/status (px4_add_module)
//   work_quue
//   px4_task_create?
// pub/sub
// parameters
// time

using namespace std::chrono_literals;

using std::placeholders::_1;



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

class MSGBridge : public px4::Node, public ModuleParams
{
public:
	MSGBridge() :
		Node("msg_bridge"),
		ModuleParams(nullptr)
	{
		RCLCPP_INFO(get_logger(), "constructing %lu", hrt_absolute_time());


		publisher_vehicle_status_ = this->create_publisher<px4::msg::VehicleStatus>("px4/vehicle_status", 1);



		timer_ = this->create_wall_timer(1000ms, std::bind(&MSGBridge::run, this));
	}

private:

	void run()
	{
		if (_uart_fd < 0) {
			_uart_fd = ::open(_device, O_RDWR | O_NONBLOCK);

			RCLCPP_INFO(get_logger(), "opening %s : %d", _device, _uart_fd);

			int speed = B921600;

			/* Try to set baud rate */
			struct termios uart_config {};
			int termios_state;

			/* Initialize the uart config */
			if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
				RCLCPP_ERROR(get_logger(), "ERR GET CONF %s: %d\n", _device, termios_state);
			}

			/* Clear ONLCR flag (which appends a CR for every LF) */
			uart_config.c_oflag &= ~ONLCR;

			/* Set baud rate */
			if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
				RCLCPP_ERROR(get_logger(), "SET BAUD %s: %d\n", _device, termios_state);
			}

			/* Put in raw mode */
			cfmakeraw(&uart_config);

			if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
				RCLCPP_WARN(get_logger(), "SET CONF %s\n", _device);
			}

			tcgetattr(_uart_fd, &uart_config);
			uart_config.c_cflag |= CRTSCTS;
			tcsetattr(_uart_fd, TCSANOW, &uart_config);
		}


		uint8_t buffer[512] {};

		// non-blocking read
		int nread = ::read(_uart_fd, buffer, sizeof(buffer));

		for (int i = 0; i < nread - 82; i++) {
			if (buffer[i] == SYNC_FLAG) {
				// 1st Byte - Sync Flag (Value: 0xff)
				// 2nd Byte - ID
				// 3rd Byte - instance
				// 3rd Byte - Message Data
				//
				// last Byte - Checksum over message ID and data

				uint8_t sz = 80;

				uint8_t sync_flag = buffer[i];
				uint8_t orb_id    = buffer[i + 1];
				uint8_t instance  = buffer[i + 2];
				// data start     = buffer[i + 3]
				// data end       = buffer[i + 3 + sz - 1]
				uint8_t crc       = buffer[i + 3 + sz];

				uint8_t msg_crc_computed = crc8(&buffer[i], 3 + sz);




				// enum ORB_ID orb_id_enum = static_cast<ORB_ID>(orb_id);

				// const orb_metadata* orb_meta = get_orb_meta(orb_id_enum);

				// if (orb_meta) {
				// 	fprintf(stderr, "orb_meta name: %s\n", orb_meta->o_name);
				// }


				// TODO: checksum (crc32)

				// vehicle_status_s => px4::msg::VehicleStatus (then ROS2 publish)
				if (crc == msg_crc_computed) {
					//fprintf(stderr, "Found ORB_ID: %d:%d, crc:%X:%X \n", orb_id, instance, crc, msg_crc_computed);

					px4_embedded::vehicle_status_s msg_in;
					memcpy(&msg_in, &buffer[i + 3], sz);
					px4::msg::VehicleStatus msg_out = px4_embedded_to_ros2(msg_in);

					//if (msg_in.timestamp > 0) {
					//print_message(vehicle_status);
					//RCLCPP_INFO(get_logger(), "vehicle_status.timestamp: %llu\n", vehicle_status.timestamp);
					//RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_out.c_str());
					//}

					publisher_vehicle_status_->publish(msg_out);

				} else {
					fprintf(stderr, "checksum error crc:%X:%X \n", crc, msg_crc_computed);

					for (int n = 0; n < nread; n++) {
						fprintf(stderr, "|%02hhx", buffer[n]);

						// if (buffer[i] == SYNC_FLAG) {
						// 	fprintf(stderr, "<=SYNC_FLAG!!!! ");
						// } else if (buffer[i] == 0xAA) {
						// 	fprintf(stderr, "<=SYNC_FLAG2!!!! ");
						// }
					}

					fprintf(stderr, "\n\n");
				}

				//break;
			}
		}

	}

	rclcpp::TimerBase::SharedPtr timer_;

	//uORB::Subscription2<px4::msg::BatteryStatus> _battery_status_sub{this, ORB_ID::battery_status};

	// uORB::Publication<px4::msg::ActuatorControls>      _actuators_0_pub{this, ORB_ID::actuator_controls};
	// uORB::Publication<px4::msg::RateCtrlStatus>       _controller_status_pub{this, ORB_ID::rate_ctrl_status};	/**< controller status publication */
	// uORB::Publication<px4::msg::VehicleRatesSetpoint> _v_rates_sp_pub{this, ORB_ID::vehicle_rates_setpoint};			/**< rate setpoint publication */


	rclcpp::Publisher<px4::msg::VehicleStatus>::SharedPtr publisher_vehicle_status_;



	static constexpr uint8_t SYNC_FLAG = 0b1010'1010;


	int _uart_fd{-1};

	const char _device[20] {"/dev/ttyACM0"};
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MSGBridge>());
	rclcpp::shutdown();
	return 0;
}
