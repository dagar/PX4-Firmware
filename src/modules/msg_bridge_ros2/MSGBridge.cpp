
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

#include <px4_platform_common/Node.hpp>
#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h> // TODO: ros time


#include <fcntl.h>
#include <termios.h>

// modules start/stop/status (px4_add_module)
//   work_quue
//   px4_task_create?
// pub/sub
// parameters
// time

using namespace std::chrono_literals;

using std::placeholders::_1;

class MSGBridge : public px4::Node, public ModuleParams
{
public:
	MSGBridge() :
		Node("msg_bridge"),
		ModuleParams(nullptr)
	{
		RCLCPP_INFO(get_logger(), "constructing %lu", hrt_absolute_time());

		timer_ = this->create_wall_timer(500ms, std::bind(&MSGBridge::run, this));
	}

private:

	void run()
	{

		if (_uart_fd == -1) {
			_uart_fd = ::open(_device, O_RDWR | O_NONBLOCK);

			RCLCPP_INFO(get_logger(), "opening %s", _device);

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


#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
			/* Put in raw mode */
			cfmakeraw(&uart_config);
#endif

			if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
				RCLCPP_WARN(get_logger(), "SET CONF %s\n", _device);
			}

			tcgetattr(_uart_fd, &uart_config);
			uart_config.c_cflag |= CRTSCTS;
			tcsetattr(_uart_fd, TCSANOW, &uart_config);
		}


		int bytes_available = 0;
		int retval = ::ioctl(_uart_fd, FIONREAD, &bytes_available);

		if (bytes_available >= 79) {

			uint8_t buffer[128];

			// non-blocking read
			int nread = ::read(_uart_fd, buffer, sizeof(buffer));

			if (nread > 0) {
				fprintf(stderr, "\n%d bytes\n", nread);

				for (int i = 0; i < nread; i++) {
					fprintf(stderr, "|%02hhx", buffer[i]);

					// if (buffer[i] == SYNC_FLAG) {
					// 	fprintf(stderr, "<=SYNC_FLAG!!!! ");
					// } else if (buffer[i] == 0xAA) {
					// 	fprintf(stderr, "<=SYNC_FLAG2!!!! ");
					// }
				}

				fprintf(stderr, "\n");
			}



			for (int i = 0; i < nread - 3; i++) {
				if (buffer[i] == SYNC_FLAG) {
					// 1st Byte - Sync Flag (Value: 0xff)
					// 2nd Byte - ID
					// 3rd Byte - instance
					// 3rd Byte - Message Data
					//
					// last Byte - Checksum over message ID and data
					uint8_t orb_id = buffer[i + 1];
					uint8_t instance = buffer[i + 2];

					fprintf(stderr, "Found ORB_ID: %d:%d\n", orb_id, instance);
					break;
				}
			}


		}



	}

	rclcpp::TimerBase::SharedPtr timer_;

	//uORB::Subscription2<px4::msg::BatteryStatus> _battery_status_sub{this, ORB_ID::battery_status};

	// uORB::Publication<px4::msg::ActuatorControls>      _actuators_0_pub{this, ORB_ID::actuator_controls};
	// uORB::Publication<px4::msg::RateCtrlStatus>       _controller_status_pub{this, ORB_ID::rate_ctrl_status};	/**< controller status publication */
	// uORB::Publication<px4::msg::VehicleRatesSetpoint> _v_rates_sp_pub{this, ORB_ID::vehicle_rates_setpoint};			/**< rate setpoint publication */



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
