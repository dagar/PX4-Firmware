
#include "sik_radio.hpp"

#include <px4_defines.h>

#include <poll.h>
#include <stdio.h>
#include <cstring>

static bool enter_command(int fd)
{
	const char AT_COMMAND[] = "+++";
	int ret_write = ::write(fd, &AT_COMMAND[0], strlen(AT_COMMAND));

	if (ret_write < 0) {
		PX4_ERR("write failed");
		return false;
	}

	sleep(1);

	struct pollfd fds[1] = {};
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	if (poll(&fds[0], 1, 2000) > 0) {

		uint8_t buf[3] {};
		int read_ret = ::read(fd, &buf[0], sizeof(buf));

		if (read_ret < 0) {
			PX4_ERR("read failed");
			return false;
		}

		return (strncmp((const char *)&buf[0], "OK", 2) == 0);
	}

	return false;
}

static bool send_command(int fd, const char *str)
{
	PX4_INFO("send_command");

	const char AT_COMMAND[] = "ATI";
	int ret = ::write(fd, &AT_COMMAND[0], strlen(AT_COMMAND));
	usleep(200000);

	// read command back
	uint8_t buf_cmd[strlen(AT_COMMAND) + 1] {};
	ret = ::read(fd, &buf_cmd[0], sizeof(buf_cmd));

	PX4_INFO("ret: %d buf: %s", ret, buf_cmd);




	struct pollfd fds[1] = {};
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	if (poll(&fds[0], 1, 2000) > 0) {

		// read feedback
		uint8_t buf[40] {};
		ret = ::read(fd, &buf[0], sizeof(buf));

		PX4_INFO("ret: %d buf: %s", ret, buf);

		return true;
	}

	return false;
}

void check_radio_config(int uart_fd, int32_t radio_id)
{
	//PX4_INFO("check_radio_config");

	/* radio config check */
	if (uart_fd >= 0) {

		for (int i = 0; i < 3; i++) {

			if (enter_command(uart_fd)) {

				send_command(uart_fd, "ATI");

				// reboot
				send_command(uart_fd, "ATZ");

				return;
			}

		}

//
//		{
//			/* switch to AT command mode */
//			//usleep(1200000);
//			//fprintf(fs, "+++\n");
//			//usleep(1200000);
//
//			// read and print
//
//			// HACK TO GET OUT OF BOOTLOADER MODE
//			// port->begin(115200, rxS, txS);
//			// port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
//			// port->write(0x30);
//			//  port->write(0x20);
//			// port->set_flow_control(old_flow_control);
//
//
////			if (radio_id > 0) {
////				/* set channel */
////				fprintf(fs, "ATS3=%u\n", radio_id);
////				usleep(200000);
////
////			} else {
////				/* reset to factory defaults */
////				fprintf(fs, "AT&F\n");
////				usleep(200000);
////			}
////
////			/* write config */
////			fprintf(fs, "AT&W");
////			usleep(200000);
//
//
//
//
//			/* reboot */
//			//fprintf(fs, "ATZ");
//			//usleep(200000);
//
//			// XXX NuttX suffers from a bug where
//			// fclose() also closes the fd, not just
//			// the file stream. Since this is a one-time
//			// config thing, we leave the file struct
//			// allocated.
//#ifndef __PX4_NUTTX
//			//fclose(fs);
//#endif
//
//		} else {
//			//PX4_WARN("open fd %d failed", uart_fd);
//		}

		/* reset param and save */
		//radio_id = 0;
		//param_set_no_notification(_param_radio_id, &_radio_id);
	}
}
