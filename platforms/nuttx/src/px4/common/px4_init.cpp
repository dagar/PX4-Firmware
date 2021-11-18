/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/init.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform/cpuload.h>
#include <uORB/uORB.h>

#include <fcntl.h>

#include <syslog.h>

#include <sys/mount.h>
#include <syslog.h>

#include <px4_platform/board_dma_alloc.h>

#if defined(CONFIG_I2C)
# include <px4_platform_common/i2c.h>
# include <nuttx/i2c/i2c_master.h>
#endif // CONFIG_I2C

#if defined(PX4_CRYPTO)
#include <px4_platform_common/crypto.h>
#endif

#include <px4_platform_common/mtd_manifest.h>

#if defined(CONFIG_MTD_RAMTRON)
#include <nuttx/spi/spi.h>
// Define the default FRAM usage
const px4_mft_device_t spifram  = {             // FM25V02A on FMUM 32K 512 X 64
	.bus_type = px4_mft_device_t::SPI,
	.devid    = SPIDEV_FLASH(0)
};

const px4_mtd_entry_t fram = {
	.device = &spifram,
	.npart = 2,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/fs/mtd_params",
			.nblocks = 32
		},
		{
			.type = MTD_WAYPOINTS,
			.path = "/fs/mtd_waypoints",
			.nblocks = 32

		}
	},
};

static const px4_mtd_manifest_t default_mtd_config = {
	.nconfigs   = 1,
	.entries = {
		&fram,
	}
};
#endif

extern void cdcacm_init(void);

int px4_platform_init()
{
	syslog(LOG_INFO, "[boot] px4_platform_init\n");

	int ret = px4_console_buffer_init();

	if (ret < 0) {
		return ret;
	}

	// replace stdout with our buffered console
	int fd_buf = open(CONSOLE_BUFFER_DEVICE, O_WRONLY);

	if (fd_buf >= 0) {
		dup2(fd_buf, 1);
		// keep stderr(2) untouched: the buffered console will use it to output to the original console
		close(fd_buf);
	}

#if defined(PX4_CRYPTO)
	PX4Crypto::px4_crypto_init();
#endif

	hrt_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif


#if defined(CONFIG_I2C)
	I2CBusIterator i2c_bus_iterator {I2CBusIterator::FilterType::All};

	while (i2c_bus_iterator.next()) {
		i2c_master_s *i2c_dev = px4_i2cbus_initialize(i2c_bus_iterator.bus().bus);

#if defined(CONFIG_I2C_RESET)
		I2C_RESET(i2c_dev);
#endif // CONFIG_I2C_RESET

		// send software reset to all
		uint8_t buf[1] {};
		buf[0] = 0x06; // software reset

		i2c_msg_s msg{};
		msg.frequency = I2C_SPEED_STANDARD;
		msg.addr = 0x00; // general call address
		msg.buffer = &buf[0];
		msg.length = 1;

		I2C_TRANSFER(i2c_dev, &msg, 1);

		px4_i2cbus_uninitialize(i2c_dev);
	}

#endif // CONFIG_I2C

#if defined(CONFIG_FS_PROCFS)
	syslog(LOG_INFO, "[boot] mount procfs\n");
	int ret_mount_procfs = mount(nullptr, "/proc", "procfs", 0, nullptr);

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret_mount_procfs);
	}

#endif // CONFIG_FS_PROCFS

#if defined(CONFIG_FS_BINFS)
	syslog(LOG_INFO, "[boot] mount binfs\n");
	int ret_mount_binfs = nx_mount(nullptr, "/bin", "binfs", 0, nullptr);

	if (ret_mount_binfs < 0) {
		syslog(LOG_ERR, "ERROR: Failed to mount binfs at /bin: %d\n", ret_mount_binfs);
	}

#endif // CONFIG_FS_BINFS


	px4::WorkQueueManagerStart();

	uorb_start();


	// configure the DMA allocator
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	// Mount filesystem
	int mount_ret = nx_mount("/dev/mmcsd0", "/fs/microsd", "vfat", 0, NULL);

	if (mount_ret < 0) {
		syslog(LOG_ERR, "[boot] Failed to mount the SDCARD. %d\n", ret);
		// TODO: scan
		// TODO: format

		// TODO: littlefs test
	}



	char *parameter_path = nullptr;
	const px4_mtd_manifest_t *mtd_list = nullptr;

	const px4_mft_s *mft = board_get_manifest();

	if (mft != nullptr) {
		syslog(LOG_INFO, "[boot] board manifest MTD\n");

		for (uint32_t m = 0; m < mft->nmft; m++) {
			if (mft->mfts[m].type == MTD) {
				mtd_list = (px4_mtd_manifest_t *)mft->mfts[m].pmft;
			}
		}
	}

	if (!mtd_list) {
		mtd_list = &default_mtd_config;
	}

	if (mtd_list) {
		syslog(LOG_INFO, "[boot] board manifest px4_mtd_config\n");
		px4_mtd_config(mtd_list);

		// mtd_list->nconfigs
		for (uint32_t i = 0; i < mtd_list->nconfigs; i++) {
			const px4_mtd_entry_t *entry = mtd_list->entries[i];

			if (entry) {
				for (uint32_t p = 0; p < entry->npart; p++) {
					if (entry->partd[p].type == MTD_PARAMETERS) {
						syslog(LOG_INFO, "[boot] board manifest %s\n", entry->partd[p].path);
						parameter_path = (char *)entry->partd[p].path;
						break;
					}
				}
			}
		}
	}

	syslog(LOG_INFO, "[boot] param init\n");
	param_init();

	// param_init_file();
	// param_init_flash();

	if (parameter_path) {
		//param_load(int fd);
		param_set_default_file(parameter_path);
		param_load_default();
	}

	// TODO: backup parameters to SD card if import fails



	// TODO:
	//  - hardfault
	//  - sd mount



	px4_log_initialize();

#if defined(CONFIG_SYSTEM_CDCACM) && defined(CONFIG_BUILD_FLAT)
	cdcacm_init();
#endif

	if (board_hardfault_init(2, true) != 0) {
		syslog(LOG_ERR, "[boot] hard fault init failed\n");
	}

	return PX4_OK;
}

int px4_platform_configure(void)
{
	return px4_mft_configure(board_get_manifest());
}
