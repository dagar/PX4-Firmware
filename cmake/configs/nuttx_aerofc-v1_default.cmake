include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/aerofc_adc
	drivers/boards/aerofc-v1
	drivers/device
	drivers/gps
	drivers/hmc5883
	drivers/ist8310
	drivers/led
	drivers/ll40ls
	drivers/mpu9250
	drivers/ms5611
	drivers/px4fmu
	drivers/rc_input
	drivers/stm32
	drivers/tap_esc
	modules/sensors

	#
	# System commands
	#
	systemcmds/config
	systemcmds/dumpfile
	systemcmds/i2c
	systemcmds/mixer
	systemcmds/motor_test
	systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/reboot
	systemcmds/top
	systemcmds/ver

	#
	# General system control
	#
	modules/commander
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/local_position_estimator

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control

	#
	# Logging
	#
	modules/logger

	#
	# Library modules
	#
	modules/dataman
	modules/systemlib
	modules/systemlib/mixer
	modules/systemlib/param
	modules/uORB

	#
	# Libraries
	#
	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/mathlib
	lib/mathlib/math/filter
	lib/micro-CDR
	lib/rc
	lib/tailsitter_recovery
	lib/version
	platforms/common
	platforms/nuttx
	platforms/nuttx/px4_layer
)

set(config_extra_builtin_cmds
	)

set(config_io_board
	)
