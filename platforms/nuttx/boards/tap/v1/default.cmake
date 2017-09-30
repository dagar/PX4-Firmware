
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT tap_common)

set(target_definitions MEMORY_CONSTRAINED_SYSTEM)

set(config_module_list
	#
	# Board support modules
	#
	drivers/led
	drivers/px4fmu
	drivers/rgbled_pwm
	drivers/tap_esc
	drivers/mpu6000
	drivers/ms5611
	drivers/hmc5883
	drivers/gps
	drivers/ms4525_airspeed
	drivers/ms5525_airspeed
	modules/sensors
	drivers/vmount

	#
	# System commands
	#
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/motor_test
	systemcmds/dumpfile
	systemcmds/ver
	systemcmds/topic_listener

	#
	# General system control
	#
	modules/commander
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/fw_pos_control_l1
	modules/fw_att_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/logger

	#
	# Library modules
	#
	lib/param
	lib/systemlib
	lib/mixer
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/led
	lib/rc
	lib/version


)