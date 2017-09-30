
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_module_list
	#
	# Board support modules
	#
	drivers/stm32
	drivers/led

	#
	# System commands
	#
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/ver

	#
	# Library modules
	#
	lib/param
	lib/systemlib
	lib/mixer
	modules/uORB

	#
	# Libraries
	#
	#lib/mathlib/CMSIS
	lib/controllib
	lib/mathlib
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/conversion
	lib/version



	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	#examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest
)