
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_uavcan_num_ifaces 1)

set(config_module_list
	# Drivers
	drivers/bma180
	drivers/bmi055
	drivers/bmi160
	drivers/bmm150
	drivers/bmp280
	drivers/px4fmu
	drivers/rgbled
	drivers/snapdragon_rc_pwm
)

# add common modules (see cmake/px4_module_presets.cmake)
PX4_ADD_ALL_MODULES()
