
px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	# Drivers
	drivers/bma180
	drivers/bmi055
	drivers/bmi160
	drivers/bmp280
	drivers/hmc5883
	drivers/ist8310
	drivers/lis3mdl
	drivers/mpu6000
	drivers/mpu9250
	drivers/ms5611
	drivers/px4fmu
	drivers/rgbled
	drivers/rgbled_pwm
)

# add common modules (see cmake/px4_module_presets.cmake)
PX4_ADD_ALL_MODULES()
