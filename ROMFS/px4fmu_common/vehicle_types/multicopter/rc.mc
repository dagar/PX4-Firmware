#!/bin/sh
#
# Multicopter setup.
#
if [ $VEHICLE_TYPE = mc ]
then
	if [ $MIXER = none ]
	then
		echo "MC mixer undefined"
	fi

	if [ $MAV_TYPE = none ]
	then
		# Set a default MAV_TYPE = 2 if not defined.
		set MAV_TYPE 2

		# Use mixer to detect vehicle type
		if [ $MIXER = coax ]
		then
			set MAV_TYPE 3
		fi
		if [ $MIXER = hexa_x -o $MIXER = hexa_+ ]
		then
			set MAV_TYPE 13
		fi
		if [ $MIXER = hexa_cox ]
		then
			set MAV_TYPE 13
		fi
		if [ $MIXER = octo_x -o $MIXER = octo_+ ]
		then
			set MAV_TYPE 14
		fi
		if [ $MIXER = octo_cox -o $MIXER = octo_cox_w ]
		then
			set MAV_TYPE 14
		fi
		if [ $MIXER = tri_y_yaw- -o $MIXER = tri_y_yaw+ ]
		then
			set MAV_TYPE 15
		fi
	fi

	# Set the mav type parameter.
	param set MAV_TYPE ${MAV_TYPE}

	# Load mixer and configure outputs.
	source /etc/init.d/rc.interface

	# LPE
	if param compare SYS_MC_EST_GROUP 1
	then
		# Try to start LPE. If it fails, start EKF2 as a default.
		# Unfortunately we do not build it on px4_fmu-v2 due to a limited flash.
		if attitude_estimator_q start
		then
			echo "WARN [init] Estimator LPE unsupported, EKF2 recommended."
			local_position_estimator start
		else
			echo "ERROR [init] Estimator LPE not available. Using EKF2"
			param set SYS_MC_EST_GROUP 2
			param save
			reboot
		fi
	else
		# Q estimator (attitude estimation only)
		if param compare SYS_MC_EST_GROUP 3
		then
			attitude_estimator_q start
		else
			# EKF2
			param set SYS_MC_EST_GROUP 2
			ekf2 start
		fi
	fi

	land_detector start multicopter

	# Start controllers
	mc_rate_control start
	mc_att_control start
	mc_hover_thrust_estimator start
	mc_pos_control start
fi
