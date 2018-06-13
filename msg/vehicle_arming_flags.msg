# This is a struct used by the commander internally.

bool mission_required
bool estimator_health
bool global_position
bool local_position
bool valid_airspeed
bool valid_attitude

bool sensor_health_gyro
bool sensor_health_accel
bool sensor_health_mag
bool sensor_health_gps
bool sensor_health_baro

# arming auth?
# GPS
# global_position


# vehicle_status_flags ==== vehicle_arming_requirements ==== vehicle_preflight_warnings
#  compare them all for arming
#  vehicle_arming_requiremnts can change based on parameters, main_state, nav_state, maybe mission?


# arming requirements
#  - gyro close to 0 (vehicle rates)
#  - attitude close to level
#  - airspeed < 5 m/s
#  - 

# crash detection to disarm motors and start beeping

# quadchute generalize
#  - fw_pos_ctrl_status
#  - losing altitude
#  - engine failure
#  - attitude failure
#  - rate controller failure

# src/lib/vehicle_rates or vehicle_model?
#  - 
#  - who owns board rotation? sensors or vehicle_model


# SDPX, etc only shows those parameters if the device is active and running
# MAVLINK
#  - overall parameters
#  - per instance parameters
#  - mavlink device (enum)
#  - baudrate
#  - data rate
#  - mode

# mavlink_status
# navigator_status
# vtol_status
# fw_pos_ctrl_status
# controller_status_attitude
# controller_health? GENERAL CONTROLLER HEALTH!!!!!
# sensors_status
#  per sensor health status + helper to reassemble
# estimator_status
#  src/lib/subscribers
#     - vehicle_rates
#     - vehicle_local_position (check all updated, but only return the best topic and a warning if changed)
#     - vehicle_attitude
#     - vehicle_accel (subscribe all and vote) then gyro, mag, etc
#     - realtime decision or a different part of the system
#     - vehicle_local_position + vehicle_local_position metadata for winner
#     - ORB multi can the first instance be the overall winner
#     - ORB selector type for timeouts?
      - ORB itself could have a general data validator built in
      - ORB subscribe returns newest or changes based on timeout, stale, confidence? OR other pluggable checker?
      - orb_subscribe_multi(BESTOF: TIMEOUT | STALE | )

# generalized MODULE format
# PX4 module should be ROS compatible????
#  ROS drivers for magnetometers, IMU, etc
#  what else can we learn from ROS


# drivers_device_arch move to appropriate location



# EKF2
 - gyro voted or specific
 - accel 
 - baro
 - mag
 - airspeed 
 - gps 
 - distance sensor


 vehicle_sensor_accel
 