
#ifndef IMU_SAMPLE_HPP
#define IMU_SAMPLE_HPP

struct imuSample {
	uint64_t         time_us{};                ///< timestamp of the measurement (uSec)
	matrix::Vector3f delta_ang{};              ///< delta angle in body frame (integrated gyro measurements) (rad)
	matrix::Vector3f delta_vel{};              ///< delta velocity in body frame (integrated accelerometer measurements) (m/sec)
	float            delta_ang_dt{};           ///< delta angle integration period (sec)
	float            delta_vel_dt{};           ///< delta velocity integration period (sec)
	bool             delta_vel_clipping[3] {}; ///< true (per axis) if this sample contained any accelerometer clipping
};


#endif // #ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP
