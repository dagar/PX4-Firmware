#include "gps.h"

namespace sensor_simulator
{
namespace sensor
{

GpsYaw::GpsYaw(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
}

GpsYaw::~GpsYaw()
{
}

void GpsYaw::send(const uint64_t time)
{
	const float dt = static_cast<float>(time - _gps_data.time_us) * 1e-6f;

	_gps_data.time_us = time;

	if (fabsf(_gps_pos_rate(0)) > FLT_EPSILON || fabsf(_gps_pos_rate(1)) > FLT_EPSILON) {
		stepHorizontalPositionByMeters(Vector2f(_gps_pos_rate) * dt);
	}

	if (fabsf(_gps_pos_rate(2)) > FLT_EPSILON) {
		stepHeightByMeters(-_gps_pos_rate(2) * dt);
	}

	_ekf->setGpsData(_gps_data);
}

void Gps::setData(const gpsSample &gps)
{
	_gps_data = gps;
}

void Gps::setYaw(const float yaw)
{
	_gps_data.yaw = yaw;
}

void Gps::stepHorizontalPositionByMeters(const Vector2f hpos_change)
{
	float hposN_curr{0.f};
	float hposE_curr{0.f};

	double lat_new{0.0};
	double lon_new{0.0};

	_ekf->global_origin().project(_gps_data.latitude, _gps_data.longitude, hposN_curr, hposE_curr);

	Vector2f hpos_new = Vector2f{hposN_curr, hposE_curr} + hpos_change;

	_ekf->global_origin().reproject(hpos_new(0), hpos_new(1), lat_new, lon_new);

	_gps_data.latitude = lat_new;
	_gps_data.longitude = lon_new;
}

gpsYawSample GpsYaw::getDefaultGpsData()
{
	gpsYawSample gps_yaw_data{};
	gps_yaw_data.time_us = 0;
	gps_yaw_data.yaw = NAN;
	gps_yaw_data.yaw_offset = NAN;
	gps_yaw_data.yaw_accuracy = NAN;
	return gps_yaw_data;
}

} // namespace sensor
} // namespace sensor_simulator
