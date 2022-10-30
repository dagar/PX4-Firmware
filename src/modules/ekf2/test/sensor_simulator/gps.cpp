#include "gps.h"

namespace sensor_simulator
{
namespace sensor
{

Gps::Gps(std::shared_ptr<Ekf> ekf): Sensor(ekf)
{
}

Gps::~Gps()
{
}

void Gps::send(const uint64_t time)
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

void Gps::setAltitude(const float alt)
{
	_gps_data.altitude = alt;
}

void Gps::setLatitude(const double lat)
{
	_gps_data.latitude = lat;
}

void Gps::setLongitude(const double lon)
{
	_gps_data.longitude = lon;
}

void Gps::setVelocity(const Vector3f &vel)
{
	_gps_data.velocity = vel;
}

void Gps::setFixType(const int fix_type)
{
	_gps_data.fix_type = fix_type;
}

void Gps::setNumberOfSatellites(const int num_satellites)
{
	_gps_data.nsats = num_satellites;
}

void Gps::setPdop(const float pdop)
{
	_gps_data.pdop = pdop;
}

void Gps::setPositionRateNED(const Vector3f &rate)
{
	_gps_pos_rate = rate;
}

void Gps::stepHeightByMeters(const float hgt_change)
{
	_gps_data.altitude += hgt_change;
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

gpsSample Gps::getDefaultGpsData()
{
	gpsSample gps_data{};
	gps_data.time_us = 0;
	gps_data.latitude = 47.3566094;
	gps_data.longitude = 8.5190237;
	gps_data.altitude = 422.056;
	gps_data.velocity.setZero();
	gps_data.horizontal_accuracy = 0.5f;
	gps_data.vertical_accuracy = 0.8f;
	gps_data.speed_accuracy = 0.2f;
	gps_data.pdop = 0.0f;
	gps_data.fix_type = 3;
	gps_data.nsats = 16;

	return gps_data;
}

} // namespace sensor
} // namespace sensor_simulator
