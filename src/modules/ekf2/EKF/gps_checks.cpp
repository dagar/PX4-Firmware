/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_checks.cpp
 * Perform pre-flight and in-flight GPS quality checks
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <mathlib/mathlib.h>

bool Ekf::collect_gps(const gps_message &gps)
{
	// Run GPS checks always
	_gps_checks_passed = true; // TODO

	if (_filter_initialised && !_NED_origin_initialised && _gps_checks_passed) {
		// If we have good GPS data set the origin's WGS-84 position to the last gps fix
		const double lat = gps.lat * 1.0e-7;
		const double lon = gps.lon * 1.0e-7;

		if (!_pos_ref.isInitialized()) {
			_pos_ref.initReference(lat, lon, _time_last_imu);

			// if we are already doing aiding, correct for the change in position since the EKF started navigating
			if (isHorizontalAidingActive()) {
				double est_lat;
				double est_lon;
				_pos_ref.reproject(-_state.pos(0), -_state.pos(1), est_lat, est_lon);
				_pos_ref.initReference(est_lat, est_lon, _time_last_imu);
			}
		}

		// Take the current GPS height and subtract the filter height above origin to estimate the GPS height of the origin
		_gps_alt_ref = 1e-3f * (float)gps.alt + _state.pos(2);
		_NED_origin_initialised = true;

		_earth_rate_NED = calcEarthRateNED((float)math::radians(_pos_ref.getProjectionReferenceLat()));
		_last_gps_origin_time_us = _time_last_imu;

		const bool declination_was_valid = PX4_ISFINITE(_mag_declination_gps);

		// set the magnetic field data returned by the geo library using the current GPS position
		_mag_declination_gps = get_mag_declination_radians(lat, lon);
		_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
		_mag_strength_gps = get_mag_strength_gauss(lat, lon);

		// request a reset of the yaw using the new declination
		if ((_params.mag_fusion_type != MAG_FUSE_TYPE_NONE)
		     && !declination_was_valid) {
			_mag_yaw_reset_req = true;
		}

		// save the horizontal and vertical position uncertainty of the origin
		_gps_origin_eph = gps.eph;
		_gps_origin_epv = gps.epv;

		// if the user has selected GPS as the primary height source, switch across to using it
		if (_params.vdist_sensor_type == VDIST_SENSOR_GPS) {
			startGpsHgtFusion();
		}

		_information_events.flags.gps_checks_passed = true;
		ECL_INFO("GPS checks passed");

	} else if (!_NED_origin_initialised) {
		// a rough 2D fix is still sufficient to lookup declination
		if ((gps.fix_type >= 2) && (gps.eph < 1000)) {

			const bool declination_was_valid = PX4_ISFINITE(_mag_declination_gps);

			// If we have good GPS data set the origin's WGS-84 position to the last gps fix
			const double lat = gps.lat * 1.0e-7;
			const double lon = gps.lon * 1.0e-7;

			// set the magnetic field data returned by the geo library using the current GPS position
			_mag_declination_gps = get_mag_declination_radians(lat, lon);
			_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
			_mag_strength_gps = get_mag_strength_gauss(lat, lon);

			// request mag yaw reset if there's a mag declination for the first time
			if (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE) {
				if (!declination_was_valid && PX4_ISFINITE(_mag_declination_gps)) {
					_mag_yaw_reset_req = true;
				}
			}

			_earth_rate_NED = calcEarthRateNED((float)math::radians(lat));
		}
	}

	// start collecting GPS if there is a 3D fix and the NED origin has been set
	return _NED_origin_initialised && (gps.fix_type >= 3);
}
