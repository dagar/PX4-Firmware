/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file gps.cpp
 * Driver for the GPS on a serial/spi port
 */

#pragma once

#include <poll.h>

#include <termios.h>
#include <cstring>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gnss_relative.h>

#include "ubx.h"

#ifdef __PX4_LINUX
#include <linux/spi/spidev.h>
#endif /* __PX4_LINUX */

#define TIMEOUT_1HZ		1300	//!< Timeout time in mS, 1000 mS (1Hz) + 300 mS delta for error
#define TIMEOUT_5HZ		500		//!< Timeout time in mS,  200 mS (5Hz) + 300 mS delta for error
#define RATE_MEASUREMENT_PERIOD 5000000

enum class gps_dump_comm_mode_t : int32_t {
	Disabled = 0,
	Full, ///< dump full RX and TX data for all devices
	RTCM ///< dump received RTCM from Main GPS
};

/* struct for dynamic allocation of satellite info data */
struct GPS_Sat_Info {
	satellite_info_s _data;
};

static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(1760);

class GnssUblox : public ModuleBase<GnssUblox>, public device::Device
{
public:
	GnssUblox(const char *path, GPSHelper::Interface interface, unsigned configured_baudrate);
	~GnssUblox() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** spawn task and select the instance */
	static int task_spawn(int argc, char *argv[], Instance instance);

	/** @see ModuleBase */
	static GnssUblox *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	int print_status() override;


	enum class GPSRestartType {
		None,

		/**
		 * In hot start mode, the receiver was powered down only for a short time (4 hours or less),
		 * so that its ephemeris is still valid. Since the receiver doesn't need to download ephemeris
		 * again, this is the fastest startup method.
		 */
		Hot,

		/**
		 * In warm start mode, the receiver has approximate information for time, position, and coarse
		 * satellite position data (Almanac). In this mode, after power-up, the receiver normally needs
		 * to download ephemeris before it can calculate position and velocity data.
		 */
		Warm,

		/**
		 * In cold start mode, the receiver has no information from the last position at startup.
		 * Therefore, the receiver must search the full time and frequency space, and all possible
		 * satellite numbers. If a satellite signal is found, it is tracked to decode the ephemeris,
		 * whereas the other channels continue to search satellites. Once there is a sufficient number
		 * of satellites with valid ephemeris, the receiver can calculate position and velocity data.
		 */
		Cold
	};

	/**
	 * Schedule reset of the GPS device
	 */
	void schedule_reset(GPSRestartType restart_type);

	/**
	 * Reset device if reset was scheduled
	 */
	void reset_if_scheduled();





	// GPSHelper
	struct SurveyInStatus {
		double latitude;              /**< NAN if unknown/not set [deg] */
		double longitude;             /**< NAN if unknown/not set [deg] */
		float altitude;               /**< NAN if unknown/not set [m] */
		uint32_t mean_accuracy;       /**< [mm] */
		uint32_t duration;            /**< [s] */
		uint8_t flags;                /**< bit 0: valid, bit 1: active */
	};

// TODO: this number seems wrong
#define GPS_EPOCH_SECS ((time_t)1234567890ULL)


	enum class OutputMode : uint8_t {
		GPS = 0,    ///< normal GPS output
		GPSAndRTCM, ///< normal GPS+RTCM output
		RTCM        ///< request RTCM output. This is used for (fixed position) base stations
	};

	enum class Interface : uint8_t {
		UART = 0,
		SPI
	};

	/**
	 * Bitmask for GPS_1_GNSS and GPS_2_GNSS
	 * No bits set should keep the receiver's default config
	 */
	enum class GNSSSystemsMask : int32_t {
		RECEIVER_DEFAULTS = 0,
		ENABLE_GPS =        1 << 0,
		ENABLE_SBAS =       1 << 1,
		ENABLE_GALILEO =    1 << 2,
		ENABLE_BEIDOU =     1 << 3,
		ENABLE_GLONASS =    1 << 4
	};

	struct GPSConfig {
		OutputMode output_mode;
		GNSSSystemsMask gnss_systems;
	};

	float getPositionUpdateRate() { return _rate_lat_lon; }
	float getVelocityUpdateRate() { return _rate_vel; }

	void resetUpdateRates()
	{
		_rate_count_vel = 0;
		_rate_count_lat_lon = 0;
		_interval_rate_start = hrt_absolute_time();
	}


	/**
	 * Allow a driver to disable RTCM injection
	 */
	bool shouldInjectRTCM() override { return _mode != UBXMode::RoverWithMovingBase; }

	enum class UBXMode : uint8_t {
		Normal,                    ///< all non-heading configurations
		RoverWithMovingBase,       ///< expect RTCM input on UART2 from a moving base for heading output
		MovingBase,                ///< RTCM output on UART2 to a rover (GPS is installed on the vehicle)
		RoverWithMovingBaseUART1, ///< expect RTCM input on UART1 from a moving base for heading output
		MovingBaseUART1,          ///< RTCM output on UART1 to a rover (GPS is installed on the vehicle)
	};

	int configure(unsigned &baudrate, const GPSConfig &config) override;

	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
	int receive(unsigned timeout) override;

	int reset(GPSRestartType restart_type) override;

	enum class Board : uint8_t {
		unknown = 0,
		u_blox5 = 5,
		u_blox6 = 6,
		u_blox7 = 7,
		u_blox8 = 8, ///< M8N or M8P
		u_blox9 = 9, ///< M9N, or any F9*, except F9P
		u_blox9_F9P = 10, ///< F9P
	};

	const Board &board() const { return _board; }

private:
	int				_serial_fd{-1};					///< serial interface to GPS
	unsigned			_baudrate{0};					///< current baudrate
	const unsigned			_configured_baudrate{0};			///< configured baudrate (0=auto-detect)
	char				_port[20] {};					///< device / serial port path

	bool				_healthy{false};				///< flag to signal if the GPS is ok

	GPSHelper::Interface		_interface;   					///< interface

	GPS_Sat_Info			*_sat_info{nullptr};				///< instance of GPS sat info data object

	sensor_gps_s _sensor_gps{};
	satellite_info_s		*_satellite_info{nullptr};			///< pointer to uORB topic for satellite info

	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};	///< uORB pub for gps position
	uORB::PublicationMulti<sensor_gnss_relative_s> _sensor_gnss_relative_pub{ORB_ID(sensor_gnss_relative)};

	uORB::PublicationMulti<satellite_info_s> _satellite_info_pub{ORB_ID(satellite_info)};		///< uORB pub for satellite info

	float				_rate{0.0f};					///< position update rate
	float				_rate_rtcm_injection{0.0f};			///< RTCM message injection rate
	unsigned			_last_rate_rtcm_injection_count{0};		///< counter for number of RTCM messages
	unsigned			_num_bytes_read{0}; 				///< counter for number of read bytes from the UART (within update interval)
	unsigned			_rate_reading{0}; 				///< reading rate in B/s

	uORB::Subscription		     _orb_inject_data_sub{ORB_ID(gps_inject_data)};
	uORB::Publication<gps_inject_data_s> _gps_inject_data_pub{ORB_ID(gps_inject_data)};
	uORB::Publication<gps_dump_s>	     _dump_communication_pub{ORB_ID(gps_dump)};
	gps_dump_s			     *_dump_to_device{nullptr};
	gps_dump_s			     *_dump_from_device{nullptr};
	gps_dump_comm_mode_t                 _dump_communication_mode{gps_dump_comm_mode_t::Disabled};

	px4::atomic<int> _scheduled_reset{(int)GPSRestartType::None};

	/**
	 * Publish RTCM corrections
	 */
	void 				publishRTCMCorrections(uint8_t *data, size_t len);

	/**
	 * check for new messages on the inject data topic & handle them
	 */
	void handleInjectDataTopic();

	/**
	 * send data to the device, such as an RTCM stream
	 * @param data
	 * @param len
	 */
	inline bool injectData(uint8_t *data, size_t len);

	/**
	 * set the Baudrate
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(unsigned baud);

	/**
	 * Dump gps communication.
	 * @param data message
	 * @param len length of the message
	 * @param mode calling source
	 * @param msg_to_gps_device if true, this is a message sent to the gps device, otherwise it's from the device
	 */
	void dumpGpsData(uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device);

	void initializeCommunicationDump();

	static constexpr int SET_CLOCK_DRIFT_TIME_S{5};			///< RTC drift time when time synchronization is needed (in seconds)




	/**
	 * set survey-in specs for RTK base station setup (for finding an accurate base station position
	 * by averaging the position measurements over time).
	 * @param survey_in_acc_limit minimum accuracy in 0.1mm
	 * @param survey_in_min_dur minimum duration in seconds
	 */
	void setSurveyInSpecs(uint32_t survey_in_acc_limit, uint32_t survey_in_min_dur)
	{
		_base_settings.type = BaseSettingsType::survey_in;
		_base_settings.settings.survey_in.acc_limit = survey_in_acc_limit;
		_base_settings.settings.survey_in.min_dur = survey_in_min_dur;
	}

	/**
	 * Set a fixed base station position. This can be used if the base position is already known to
	 * avoid doing a survey-in.
	 * @param latitude [deg]
	 * @param longitude [deg]
	 * @param altitude [m]
	 * @param position_accuracy 3D position accuracy (set to 0 if unknown) [mm]
	 */
	void setBasePosition(double latitude, double longitude, float altitude, float position_accuracy)
	{
		_base_settings.type = BaseSettingsType::fixed_position;
		_base_settings.settings.fixed_position.latitude = latitude;
		_base_settings.settings.fixed_position.longitude = longitude;
		_base_settings.settings.fixed_position.altitude = altitude;
		_base_settings.settings.fixed_position.position_accuracy = position_accuracy;
	}






	/**
	 * Convert an ECEF (Earth Centered Earth Fixed) coordinate to LLA WGS84 (Lat, Lon, Alt).
	 * Ported from: https://stackoverflow.com/a/25428344
	 * @param ecef_x ECEF X-coordinate [m]
	 * @param ecef_y ECEF Y-coordinate [m]
	 * @param ecef_z ECEF Z-coordinate [m]
	 * @param latitude [deg]
	 * @param longitude [deg]
	 * @param altitude [m]
	 */
	static void ECEF2lla(double ecef_x, double ecef_y, double ecef_z, double &latitude, double &longitude, float &altitude)
	{
		// WGS84 ellipsoid constants
		constexpr double a = 6378137.; // radius
		constexpr double e = 8.1819190842622e-2;  // eccentricity

		constexpr double asq = a * a;
		constexpr double esq = e * e;

		double x = ecef_x;
		double y = ecef_y;
		double z = ecef_z;

		double b = sqrt(asq * (1. - esq));
		double bsq = b * b;
		double ep = sqrt((asq - bsq) / bsq);
		double p = sqrt(x * x + y * y);
		double th = atan2(a * z, b * p);

		longitude = atan2(y, x);
		double sin_th = sin(th);
		double cos_th = cos(th);
		latitude = atan2(z + ep * ep * b * sin_th * sin_th * sin_th, p - esq * a * cos_th * cos_th * cos_th);
		double sin_lat = sin(latitude);
		double N = a / sqrt(1. - esq * sin_lat * sin_lat);
		altitude = (float)(p / cos(latitude) - N);

		// rad to deg
		longitude *= 180. / M_PI;
		latitude *= 180. / M_PI;

		// correction for altitude near poles left out.
	}





	// GPS helper
	uint8_t _rate_count_lat_lon{};
	uint8_t _rate_count_vel{};

	float _rate_lat_lon{0.0f};
	float _rate_vel{0.0f};

	uint64_t _interval_rate_start{0};



	enum class BaseSettingsType : uint8_t {
		survey_in,
		fixed_position
	};
	struct SurveyInSettings {
		uint32_t acc_limit;
		uint32_t min_dur;
	};
	struct FixedPositionSettings {
		double latitude;
		double longitude;
		float altitude;
		float position_accuracy;
	};
	struct BaseSettings {
		BaseSettingsType type;
		union {
			SurveyInSettings survey_in;
			FixedPositionSettings fixed_position;
		} settings;
	};

	BaseSettings _base_settings{};



	// ubx
	int activateRTCMOutput(bool reduce_update_rate);

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void addByteToChecksum(const uint8_t);

	/**
	 * Calculate & add checksum for given buffer
	 */
	void calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);

	/**
	 * Configure message rate.
	 * Note: this is deprecated with protocol version >= 27
	 * @return true on success, false on write error
	 */
	bool configureMessageRate(const uint16_t msg, const uint8_t rate);

	/**
	 * Combines the configure_message_rate & wait_for_ack calls.
	 * Note: this is deprecated with protocol version >= 27
	 * @return true on success
	 */
	inline bool configureMessageRateAndAck(uint16_t msg, uint8_t rate, bool report_ack_error = false);

	/**
	 * Send configuration values and desired message rates
	 * @param gnssSystems Set of GNSS systems to use
	 * @return 0 on success, <0 on error
	 */
	int configureDevice(const GNSSSystemsMask &gnssSystems);

	/**
	 * Add a configuration value to _buf and increase the message size msg_size as needed
	 * @param key_id one of the UBX_CFG_KEY_* constants
	 * @param value configuration value
	 * @param msg_size CFG-VALSET message size: this is an input & output param
	 * @return true on success, false if buffer too small
	 */
	template<typename T>
	bool cfgValset(uint32_t key_id, T value, int &msg_size);

	/**
	 * Add a configuration value that is port-specific (MSGOUT messages).
	 * Note: Key ID must be the one for I2C, and the implementation assumes the
	 *       Key ID's are in increasing order for the other ports: I2C, UART1, UART2, USB, SPI
	 *       (this is a safe assumption for all MSGOUT messages according to u-blox).
	 *
	 * @param key_id I2C key ID
	 * @param value configuration value
	 * @param msg_size CFG-VALSET message size: this is an input & output param
	 * @return true on success, false if buffer too small
	 */
	bool cfgValsetPort(uint32_t key_id, uint8_t value, int &msg_size);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit(void);

	/**
	 * Calculate FNV1 hash
	 */
	uint32_t fnv1_32_str(uint8_t *str, uint32_t hval);

	/**
	 * Init _buf as CFG-VALSET
	 * @return size of the message (without any config values)
	 */
	int initCfgValset();

	/**
	 * Start or restart the survey-in procees. This is only used in RTCM ouput mode.
	 * It will be called automatically after configuring.
	 * @return 0 on success, <0 on error
	 */
	int restartSurveyIn();

	/**
	 * Parse the binary UBX packet
	 *  0 = decoding, 1 = message handled, 2 = sat info message handled
	 */
	int parseChar(const uint8_t b);

	/**
	 * Start payload rx
	 * -1 = abort, 0 = continue
	 */
	int payloadRxInit(void);

	/**
	 * Add payload rx byte
	 *  -1 = error, 0 = ok, 1 = payload completed
	 */
	int payloadRxAdd(const uint8_t b);

	/**
	 * Add MON-VER payload rx byte
	 *  -1 = error, 0 = ok, 1 = payload completed
	 */
	int payloadRxAddMonVer(const uint8_t b);

	// -1 = error, 0 = ok, 1 = payload completed
	int payloadRxAddNavSat(const uint8_t b);

	/**
	 * Add NAV-SVINFO payload rx byte
	 *  -1 = error, 0 = ok, 1 = payload completed
	 */
	int payloadRxAddNavSvinfo(const uint8_t b);

	/**
	 * Finish payload rx
	 *  0 = no message handled, 1 = message handled, 2 = sat info message handled
	 */
	int payloadRxDone(void);

	/**
	 * Send a message
	 * @return true on success, false on write error (errno set)
	 */
	bool sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length);

	/**
	 * Wait for message acknowledge
	 */
	int waitForAck(const uint16_t msg, const unsigned timeout, const bool report);

	const Interface _interface{};

	hrt_abstime _disable_cmd_last{0};

	ubx_ack_state_t         _ack_state{UBX_ACK_IDLE};
	ubx_buf_t               _buf{};
	ubx_decode_state_t      _decode_state{};
	ubx_rxmsg_state_t       _rx_state{UBX_RXMSG_IGNORE};

	bool _configured{false};
	bool _got_posllh{false};
	bool _got_velned{false};
	bool _use_nav_pvt{false};

	uint8_t _rx_ck_a{0};
	uint8_t _rx_ck_b{0};
	uint8_t _dyn_model{7};  ///< ublox Dynamic platform model default 7: airborne with <2g acceleration

	uint16_t _ack_waiting_msg{0};
	uint16_t _rx_msg{};
	uint16_t _rx_payload_index{0};
	uint16_t _rx_payload_length{0};

	uint32_t _ubx_version{0};

	uint64_t _last_timestamp_time{0};

	Board _board{Board::unknown};

	OutputMode _output_mode{OutputMode::GPS};

	RTCMParsing *_rtcm_parsing{nullptr};

	const UBXMode _mode;
	const float _heading_offset;
};
