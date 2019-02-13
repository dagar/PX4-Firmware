/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "mixer.h"

#include <matrix/math.hpp>


/**
 * Supported multirotor geometries.
 *
 * Values are generated by the px_generate_mixers.py script and placed to mixer_multirotor_normalized.generated.h
 */
// typedef unsigned int MultirotorGeometryUnderlyingType;
// enum class MultirotorGeometry : MultirotorGeometryUnderlyingType;

/**
 * Multi-rotor mixer for pre-defined vehicle geometries.
 *
 * Collects six inputs (roll, pitch, yaw, x thrust, y thrust, z thrust) and mixes them to
 * a set of outputs based on the configured geometry.
 */
class __EXPORT MultirotorMixer6dof : public Mixer
{
public:

	/**
	 * Convention for command order
	 */
	enum Command {
		ROLL_COMMAND 	= 0,
		PITCH_COMMAND	= 1,
		YAW_COMMAND		= 2,
		X_COMMAND		= 3,
		Y_COMMAND		= 4,
		Z_COMMAND		= 5,

		COMMAND_MAX		= 6,
	};

	/**
	 * Precalculated rotor mix.
	 */
	struct Rotor {
		float scale[COMMAND_MAX];	 /**< scales on roll, pitch, yaw, x thrust, y thrust, z thrust for this rotor */
	};

	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked to read inputs.
	 * @param cb_handle		Passed to control_cb.
	 * @param geometry		The selected geometry.
	 * @param roll_scale	Scaling factor applied to roll
	 * @param pitch_scale	Scaling factor applied to pitch
	 * @param yaw_scale		Scaling factor applied to yaw inputs
	 * @param x_scale		Scaling factor applied to x thrust inputs
	 * @param y_scale		Scaling factor applied to y thrust inputs
	 * @param z_scale		Scaling factor applied to z thrust inputs
	 * @param idle_speed	Minimum rotor control output value; usually
	 *						tuned to ensure that rotors never stall at the
	 * 						low end of their control range.
	 */
	MultirotorMixer6dof(ControlCallback control_cb,
			    uintptr_t cb_handle,
			    MultirotorGeometry geometry,
			    float roll_scale,
			    float pitch_scale,
			    float yaw_scale,
			    float x_scale,
			    float y_scale,
			    float z_scale,
			    float idle_speed);

	~MultirotorMixer6dof();

	/**
	 * Factory method.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new MultirotorMixer6dof instance, or nullptr
	 *				if the text format is bad.
	 */
	static MultirotorMixer6dof		*from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf,
			unsigned &buflen);

	virtual unsigned		mix(float *outputs, unsigned space);
	virtual uint16_t		get_saturation_status(void);
	virtual void			groups_required(uint32_t &groups);

	/**
	 * @brief      Update slew rate parameter. This tells the multicopter mixer
	 *             the maximum allowed change of the output values per cycle.
	 *             The value is only valid for one cycle, in order to have continuous
	 *             slew rate limiting this function needs to be called before every call
	 *             to mix().
	 *
	 * @param[in]  delta_out_max  Maximum delta output.
	 *
	 */
	virtual void 			set_max_delta_out_once(float delta_out_max) { _delta_out_max = delta_out_max; }

	unsigned			set_trim(float trim) { return _rotor_count; }

	/**
	 * @brief      Sets the thrust factor used to calculate mapping from desired thrust to pwm.
	 *
	 * @param[in]  val   The value
	 */
	virtual void			set_thrust_factor(float val) { _thrust_factor = val; }

private:
	float				_roll_scale;
	float				_pitch_scale;
	float				_yaw_scale;
	float				_x_scale;
	float				_y_scale;
	float				_z_scale;
	float				_idle_speed;
	float				_out_max;
	float 				_out_min;
	float 				_delta_out_max;
	float 				_thrust_factor;

	bool 				_controlled_axes[6]; 	// for underactuated systems, keeps track of which axes are controllable

	void update_saturation_status(unsigned index, bool clipping_high, bool clipping_low);
	MultirotorMixer::saturation_status _saturation_status;

	unsigned			_rotor_count;
	const Rotor			*_rotors;

	matrix::Vector<float, 6> get_command(void) const;
	matrix::Vector<float, 6> clip_command(const matrix::Vector<float, 6> &command) const;

	float 				*_outputs_prev{nullptr};

};
