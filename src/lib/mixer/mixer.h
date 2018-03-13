/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mixer.h
 *
 * Generic, programmable, procedural control signal mixers.
 *
 * This library implements a generic mixer interface that can be used
 * by any driver or subsytem that wants to combine several control signals
 * into a single output.
 *
 * Terminology
 * ===========
 *
 * control value
 *	A mixer input value, typically provided by some controlling
 *	component of the system.
 *
 * control group
 * 	A collection of controls provided by a single controlling component.
 *
 * actuator
 *	The mixer output value.
 *
 *
 * Mixing basics
 * =============
 *
 * An actuator derives its value from the combination of one or more
 * control values. Each of the control values is scaled according to
 * the actuator's configuration and then combined to produce the
 * actuator value, which may then be further scaled to suit the specific
 * output type.
 *
 * Internally, all scaling is performed using floating point values.
 * Inputs and outputs are clamped to the range -1.0 to 1.0.
 *
 * control    control   control
 *    |          |         |
 *    v          v         v
 *  scale      scale     scale
 *    |          |         |
 *    |          v         |
 *    +-------> mix <------+
 *               |
 *             scale
 *               |
 *               v
 *              out
 *
 * Scaling
 * -------
 *
 * Each scaler allows the input value to be scaled independently for
 * inputs greater/less than zero. An offset can be applied to the output,
 * as well as lower and upper boundary constraints.
 * Negative scaling factors cause the output to be inverted (negative input
 * produces positive output).
 *
 * Scaler pseudocode:
 *
 * if (input < 0)
 *     output = (input * NEGATIVE_SCALE) + OFFSET
 * else
 *     output = (input * POSITIVE_SCALE) + OFFSET
 *
 * if (output < LOWER_LIMIT)
 *     output = LOWER_LIMIT
 * if (output > UPPER_LIMIT)
 *     output = UPPER_LIMIT
 *
 *
 * Mixing
 * ------
 *
 * Mixing behaviour varies based on the specific mixer class; each
 * mixer class describes its behaviour in more detail.
 *
 *
 * Controls
 * --------
 *
 * The precise assignment of controls may vary depending on the
 * application, but the following assignments should be used
 * when appropriate.  Some mixer classes have specific assumptions
 * about the assignment of controls.
 *
 * control | standard meaning
 * --------+-----------------------
 *     0   | roll
 *     1   | pitch
 *     2   | yaw
 *     3   | primary thrust
 */


#ifndef _SYSTEMLIB_MIXER_MIXER_H
#define _SYSTEMLIB_MIXER_MIXER_H value

#include <px4_config.h>
#include "drivers/drv_mixer.h"

#include "mixer_load.h"

/**
 * Abstract class defining a mixer mixing zero or more inputs to
 * one or more outputs.
 */
class __EXPORT Mixer
{
public:
	/** next mixer in a list */
	Mixer				*_next;

	/**
	 * Fetch a control value.
	 *
	 * @param handle		Token passed when the callback is registered.
	 * @param control_group		The group to fetch the control from.
	 * @param control_index		The group-relative index to fetch the control from.
	 * @param control		The returned control
	 * @return			Zero if the value was fetched, nonzero otherwise.
	 */
	typedef int	(* ControlCallback)(uintptr_t handle,
					    uint8_t control_group,
					    uint8_t control_index,
					    float &control);

	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked when reading controls.
	 */
	Mixer(ControlCallback control_cb, uintptr_t cb_handle);
	virtual ~Mixer() {}

	/**
	 * Perform the mixing function.
	 *
	 * @param outputs		Array into which mixed output(s) should be placed.
	 * @param space			The number of available entries in the output array;
	 * @return			The number of entries in the output array that were populated.
	 */
	virtual unsigned		mix(float *outputs, unsigned space) = 0;

	/**
	 * Get the saturation status.
	 *
	 * @return			Integer bitmask containing saturation_status from multirotor_motor_limits.msg.
	 */
	virtual uint16_t		get_saturation_status(void) = 0;

	/**
	 * Analyses the mix configuration and updates a bitmask of groups
	 * that are required.
	 *
	 * @param groups		A bitmask of groups (0-31) that the mixer requires.
	 */
	virtual void			groups_required(uint32_t &groups) = 0;

	/**
	 * @brief      Empty method, only implemented for MultirotorMixer and MixerGroup class.
	 *
	 * @param[in]  delta_out_max  Maximum delta output.
	 *
	 */
	virtual void 			set_max_delta_out_once(float delta_out_max) {}

	/**
	 * @brief Set trim offset for this mixer
	 *
	 * @return the number of outputs this mixer feeds to
	 */
	virtual unsigned set_trim(float trim) = 0;

	/**
	 * @brief Get trim offset for this mixer
	 *
	 * @return the number of outputs this mixer feeds to
	 */
	virtual unsigned get_trim(float *trim) = 0;

	/*
	 * @brief      Sets the thrust factor used to calculate mapping from desired thrust to pwm.
	 *
	 * @param[in]  val   The value
	 */
	virtual void 			set_thrust_factor(float val) {}

protected:
	/** client-supplied callback used when fetching control values */
	ControlCallback			_control_cb;
	uintptr_t			_cb_handle;

	/**
	 * Invoke the client callback to fetch a control value.
	 *
	 * @param group			Control group to fetch from.
	 * @param index			Control index to fetch.
	 * @return			The control value.
	 */
	float				get_control(uint8_t group, uint8_t index);

	/**
	 * Perform simpler linear scaling.
	 *
	 * @param scaler		The scaler configuration.
	 * @param input			The value to be scaled.
	 * @return			The scaled value.
	 */
	static float			scale(const mixer_scaler_s &scaler, float input);

	/**
	 * Validate a scaler
	 *
	 * @param scaler		The scaler to be validated.
	 * @return			Zero if good, nonzero otherwise.
	 */
	static int			scale_check(struct mixer_scaler_s &scaler);

	/**
	 * Find a tag
	 *
	 * @param buf			The buffer to operate on.
	 * @param buflen		length of the buffer.
	 * @param tag			character to search for.
	 */
	static const char 		*findtag(const char *buf, unsigned &buflen, char tag);

	/**
	 * Skip a line
	 *
	 * @param buf			The buffer to operate on.
	 * @param buflen		length of the buffer.
	 * @return			0 / OK if a line could be skipped, 1 else
	 */
	static const char 		*skipline(const char *buf, unsigned &buflen);

	/**
	 * Check wether the string is well formed and suitable for parsing
	 */
	static bool				string_well_formed(const char *buf, unsigned &buflen);

private:

	/* do not allow to copy due to pointer data members */
	Mixer(const Mixer &);
	Mixer &operator=(const Mixer &);
};

/**
 * Simple summing mixer.
 *
 * Collects zero or more inputs and mixes them to a single output.
 */
class __EXPORT SimpleMixer : public Mixer
{
public:
	/**
	 * Constructor
	 *
	 * @param mixinfo		Mixer configuration.  The pointer passed
	 *				becomes the property of the mixer and
	 *				will be freed when the mixer is deleted.
	 */
	SimpleMixer(ControlCallback control_cb,
		    uintptr_t cb_handle,
		    mixer_simple_s *mixinfo);
	~SimpleMixer();

	/**
	 * Factory method with full external configuration.
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
	 * @return			A new SimpleMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static SimpleMixer		*from_text(Mixer::ControlCallback control_cb,
			uintptr_t cb_handle,
			const char *buf,
			unsigned &buflen);

	/**
	 * Factory method for PWM/PPM input to internal float representation.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param input			The control index used when fetching the input.
	 * @param min			The PWM/PPM value considered to be "minimum" (gives -1.0 out)
	 * @param mid			The PWM/PPM value considered to be the midpoint (gives 0.0 out)
	 * @param max			The PWM/PPM value considered to be "maximum" (gives 1.0 out)
	 * @return			A new SimpleMixer instance, or nullptr if one could not be
	 *				allocated.
	 */
	static SimpleMixer		*pwm_input(Mixer::ControlCallback control_cb, uintptr_t cb_handle, unsigned input, uint16_t min,
			uint16_t mid, uint16_t max);

	virtual unsigned		mix(float *outputs, unsigned space);
	virtual uint16_t		get_saturation_status(void);
	virtual void			groups_required(uint32_t &groups);

	/**
	 * Check that the mixer configuration as loaded is sensible.
	 *
	 * Note that this function will call control_cb, but only cares about
	 * error returns, not the input value.
	 *
	 * @return			Zero if the mixer makes sense, nonzero otherwise.
	 */
	int				check();

	unsigned set_trim(float trim);

	unsigned get_trim(float *trim);

protected:

private:
	mixer_simple_s			*_pinfo;

	static int			parse_output_scaler(const char *buf, unsigned &buflen, mixer_scaler_s &scaler);
	static int			parse_control_scaler(const char *buf,
			unsigned &buflen,
			mixer_scaler_s &scaler,
			uint8_t &control_group,
			uint8_t &control_index);

	/* do not allow to copy due to ptr data members */
	SimpleMixer(const SimpleMixer &);
	SimpleMixer operator=(const SimpleMixer &);
};

#include "HelicopterMixer.hpp"
#include "MixerGroup.hpp"
#include "MultirotorMixer.hpp"
#include "NullMixer.hpp"

#endif
