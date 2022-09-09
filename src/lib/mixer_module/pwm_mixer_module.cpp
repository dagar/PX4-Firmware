#include "pwm_mixer_module.hpp"

void PWMMixingOutput::output_limit_calc(const bool armed, const int num_channels, const float output[MAX_ACTUATORS])
{
	const bool pre_armed = armNoThrottle();

	// time to slowly ramp up the ESCs
	static constexpr hrt_abstime RAMP_TIME_US = 500_ms;

	/* first evaluate state changes */
	switch (_output_state) {
	case OutputLimitState::INIT:
		if (armed) {
			// set arming time for the first call
			if (_output_time_armed == 0) {
				_output_time_armed = hrt_absolute_time();
			}

			// time for the ESCs to initialize (this is not actually needed if the signal is sent right after boot)
			if (hrt_elapsed_time(&_output_time_armed) >= 50_ms) {
				_output_state = OutputLimitState::OFF;
			}
		}

		break;

	case OutputLimitState::OFF:
		if (armed) {
			if (_output_ramp_up) {
				_output_state = OutputLimitState::RAMP;

			} else {
				_output_state = OutputLimitState::ON;
			}

			// reset arming time, used for ramp timing
			_output_time_armed = hrt_absolute_time();
		}

		break;

	case OutputLimitState::RAMP:
		if (!armed) {
			_output_state = OutputLimitState::OFF;

		} else if (hrt_elapsed_time(&_output_time_armed) >= RAMP_TIME_US) {
			_output_state = OutputLimitState::ON;
		}

		break;

	case OutputLimitState::ON:
		if (!armed) {
			_output_state = OutputLimitState::OFF;
		}

		break;
	}

	/* if the system is pre-armed, the limit state is temporarily on,
	 * as some outputs are valid and the non-valid outputs have been
	 * set to NaN. This is not stored in the state machine though,
	 * as the throttle channels need to go through the ramp at
	 * regular arming time.
	 */
	auto local_limit_state = _output_state;

	if (pre_armed) {
		local_limit_state = OutputLimitState::ON;
	}

	// then set _current_output_value based on state
	switch (local_limit_state) {
	case OutputLimitState::OFF:
	case OutputLimitState::INIT:
		for (int i = 0; i < num_channels; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		break;

	case OutputLimitState::RAMP: {
			hrt_abstime diff = hrt_elapsed_time(&_output_time_armed);

			static constexpr int PROGRESS_INT_SCALING = 10000;
			int progress = diff * PROGRESS_INT_SCALING / RAMP_TIME_US;

			if (progress > PROGRESS_INT_SCALING) {
				progress = PROGRESS_INT_SCALING;
			}

			for (int i = 0; i < num_channels; i++) {

				float control_value = output[i];

				/* check for invalid / disabled channels */
				if (!PX4_ISFINITE(control_value)) {
					_current_output_value[i] = _disarmed_value[i];
					continue;
				}

				uint16_t ramp_min_output;

				/* if a disarmed output value was set, blend between disarmed and min */
				if (_disarmed_value[i] > 0) {

					/* safeguard against overflows */
					auto disarmed = _disarmed_value[i];

					if (disarmed > _min_value[i]) {
						disarmed = _min_value[i];
					}

					int disarmed_min_diff = _min_value[i] - disarmed;
					ramp_min_output = disarmed + (disarmed_min_diff * progress) / PROGRESS_INT_SCALING;

				} else {
					/* no disarmed output value set, choose min output */
					ramp_min_output = _min_value[i];
				}

				if (_reverse_output_mask & (1 << i)) {
					control_value = -1.f * control_value;
				}

				_current_output_value[i] = control_value * (_max_value[i] - ramp_min_output) / 2 + (_max_value[i] + ramp_min_output) /
							   2;

				/* last line of defense against invalid inputs */
				_current_output_value[i] = math::constrain(_current_output_value[i], ramp_min_output, _max_value[i]);
			}
		}
		break;

	case OutputLimitState::ON:
		for (int i = 0; i < num_channels; i++) {
			_current_output_value[i] = output_limit_calc_single(i, output[i]);
		}

		break;
	}
}
