
#pragma once

#include "mixer_module.hpp"

class PWMMixingOutput : public MixingOutput
{
public:
	/**
	 * Constructor
	 * @param param_prefix for min/max/etc. params, e.g. "PWM_MAIN". This needs to match 'param_prefix' in the module.yaml
	 * @param max_num_outputs maximum number of supported outputs
	 * @param interface Parent module for scheduling, parameter updates and callbacks
	 * @param scheduling_policy
	 */
	PWMMixingOutput(const char *param_prefix, uint8_t max_num_outputs, OutputModuleInterface &interface,
			SchedulingPolicy scheduling_policy = MixingOutput::SchedulingPolicy::Auto) :
		MixingOutput(param_prefix, max_num_outputs, interface, scheduling_policy) {}

private:
	void output_limit_calc(const bool armed, const int num_channels, const float outputs[MAX_ACTUATORS]);

	enum class OutputLimitState {
		OFF = 0,
		INIT,
		RAMP,
		ON
	} _output_state{OutputLimitState::INIT};

	hrt_abstime _output_time_armed{0};

	bool _output_ramp_up{false}; ///< if true, motors will ramp up from disarmed to min_output after arming

	const bool _support_esc_calibration{true};
};
