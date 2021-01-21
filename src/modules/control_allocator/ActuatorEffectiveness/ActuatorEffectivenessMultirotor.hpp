/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessMultirotor.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"

class ActuatorEffectivenessMultirotor: public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessMultirotor() = default;
	~ActuatorEffectivenessMultirotor() override = default;

	static constexpr int NUM_ROTORS_MAX = 8;

	struct RotorGeometry {
		matrix::Vector3f position{};
		matrix::Vector3f axis{};
		float thrust_coef{0.f};
		float moment_ratio{0.f};
	};

	struct MultirotorGeometry {
		RotorGeometry rotors[NUM_ROTORS_MAX];
	};

	static int computeEffectivenessMatrix(const MultirotorGeometry &geometry,
					      Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness);

	bool getEffectivenessMatrix(Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix) override;

	int numActuators() const override { return _num_actuators; }
private:

};
