/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file KFxyzDecoupledStatic.h
 * Simple Kalman Filter for variable gain low-passing
 *
 * State: [r, r_dot, bias]
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>

#include "target_estimator.h"

#pragma once

namespace landing_target_estimator
{
class KFxyzDecoupledStatic : public TargetEstimator
{
public:
	/**
	 * Default constructor, state not initialized
	 */
	KFxyzDecoupledStatic() = default;

	/**
	 * Default desctructor
	 */
	virtual ~KFxyzDecoupledStatic() = default;

	//Prediction step:
	void predictState(float dt, float acc) override;
	void predictCov(float dt) override;

	// Backwards state prediciton
	void syncState(float dt, float acc) override;

	void setH(matrix::Vector<float, 12> h_meas) override;

	virtual float computeInnovCov(float measUnc) override;
	virtual float computeInnov(float meas) override;

	bool update() override;

	// Init: x_0
	void setPosition(float pos) override { _state(0, 0) = pos; };
	void setVelocity(float vel) override { _state(1, 0) = vel; };

	// Init: P_0
	void setStatePosVar(float pos_unc) override { _covariance(0, 0) = pos_unc; };
	void setStateVelVar(float vel_unc) override { _covariance(1, 1) = vel_unc; };
	void setStateBiasVar(float bias_unc) override { _covariance(2, 2) = bias_unc; };

	// Retreive output of filter
	float getPosition() override { return _state(0, 0); };
	float getVelocity() override { return _state(1, 0); };

	float getPosVar() override { return _covariance(0, 0); };
	float getVelVar() override { return _covariance(1, 1); };

	void setInputAccVar(float var) override { _input_var = var; };
	void setBiasVar(float var) override { _bias_var = var; };

	/* Unused functions:  */
	void setTargetAcc(float acc) override {}
	void setStateAccVar(float acc_unc) override {};
	float getAcceleration() override { return 0.f; };
	float getAccVar() override { return 0.f; };
	void setTargetAccVar(float var) override {};
	void computeDynamicMatrix(float dt) override {};
	void computeProcessNoise(float dt) override {};

	// For orientation filter
	void setH(float h_meas) override {};

private:
	matrix::Matrix<float, 3, 1> _state; // state

	matrix::Matrix<float, 3, 1> _sync_state; // state

	matrix::Matrix<float, 1, 3> _meas_matrix; // row of measurement matrix

	matrix::Matrix<float, 3, 3> _covariance; // state covariance

	float _bias_var{0.f}; // target/UAV GPS bias variance

	float _input_var{0.f}; // UAV acceleration variance

	float _innov{0.0f}; // residual of last measurement update

	float _innov_cov{0.0f}; // innovation covariance of last measurement update
};
} // namespace landing_target_estimator
