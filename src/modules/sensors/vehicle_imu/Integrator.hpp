/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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
 * @file Integrator.hpp
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

class Integrator
{
public:
	Integrator(bool coning_compensation = false) : _coning_comp_on(coning_compensation) {}
	~Integrator() = default;

	/**
	 * Put an item into the integral.
	 *
	 * @param dt     	Delta time for current value
	 * @param val		Item to put.
	 * @return		true if data was accepted and integrated.
	 */
	bool put(const float dt, const matrix::Vector3f &val)
	{
		// Use trapezoidal integration to calculate the delta integral
		const matrix::Vector3f delta_alpha{(val + _last_val) *dt * 0.5f};
		_last_val = val;
		_integral_dt_s += dt;
		_integrated_samples++;

		// Calculate coning corrections if required
		if (_coning_comp_on) {
			// Coning compensation derived by Paul Riseborough and Jonathan Challinger,
			// following:
			// Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
			// Sourced: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
			// Simulated: https://github.com/priseborough/InertialNav/blob/master/models/imu_error_modelling.m
			_beta += ((_last_alpha + _last_delta_alpha * (1.f / 6.f)) % delta_alpha) * 0.5f;
			_last_delta_alpha = delta_alpha;
			_last_alpha = _alpha;
		}

		// accumulate delta integrals
		_alpha += delta_alpha;

		return true;
	}

	/**
	 * Set reset interval during runtime. This won't reset the integrator.
	 *
	 * @param reset_interval	    	New reset time interval for the integrator.
	 */
	void set_reset_interval(uint32_t reset_interval) { _reset_interval_min = reset_interval; }

	/**
	 * Set required samples for reset. This won't reset the integrator.
	 *
	 * @param reset_samples	    	New reset time interval for the integrator.
	 */
	void set_reset_samples(uint8_t reset_samples) { _reset_samples_min = reset_samples; }
	uint8_t reset_samples() const { return _reset_samples_min; }

	/**
	 * Is the Integrator ready to reset?
	 *
	 * @return		true if integrator has sufficient data (minimum interval & samples satisfied) to reset.
	 */
	bool integral_ready() const { return (_integrated_samples >= _reset_samples_min) || (_integral_dt_s >= (_reset_interval_min / 1e6f)); }

	/* Reset integrator and return current integral & integration time
	 *
	 * @param integral_dt	Get the dt in us of the current integration.
	 * @return		true if integral valid
	 */
	bool reset(matrix::Vector3f &integral, uint32_t &integral_dt)
	{
		if (integral_ready()) {
			integral = _alpha;
			integral_dt = roundf(_integral_dt_s * 1e6f); // seconds -> microseconds

			// reset
			_alpha.zero();
			_integral_dt_s = 0;
			_integrated_samples = 0;

			// apply coning corrections if required
			if (_coning_comp_on) {
				integral += _beta;
				_beta.zero();
				//_last_alpha.zero(); // TODO: review?
			}

			return true;
		}

		return false;
	}

private:
	matrix::Vector3f _alpha{0.f, 0.f, 0.f};            /**< integrated value before coning corrections are applied */
	matrix::Vector3f _last_alpha{0.f, 0.f, 0.f};       /**< previous value of _alpha */
	matrix::Vector3f _beta{0.f, 0.f, 0.f};             /**< accumulated coning corrections */
	matrix::Vector3f _last_val{0.f, 0.f, 0.f};         /**< previous input */
	matrix::Vector3f _last_delta_alpha{0.f, 0.f, 0.f}; /**< integral from previous previous sampling interval */

	float _integral_dt_s{0};

	uint32_t _reset_interval_min{1}; /**< the interval after which the content will be published and the integrator reset */

	uint8_t _integrated_samples{0};
	uint8_t _reset_samples_min{1};

	const bool _coning_comp_on{false};                       /**< true to turn on coning corrections */
};
