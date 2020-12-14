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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>

#include <float.h>

using namespace matrix;
using namespace time_literals;

using math::constrain;

namespace sensors
{

static constexpr int32_t sum(const int16_t samples[32], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

VehicleIMU::VehicleIMU(int instance, bool accel_fifo, uint8_t accel_index, bool gyro_fifo, uint8_t gyro_index,
		       const px4::wq_config_t &config) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_sensor_accel_sub(this, accel_fifo ? ORB_ID(sensor_accel_fifo) : ORB_ID(sensor_accel), accel_index),
	_sensor_gyro_sub(this, gyro_fifo ? ORB_ID(sensor_gyro_fifo) : ORB_ID(sensor_gyro), gyro_index),
	_instance(instance)
{
	_accel.fifo = accel_fifo;
	_gyro.fifo = gyro_fifo;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// currently with lockstep every raw sample needs a corresponding vehicle_imu publication
	_sensor_accel_sub.set_required_updates(1);
	_sensor_gyro_sub.set_required_updates(1);
#else
	// schedule conservatively until the actual accel & gyro rates are known
	_sensor_accel_sub.set_required_updates(sensor_accel_s::ORB_QUEUE_LENGTH / 2);
	_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH / 2);
#endif

	// advertise immediately to ensure consistent ordering
	_vehicle_imu_pub.advertise();
	_vehicle_imu_status_pub.advertise();
}

VehicleIMU::~VehicleIMU()
{
	Stop();

	perf_free(_accel_generation_gap_perf);
	perf_free(_accel_update_perf);
	perf_free(_gyro_generation_gap_perf);
	perf_free(_gyro_update_perf);

	_vehicle_imu_pub.unadvertise();
	_vehicle_imu_status_pub.unadvertise();
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_sensor_gyro_sub.registerCallback();
	_sensor_accel_sub.registerCallback();
	ScheduleNow();
	return true;
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	_sensor_accel_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();

	Deinit();
}

void VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		const auto imu_integ_rate_prev = _param_imu_integ_rate.get();

		updateParams();

		_accel_calibration.ParametersUpdate();
		_gyro_calibration.ParametersUpdate();

		// constrain IMU integration time 1-10 milliseconds (100-1000 Hz)
		int32_t imu_integration_rate_hz = constrain(_param_imu_integ_rate.get(),
						  100, math::max(_param_imu_gyro_ratemax.get(), 1000));

		if (imu_integration_rate_hz != _param_imu_integ_rate.get()) {
			PX4_WARN("IMU_INTEG_RATE updated %d -> %d", _param_imu_integ_rate.get(), imu_integration_rate_hz);
			_param_imu_integ_rate.set(imu_integration_rate_hz);
			_param_imu_integ_rate.commit_no_notification();
		}

		_imu_integration_interval_us = 1000000 / imu_integration_rate_hz;

		if (_param_imu_integ_rate.get() != imu_integ_rate_prev) {
			// force update
			UpdateIntegratorConfiguration();
		}
	}
}

bool VehicleIMU::UpdateIntervalAverage(IntervalAverage &intavg, const hrt_abstime &timestamp_sample, uint8_t samples)
{
	bool updated = false;

	// conservative maximum time between samples to reject large gaps and reset averaging
	float max_interval_us = 10000; // 100 Hz
	float min_interval_us = 100;   // 10,000 Hz

	if (intavg.update_interval > 0) {
		// if available use previously calculated interval as bounds
		max_interval_us = 1.5f * intavg.update_interval;
		min_interval_us = 0.5f * intavg.update_interval;
	}

	const float interval_us = (timestamp_sample - intavg.timestamp_sample_last);

	if ((intavg.timestamp_sample_last > 0) && (interval_us < max_interval_us) && (interval_us > min_interval_us)) {

		intavg.interval_sum += interval_us;
		intavg.interval_count += samples;

		// periodically calculate sensor update rate
		if (intavg.interval_count > 10000 || ((intavg.update_interval <= FLT_EPSILON) && intavg.interval_count > 100)) {

			const float sample_interval_avg = intavg.interval_sum / intavg.interval_count;

			if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.f)) {
				// update if interval has changed by more than 0.5%
				if ((fabsf(intavg.update_interval - sample_interval_avg) / intavg.update_interval) > 0.005f) {

					intavg.update_interval = sample_interval_avg;
					updated = true;
				}
			}

			// reset sample interval accumulator
			intavg.interval_sum = 0.f;
			intavg.interval_count = 0.f;
		}

	} else {
		// reset
		intavg.interval_sum = 0.f;
		intavg.interval_count = 0.f;
	}

	intavg.timestamp_sample_last = timestamp_sample;

	return updated;
}

void VehicleIMU::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	ParametersUpdate();

	if (!_accel_calibration.enabled() || !_gyro_calibration.enabled()) {
		return;
	}

	if (_gyro.fifo) {
		UpdateGyroFifo();

	} else {
		UpdateGyro();
	}

	if (_accel.fifo) {
		UpdateAccelFifo();

	} else {
		UpdateAccel();
	}

	if (_sensor_data_gap) {
		_consecutive_data_gap++;

		// if there's consistently a gap in data start monitoring publication interval again
		if (_consecutive_data_gap > 10) {
			_intervals_configured = false;
		}

	} else {
		_consecutive_data_gap = 0;
	}

	// reconfigure integrators if calculated sensor intervals have changed
	if (_update_integrator_config) {
		UpdateIntegratorConfiguration();
	}

	Publish();
}

void VehicleIMU::Publish()
{
	// publish if both accel & gyro integrators are ready
	if (_accel.integral_ready() && _gyro.integral_ready()) {

		uint32_t accel_integral_dt;
		uint32_t gyro_integral_dt;
		Vector3f delta_angle;
		Vector3f delta_velocity;

		if (_accel.integrator_reset(delta_velocity, accel_integral_dt)
		    && _gyro.integrator_reset(delta_angle, gyro_integral_dt)) {

			// delta angle: apply offsets, scale, and board rotation
			_gyro_calibration.SensorCorrectionsUpdate();
			const float gyro_dt_inv = 1.e6f / gyro_integral_dt;
			const Vector3f delta_angle_corrected{_gyro_calibration.Correct(delta_angle * gyro_dt_inv) / gyro_dt_inv};

			// delta velocity: apply offsets, scale, and board rotation
			_accel_calibration.SensorCorrectionsUpdate();
			const float accel_dt_inv = 1.e6f / accel_integral_dt;
			Vector3f delta_velocity_corrected{_accel_calibration.Correct(delta_velocity * accel_dt_inv) / accel_dt_inv};

			UpdateAccelVibrationMetrics(delta_velocity_corrected);
			UpdateGyroVibrationMetrics(delta_angle_corrected);

			_accel.data_sum += delta_velocity_corrected;
			_accel.sum_count++;
			_gyro.data_sum += delta_angle_corrected;
			_gyro.sum_count++;

			// vehicle_imu_status
			//  publish before vehicle_imu so that error counts are available synchronously if needed
			if ((_accel_gyro_sum_count > 0) && (_publish_status || (hrt_elapsed_time(&_status.timestamp) >= 100_ms))) {
				// accel
				_status.accel_device_id = _accel_calibration.device_id();
				const Vector3f accel_mean{_accel.data_sum / _accel_gyro_sum_count};
				accel_mean.copyTo(_status.mean_accel);
				_status.temperature_accel = _accel.temperature_sum / _accel.temperature_sum_count;

				// gyro
				_status.gyro_device_id = _gyro_calibration.device_id();
				const Vector3f gyro_mean{_gyro.data_sum / _accel_gyro_sum_count};
				gyro_mean.copyTo(_status.mean_gyro);
				_status.temperature_gyro = _gyro.temperature_sum / _gyro.temperature_sum_count;

				_status.timestamp = hrt_absolute_time();
				_vehicle_imu_status_pub.publish(_status);

				// reset
				_accel.data_sum.zero();
				_accel.temperature_sum = 0;
				_accel.temperature_sum_count = 0;
				_accel.sum_count = 0;

				_gyro.data_sum.zero();
				_gyro.temperature_sum = 0;
				_gyro.temperature_sum_count = 0;
				_accel.sum_count = 0;
			}


			// publish vehicle_imu
			vehicle_imu_s imu;
			imu.timestamp_sample = _gyro.last_timestamp_sample;
			imu.accel_device_id = _accel_calibration.device_id();
			imu.gyro_device_id = _gyro_calibration.device_id();
			delta_angle_corrected.copyTo(imu.delta_angle);
			delta_velocity_corrected.copyTo(imu.delta_velocity);
			imu.delta_angle_dt = gyro_integral_dt;
			imu.delta_velocity_dt = accel_integral_dt;
			imu.delta_velocity_clipping = _delta_velocity_clipping;
			imu.calibration_count = _accel_calibration.calibration_count() + _gyro_calibration.calibration_count();
			imu.timestamp = hrt_absolute_time();
			_vehicle_imu_pub.publish(imu);

			// reset clip counts
			_delta_velocity_clipping = 0;

			return;
		}
	}
}

void VehicleIMU::UpdateAccel()
{
	// update accel, stopping once caught up to the last gyro sample
	sensor_accel_s accel;

	while (_sensor_accel_sub.update(&accel)) {
		perf_count_interval(_accel_update_perf, accel.timestamp_sample);

		if (_sensor_accel_sub.get_last_generation() != _accel.last_generation + 1) {
			_sensor_data_gap = true;
			perf_count(_accel_generation_gap_perf);

			_accel.interval.timestamp_sample_last = 0; // invalidate any ongoing publication rate averaging

		} else {
			// collect sample interval average for filters
			if (!_intervals_configured && UpdateIntervalAverage(_accel.interval, accel.timestamp_sample)) {
				_update_integrator_config = true;
				_publish_status = true;
				_status.accel_rate_hz = 1e6f / _accel.interval.update_interval;
			}
		}

		_accel.last_generation = _sensor_accel_sub.get_last_generation();

		_accel_calibration.set_device_id(accel.device_id);

		if (accel.error_count != _status.accel_error_count) {
			_publish_status = true;
			_status.accel_error_count = accel.error_count;
		}

		_accel.temperature_sum += accel.temperature;
		_accel.temperature_sum_count++;

		_accel.last_timestamp_sample = accel.timestamp_sample;

		// if (accel.clip_counter[0] > 0 || accel.clip_counter[1] > 0 || accel.clip_counter[2] > 0) {

		// 	// rotate sensor clip counts into vehicle body frame
		// 	const Vector3f clipping{_accel_calibration.rotation() *
		// 				Vector3f{(float)accel.clip_counter[0], (float)accel.clip_counter[1], (float)accel.clip_counter[2]}};

		// 	// round to get reasonble clip counts per axis (after board rotation)
		// 	const uint8_t clip_x = roundf(fabsf(clipping(0)));
		// 	const uint8_t clip_y = roundf(fabsf(clipping(1)));
		// 	const uint8_t clip_z = roundf(fabsf(clipping(2)));

		// 	_status.accel_clipping[0] += clip_x;
		// 	_status.accel_clipping[1] += clip_y;
		// 	_status.accel_clipping[2] += clip_z;

		// 	if (clip_x > 0) {
		// 		_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_X;
		// 	}

		// 	if (clip_y > 0) {
		// 		_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Y;
		// 	}

		// 	if (clip_z > 0) {
		// 		_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Z;
		// 	}

		// 	publish_status = true;
		// }

		// break once caught up to gyro
		if (!_sensor_data_gap && _intervals_configured
		    && (_accel.last_timestamp_sample >= (_gyro.last_timestamp_sample - 0.5f * _accel.interval.update_interval))) {

			break;
		}
	}
}

void VehicleIMU::UpdateAccelFifo()
{
	// integrate queued gyro FIFO data
	sensor_gyro_fifo_s sample;

	while (_sensor_accel_sub.update(&sample)) {
		perf_count_interval(_accel_update_perf, sample.timestamp_sample);

		const uint8_t N = sample.samples;

		if (_sensor_accel_sub.get_last_generation() != _accel.last_generation + 1) {
			_sensor_data_gap = true;
			perf_count(_accel_generation_gap_perf);

			_accel.interval.timestamp_sample_last = 0; // invalidate any ongoing publication rate averaging

		} else {
			// collect sample interval average for filters
			if (!_intervals_configured && UpdateIntervalAverage(_accel.interval, sample.timestamp_sample, N)) {
				_update_integrator_config = true;
				_publish_status = true;
				_status.gyro_rate_hz = 1e6f / _accel.interval.update_interval;
			}
		}

		_accel.last_generation = _sensor_accel_sub.get_last_generation();

		_accel_calibration.set_device_id(sample.device_id);

		if (sample.error_count != _status.accel_error_count) {
			_publish_status = true;
			_status.accel_error_count = sample.error_count;
		}

		// trapezoidal integration (equally spaced, scaled by dt later)
		_accel.integrated_time_us += sample.dt;
		_accel.integrated_samples += N;
		_accel.integrated_data += Vector3f{
			(0.5f * (_accel.last_sample_fifo[0] + sample.x[N - 1]) + sum(sample.x, N - 1)),
			(0.5f * (_accel.last_sample_fifo[1] + sample.y[N - 1]) + sum(sample.y, N - 1)),
			(0.5f * (_accel.last_sample_fifo[2] + sample.z[N - 1]) + sum(sample.z, N - 1)),
		};

		_accel.last_sample_fifo[0] = sample.x[N - 1];
		_accel.last_sample_fifo[1] = sample.y[N - 1];
		_accel.last_sample_fifo[2] = sample.z[N - 1];


		// // clipping
		// uint8_t clip_count[3] {
		// 	clipping(sample.x, _clip_limit, N),
		// 	clipping(sample.y, _clip_limit, N),
		// 	clipping(sample.z, _clip_limit, N),
		// };

		_accel.temperature_sum += sample.temperature;
		_accel.temperature_sum_count++;

		_accel.last_timestamp_sample = sample.timestamp_sample;

		// break if interval is configured and we haven't fallen behind
		if (_intervals_configured && _accel.integral_ready()
		    && (hrt_elapsed_time(&sample.timestamp) < _imu_integration_interval_us) && !_sensor_data_gap) {

			return;
		}
	}
}

void VehicleIMU::UpdateGyro()
{
	// integrate queued gyro
	sensor_gyro_s gyro;

	while (_sensor_gyro_sub.update(&gyro)) {
		perf_count_interval(_gyro_update_perf, gyro.timestamp_sample);

		if (_sensor_gyro_sub.get_last_generation() != _gyro.last_generation + 1) {
			_sensor_data_gap = true;
			perf_count(_gyro_generation_gap_perf);

			_gyro.interval.timestamp_sample_last = 0; // invalidate any ongoing publication rate averaging

		} else {
			// collect sample interval average for filters
			if (!_intervals_configured && UpdateIntervalAverage(_gyro.interval, gyro.timestamp_sample)) {
				_update_integrator_config = true;
				_publish_status = true;
				_status.gyro_rate_hz = 1e6f / _gyro.interval.update_interval;
			}
		}

		_gyro.last_generation = _sensor_gyro_sub.get_last_generation();

		_gyro_calibration.set_device_id(gyro.device_id);

		if (gyro.error_count != _status.gyro_error_count) {
			_publish_status = true;
			_status.gyro_error_count = gyro.error_count;
		}

		_gyro.temperature_sum += gyro.temperature;
		_gyro.temperature_sum_count++;

		_gyro_integrator.put(gyro.timestamp_sample - _gyro.last_timestamp_sample, Vector3f{gyro.x, gyro.y, gyro.z});
		_gyro.last_timestamp_sample = gyro.timestamp_sample;

		// break if interval is configured and we haven't fallen behind
		if (_intervals_configured && _gyro.integral_ready()
		    && (hrt_elapsed_time(&gyro.timestamp) < _imu_integration_interval_us) && !_sensor_data_gap) {

			break;
		}
	}
}

void VehicleIMU::UpdateGyroFifo()
{
	// integrate queued gyro FIFO data
	sensor_gyro_fifo_s sample;

	while (_sensor_gyro_sub.update(&sample)) {
		perf_count_interval(_gyro_update_perf, sample.timestamp_sample);

		const uint8_t N = sample.samples;

		if (_sensor_gyro_sub.get_last_generation() != _gyro.last_generation + 1) {
			_sensor_data_gap = true;
			perf_count(_gyro_generation_gap_perf);

			_gyro.interval.timestamp_sample_last = 0; // invalidate any ongoing publication rate averaging

		} else {
			// collect sample interval average for filters
			if (!_intervals_configured && UpdateIntervalAverage(_gyro.interval, sample.timestamp_sample, N)) {
				_update_integrator_config = true;
				_publish_status = true;
				_status.gyro_rate_hz = 1e6f / _gyro.interval.update_interval;
			}
		}

		_gyro.last_generation = _sensor_gyro_sub.get_last_generation();

		_gyro_calibration.set_device_id(sample.device_id);

		if (sample.error_count != _status.gyro_error_count) {
			_publish_status = true;
			_status.gyro_error_count = sample.error_count;
		}

		const float dt_s = sample.dt * 1e-6f;

		for (int n = 0; n < N; n++) {
			_gyro_integrator.put(dt_s, Vector3f{(float)sample.x[n], (float)sample.y[n], (float)sample.z[n]});
		}

		_gyro.temperature_sum += sample.temperature;
		_gyro.temperature_sum_count++;

		_gyro.last_timestamp_sample = sample.timestamp_sample;

		// break if interval is configured and we haven't fallen behind
		if (_intervals_configured && _gyro.integral_ready()
		    && (hrt_elapsed_time(&sample.timestamp) < _imu_integration_interval_us) && !_sensor_data_gap) {

			return;
		}
	}
}

void VehicleIMU::UpdateIntegratorConfiguration()
{
	if ((_accel.interval.update_interval > 0) && (_gyro.interval.update_interval > 0)) {

		const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();

		// determine number of sensor samples that will get closest to the desired integration interval
		const uint8_t accel_integral_samples = math::max(1.f, roundf(configured_interval_us / _accel.interval.update_interval));
		const uint8_t gyro_integral_samples = math::max(1.f, roundf(configured_interval_us / _gyro.interval.update_interval));

		// let the gyro set the configuration and scheduling
		// accel integrator will be forced to reset when gyro integrator is ready
		_gyro_integrator.set_reset_samples(gyro_integral_samples);
		//_accel_integrator.set_reset_samples(1);

		// relaxed minimum integration time required
		//_accel_integrator.set_reset_interval(roundf((accel_integral_samples - 0.5f) * _accel.interval.update_interval));
		_gyro_integrator.set_reset_interval(roundf((gyro_integral_samples - 0.5f) * _gyro.interval.update_interval));

		// gyro: find largest integer multiple of gyro_integral_samples
		for (int n = sensor_gyro_s::ORB_QUEUE_LENGTH; n > 0; n--) {
			if (gyro_integral_samples % n == 0) {
				_sensor_gyro_sub.set_required_updates(n);

				// run when there are enough new gyro samples, unregister accel
				_sensor_accel_sub.unregisterCallback();

				_intervals_configured = true; // stop monitoring topic publication rates

				PX4_DEBUG("accel (%d), gyro (%d), accel samples: %d, gyro samples: %d, accel interval: %.1f, gyro interval: %.1f sub samples: %d",
					  _accel_calibration.device_id(), _gyro_calibration.device_id(), accel_integral_samples, gyro_integral_samples,
					  (double)_accel.interval.update_interval, (double)_gyro.interval.update_interval, n);

				break;
			}
		}
	}
}

void VehicleIMU::UpdateAccelVibrationMetrics(const Vector3f &delta_velocity)
{
	// Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	const Vector3f delta_velocity_diff = delta_velocity - _delta_velocity_prev;
	_status.accel_vibration_metric = 0.99f * _status.accel_vibration_metric + 0.01f * delta_velocity_diff.norm();

	_delta_velocity_prev = delta_velocity;
}

void VehicleIMU::UpdateGyroVibrationMetrics(const Vector3f &delta_angle)
{
	// Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	const Vector3f delta_angle_diff = delta_angle - _delta_angle_prev;
	_status.gyro_vibration_metric = 0.99f * _status.gyro_vibration_metric + 0.01f * delta_angle_diff.norm();

	// Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	const Vector3f coning_metric = delta_angle % _delta_angle_prev;
	_status.gyro_coning_vibration = 0.99f * _status.gyro_coning_vibration + 0.01f * coning_metric.norm();

	_delta_angle_prev = delta_angle;
}

void VehicleIMU::PrintStatus()
{
	if (_accel_calibration.device_id() == _gyro_calibration.device_id()) {
		PX4_INFO("%d - IMU ID: %d, accel interval: %.1f us, gyro interval: %.1f us", _instance, _accel_calibration.device_id(),
			 (double)_accel.interval.update_interval, (double)_gyro.interval.update_interval);

	} else {
		PX4_INFO("%d - Accel ID: %d, interval: %.1f us, Gyro ID: %d, interval: %.1f us", _instance,
			 _accel_calibration.device_id(),
			 (double)_accel.interval.update_interval, _gyro_calibration.device_id(), (double)_gyro.interval.update_interval);
	}

	perf_print_counter(_accel_generation_gap_perf);
	perf_print_counter(_gyro_generation_gap_perf);
	perf_print_counter(_accel_update_perf);
	perf_print_counter(_gyro_update_perf);

	_accel_calibration.PrintStatus();
	_gyro_calibration.PrintStatus();
}

} // namespace sensors
