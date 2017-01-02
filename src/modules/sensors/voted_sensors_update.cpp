/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file voted_sensors_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "voted_sensors_update.h"

#include <systemlib/mavlink_log.h>

#include <conversion/rotation.h>

#define MAG_ROT_VAL_INTERNAL		-1
#define CAL_ERROR_APPLY_CAL_MSG "FAILED APPLYING %s CAL #%u"


using namespace sensors;
using namespace DriverFramework;

VotedSensorsUpdate::VotedSensorsUpdate(const Parameters &parameters)
	: _parameters(parameters)
{
	memset(&_last_sensor_data, 0, sizeof(_last_sensor_data));
	memset(&_last_accel_timestamp, 0, sizeof(_last_accel_timestamp));
	memset(&_last_mag_timestamp, 0, sizeof(_last_mag_timestamp));
	memset(&_last_baro_timestamp, 0, sizeof(_last_baro_timestamp));
	memset(&_accel_diff, 0, sizeof(_accel_diff));
	memset(&_gyro_diff, 0, sizeof(_gyro_diff));

	_baro.voter.set_timeout(300000);
	_mag.voter.set_timeout(300000);
	_mag.voter.set_equal_value_threshold(1000);
}

int VotedSensorsUpdate::init(sensor_combined_s &raw)
{
	raw.accelerometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
	raw.magnetometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
	raw.baro_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
	raw.timestamp = 0;

	initialize_sensors();

	// get the parameter handles for the gyro temperature compensation parameters
	sensors_temp_comp::initialize_parameter_handles(_thermal_correction_param_handles);

	// ensure pointer to sensor corrections publiations is defined
	_sensor_correction_pub = nullptr;

	// initialise the corrections
	memset(&_corrections, 0, sizeof(_corrections));
	memset(&_accel_offset, 0, sizeof(_accel_offset));
	memset(&_gyro_offset, 0, sizeof(_gyro_offset));
	memset(&_baro_offset, 0, sizeof(_baro_offset));

	for (unsigned i = 0; i < 3; i++) {
		_corrections.gyro_scale[i] = 1.0f;
		_corrections.accel_scale[i] = 1.0f;
		for (unsigned j = 0; j < SENSOR_COUNT_MAX; j++) {
			_accel_scale[j][i] = 1.0f;
			_gyro_scale[j][i] = 1.0f;
		}
	}

	return 0;
}

void VotedSensorsUpdate::initialize_sensors()
{
	init_sensor_class(ORB_ID(sensor_gyro), _gyro);
	init_sensor_class(ORB_ID(sensor_mag), _mag);
	init_sensor_class(ORB_ID(sensor_accel), _accel);
	init_sensor_class(ORB_ID(sensor_baro), _baro);
}

void VotedSensorsUpdate::deinit()
{
	for (unsigned i = 0; i < _gyro.subscription_count; i++) {
		orb_unsubscribe(_gyro.subscription[i]);
	}

	for (unsigned i = 0; i < _accel.subscription_count; i++) {
		orb_unsubscribe(_accel.subscription[i]);
	}

	for (unsigned i = 0; i < _mag.subscription_count; i++) {
		orb_unsubscribe(_mag.subscription[i]);
	}

	for (unsigned i = 0; i < _baro.subscription_count; i++) {
		orb_unsubscribe(_baro.subscription[i]);
	}
}

void VotedSensorsUpdate::parameters_update()
{
	get_rot_matrix((enum Rotation)_parameters.board_rotation, &_board_rotation);
	/* fine tune board offset */
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _parameters.board_offset[0],
					 M_DEG_TO_RAD_F * _parameters.board_offset[1],
					 M_DEG_TO_RAD_F * _parameters.board_offset[2]);

	_board_rotation = board_rotation_offset * _board_rotation;

	/* set offset parameters to new values */
	bool failed;
	char str[30];
	unsigned mag_count = 0;
	unsigned gyro_count = 0;
	unsigned accel_count = 0;

	/* get stored sensor hub temperature compensation parameters */
	sensors_temp_comp::update_parameters(_thermal_correction_param_handles, _thermal_correction_param);

	/* run through all gyro sensors */
	for (unsigned s = 0; s < SENSOR_COUNT_MAX; s++) {

		(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			continue;
		}

		bool config_ok = false;

		/* run through all stored calibrations that are applied at the driver level*/
		for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_GYRO%u_ID", i);
			int device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			if (failed) {
				DevMgr::releaseHandle(h);
				continue;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == h.ioctl(DEVIOCGDEVICEID, 0)) {
				struct gyro_calibration_s gscale = {};
				(void)sprintf(str, "CAL_GYRO%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_offset));
				(void)sprintf(str, "CAL_GYRO%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_offset));
				(void)sprintf(str, "CAL_GYRO%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.z_offset));
				(void)sprintf(str, "CAL_GYRO%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_scale));
				(void)sprintf(str, "CAL_GYRO%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_scale));
				(void)sprintf(str, "CAL_GYRO%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.z_scale));

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro", i);

				} else {
					/* apply new scaling and offsets */
					config_ok = apply_gyro_calibration(h, &gscale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			gyro_count++;
		}
	}

	/* run through all accel sensors */
	for (unsigned s = 0; s < SENSOR_COUNT_MAX; s++) {

		(void)sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, s);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			continue;
		}

		bool config_ok = false;

		/* run through all stored calibrations */
		for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_ACC%u_ID", i);
			int device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			if (failed) {
				DevMgr::releaseHandle(h);
				continue;
			}

			/* if the calibration is for this device, apply it */
			if (device_id == h.ioctl(DEVIOCGDEVICEID, 0)) {
				struct accel_calibration_s ascale = {};
				(void)sprintf(str, "CAL_ACC%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_offset));
				(void)sprintf(str, "CAL_ACC%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_offset));
				(void)sprintf(str, "CAL_ACC%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.z_offset));
				(void)sprintf(str, "CAL_ACC%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_scale));
				(void)sprintf(str, "CAL_ACC%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_scale));
				(void)sprintf(str, "CAL_ACC%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.z_scale));

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel", i);

				} else {
					/* apply new scaling and offsets */
					config_ok = apply_accel_calibration(h, &ascale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			accel_count++;
		}
	}

	/* run through all mag sensors */
	for (unsigned s = 0; s < SENSOR_COUNT_MAX; s++) {

		/* set a valid default rotation (same as board).
		 * if the mag is configured, this might be replaced
		 * in the section below.
		 */
		_mag_rotation[s] = _board_rotation;

		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, s);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			/* the driver is not running, abort */
			continue;
		}

		bool config_ok = false;

		/* run through all stored calibrations */
		for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_MAG%u_ID", i);
			int device_id;
			failed = failed || (OK != param_get(param_find(str), &device_id));
			(void)sprintf(str, "CAL_MAG%u_ROT", i);
			(void)param_find(str);

			if (failed) {
				DevMgr::releaseHandle(h);
				continue;
			}

			// int id = h.ioctl(DEVIOCGDEVICEID, 0);
			// PX4_WARN("sensors: device ID: %s: %d, %u", str, id, (unsigned)id);

			/* if the calibration is for this device, apply it */
			if (device_id == h.ioctl(DEVIOCGDEVICEID, 0)) {
				struct mag_calibration_s mscale = {};
				(void)sprintf(str, "CAL_MAG%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_offset));
				(void)sprintf(str, "CAL_MAG%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_offset));
				(void)sprintf(str, "CAL_MAG%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_offset));
				(void)sprintf(str, "CAL_MAG%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_scale));
				(void)sprintf(str, "CAL_MAG%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_scale));
				(void)sprintf(str, "CAL_MAG%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_scale));

				(void)sprintf(str, "CAL_MAG%u_ROT", i);

				if (h.ioctl(MAGIOCGEXTERNAL, 0) <= 0) {
					/* mag is internal */
					_mag_rotation[s] = _board_rotation;
					/* reset param to -1 to indicate internal mag */
					int32_t minus_one;
					param_get(param_find(str), &minus_one);

					if (minus_one != MAG_ROT_VAL_INTERNAL) {
						minus_one = MAG_ROT_VAL_INTERNAL;
						param_set_no_notification(param_find(str), &minus_one);
					}

				} else {

					int32_t mag_rot;
					param_get(param_find(str), &mag_rot);

					/* check if this mag is still set as internal */
					if (mag_rot < 0) {
						/* it was marked as internal, change to external with no rotation */
						mag_rot = 0;
						param_set_no_notification(param_find(str), &mag_rot);
					}

					/* handling of old setups, will be removed later (noted Feb 2015) */
					int32_t deprecated_mag_rot = 0;
					param_get(param_find("SENS_EXT_MAG_ROT"), &deprecated_mag_rot);

					/*
					 * If the deprecated parameter is non-default (is != 0),
					 * and the new parameter is default (is == 0), then this board
					 * was configured already and we need to copy the old value
					 * to the new parameter.
					 * The < 0 case is special: It means that this param slot was
					 * used previously by an internal sensor, but the the call above
					 * proved that it is currently occupied by an external sensor.
					 * In that case we consider the orientation to be default as well.
					 */
					if ((deprecated_mag_rot != 0) && (mag_rot <= 0)) {
						mag_rot = deprecated_mag_rot;
						param_set_no_notification(param_find(str), &mag_rot);
						/* clear the old param, not supported in GUI anyway */
						deprecated_mag_rot = 0;
						param_set_no_notification(param_find("SENS_EXT_MAG_ROT"), &deprecated_mag_rot);
					}

					/* handling of transition from internal to external */
					if (mag_rot < 0) {
						mag_rot = 0;
					}

					get_rot_matrix((enum Rotation)mag_rot, &_mag_rotation[s]);
				}

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag", i);

				} else {

					/* apply new scaling and offsets */
					config_ok = apply_mag_calibration(h, &mscale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			mag_count++;
		}
	}
}

void VotedSensorsUpdate::accel_poll(struct sensor_combined_s &raw)
{
	for (unsigned uorb_index = 0; uorb_index < _accel.subscription_count; uorb_index++) {
		bool accel_updated;
		orb_check(_accel.subscription[uorb_index], &accel_updated);

		if (accel_updated) {
			struct accel_report accel_report;

			orb_copy(ORB_ID(sensor_accel), _accel.subscription[uorb_index], &accel_report);

			if (accel_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_accel.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_accel.subscription[uorb_index], &priority);
				_accel.priority[uorb_index] = (uint8_t)priority;
			}

			if (accel_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * becasue it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta velocities to an equivalent acceleration before application of corrections
				float dt_inv = 1.e6f / accel_report.integral_dt;
				math::Vector<3> accel_data(accel_report.x_integral * dt_inv , accel_report.y_integral * dt_inv ,
							   accel_report.z_integral * dt_inv);

				if (_thermal_correction_param.accel_tc_enable == 1) {
					// search through the available compensation parameter sets looking for one with a matching sensor ID and corect data if found
					for (unsigned param_index = 0; param_index < 3; param_index++) {
						if (accel_report.device_id == _thermal_correction_param.accel_cal_data[param_index].ID) {
							// get the offsets
							sensors_temp_comp::calc_thermal_offsets_3D(_thermal_correction_param.accel_cal_data[param_index], accel_report.temperature, _accel_offset[uorb_index]);

							// get the scale factors and correct the data
							for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
								_accel_scale[uorb_index][axis_index] = _thermal_correction_param.accel_cal_data[param_index].scale[axis_index];
								accel_data(axis_index) = accel_data(axis_index) * _accel_scale[uorb_index][axis_index] + _accel_offset[uorb_index][axis_index];
							}

							break;

						}
					}
				}

				// rotate corrected measurements from sensor to body frame
				accel_data = _board_rotation * accel_data;

				// write corrected data to uORB struct
				_last_sensor_data[uorb_index].accelerometer_integral_dt = accel_report.integral_dt / 1.e6f;
				_last_sensor_data[uorb_index].accelerometer_m_s2[0] = accel_data(0);
				_last_sensor_data[uorb_index].accelerometer_m_s2[1] = accel_data(1);
				_last_sensor_data[uorb_index].accelerometer_m_s2[2] = accel_data(2);

			} else {
				// using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				math::Vector<3> accel_data(accel_report.x , accel_report.y , accel_report.z);

				if (_thermal_correction_param.accel_tc_enable == 1) {
					// search through the available compensation parameter sets looking for one with a matching sensor ID
					for (unsigned param_index = 0; param_index < 3; param_index++) {
						if (accel_report.device_id == _thermal_correction_param.accel_cal_data[param_index].ID) {
							// get the offsets
							sensors_temp_comp::calc_thermal_offsets_3D(_thermal_correction_param.accel_cal_data[param_index], accel_report.temperature, _accel_offset[uorb_index]);

							// get the scale factors and correct the data
							for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
								_accel_scale[uorb_index][axis_index] = _thermal_correction_param.gyro_cal_data[param_index].scale[axis_index];
								accel_data(axis_index) = accel_data(axis_index) * _accel_scale[uorb_index][axis_index] + _accel_offset[uorb_index][axis_index];
							}

							break;

						}
					}
				}

				// rotate corrected measurements from sensor to body frame
				accel_data = _board_rotation * accel_data;

				// handle the cse where this is our first output
				if (_last_accel_timestamp[uorb_index] == 0) {
					_last_accel_timestamp[uorb_index] = accel_report.timestamp - 1000;
				}

				// approximate the  delta time using the difference in accel data time stamps
				// and write to uORB struct
				_last_sensor_data[uorb_index].accelerometer_integral_dt =
					(accel_report.timestamp - _last_accel_timestamp[uorb_index]) / 1.e6f;

				// write corrected body frame acceleration to uORB struct
				_last_sensor_data[uorb_index].accelerometer_m_s2[0] = accel_data(0);
				_last_sensor_data[uorb_index].accelerometer_m_s2[1] = accel_data(1);
				_last_sensor_data[uorb_index].accelerometer_m_s2[2] = accel_data(2);
			}

			_last_accel_timestamp[uorb_index] = accel_report.timestamp;
			_accel.voter.put(uorb_index, accel_report.timestamp, _last_sensor_data[uorb_index].accelerometer_m_s2, accel_report.error_count, _accel.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index;
	_accel.voter.get_best(hrt_absolute_time(), &best_index);

	// write the best sensor data to the output variables
	if (best_index >= 0) {
		raw.accelerometer_integral_dt = _last_sensor_data[best_index].accelerometer_integral_dt;
		_accel.last_best_vote = (uint8_t)best_index;
		_corrections.accel_select = (uint8_t)best_index;
		for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
			raw.accelerometer_m_s2[axis_index] = _last_sensor_data[best_index].accelerometer_m_s2[axis_index];
			_corrections.accel_offset[axis_index] = _accel_offset[best_index][axis_index];
			_corrections.accel_scale[axis_index] = _accel_scale[best_index][axis_index];
		}
	}
}

void VotedSensorsUpdate::gyro_poll(struct sensor_combined_s &raw)
{
	for (unsigned uorb_index = 0; uorb_index < _gyro.subscription_count; uorb_index++) {
		bool gyro_updated;
		orb_check(_gyro.subscription[uorb_index], &gyro_updated);

		if (gyro_updated) {
			struct gyro_report gyro_report;

			orb_copy(ORB_ID(sensor_gyro), _gyro.subscription[uorb_index], &gyro_report);

			if (gyro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_gyro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_gyro.subscription[uorb_index], &priority);
				_gyro.priority[uorb_index] = (uint8_t)priority;
			}

			if (gyro_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * becasue it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta angles to an equivalent angular rate before application of corrections
				float dt_inv = 1.e6f / gyro_report.integral_dt;
				math::Vector<3> gyro_rate(gyro_report.x_integral * dt_inv , gyro_report.y_integral * dt_inv ,
							  gyro_report.z_integral * dt_inv);

				if (_thermal_correction_param.gyro_tc_enable == 1) {
					// search through the available compensation parameter sets looking for one with a matching sensor ID and correct data if found
					for (unsigned param_index = 0; param_index < 3; param_index++) {
						if (gyro_report.device_id == _thermal_correction_param.gyro_cal_data[param_index].ID) {
							// get the offsets
							sensors_temp_comp::calc_thermal_offsets_3D(_thermal_correction_param.gyro_cal_data[param_index], gyro_report.temperature, _gyro_offset[uorb_index]);

							// get the sensor scale factors and correct the data
							for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
								_gyro_scale[uorb_index][axis_index] = _thermal_correction_param.gyro_cal_data[param_index].scale[axis_index];
								gyro_rate(axis_index) = gyro_rate(axis_index) * _gyro_scale[uorb_index][axis_index] + _gyro_offset[uorb_index][axis_index];
							}

							break;

						}
					}
				}

				// rotate corrected measurements from sensor to body frame
				gyro_rate = _board_rotation * gyro_rate;

				// write to uORB struct
				_last_sensor_data[uorb_index].gyro_integral_dt = gyro_report.integral_dt / 1.e6f;
				_last_sensor_data[uorb_index].gyro_rad[0] = gyro_rate(0);
				_last_sensor_data[uorb_index].gyro_rad[1] = gyro_rate(1);
				_last_sensor_data[uorb_index].gyro_rad[2] = gyro_rate(2);

			} else {
				//using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				math::Vector<3> gyro_rate(gyro_report.x, gyro_report.y, gyro_report.z);

				if (_thermal_correction_param.gyro_tc_enable == 1) {
					// search through the available compensation parameter sets looking for one with a matching sensor ID
					for (unsigned param_index = 0; param_index < 3; param_index++) {
						if (gyro_report.device_id == _thermal_correction_param.gyro_cal_data[param_index].ID) {
							// get the offsets
							sensors_temp_comp::calc_thermal_offsets_3D(_thermal_correction_param.gyro_cal_data[param_index], gyro_report.temperature, _gyro_offset[uorb_index]);

							// get the sensor scale factors and correct the data
							for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
								_gyro_scale[uorb_index][axis_index] = _thermal_correction_param.gyro_cal_data[param_index].scale[axis_index];
								gyro_rate(axis_index) = gyro_rate(axis_index) * _gyro_scale[uorb_index][axis_index] + _gyro_offset[uorb_index][axis_index];
							}

							break;

						}
					}
				}

				// rotate corrected measurements from sensor to body frame
				gyro_rate = _board_rotation * gyro_rate;

				// handle the case where this is our first output
				if (_last_sensor_data[uorb_index].timestamp == 0) {
					_last_sensor_data[uorb_index].timestamp = gyro_report.timestamp - 1000;
				}

				// approximate the  delta time using the difference in gyro data time stamps
				// and write to uORB struct
				_last_sensor_data[uorb_index].gyro_integral_dt =
					(gyro_report.timestamp - _last_sensor_data[uorb_index].timestamp) / 1.e6f;

				// write corrected body frame rates to uORB struct
				_last_sensor_data[uorb_index].gyro_rad[0] = gyro_rate(0);
				_last_sensor_data[uorb_index].gyro_rad[1] = gyro_rate(1);
				_last_sensor_data[uorb_index].gyro_rad[2] = gyro_rate(2);
			}

			_last_sensor_data[uorb_index].timestamp = gyro_report.timestamp;
			_gyro.voter.put(uorb_index, gyro_report.timestamp, _last_sensor_data[uorb_index].gyro_rad,
					gyro_report.error_count, _gyro.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index;
	_gyro.voter.get_best(hrt_absolute_time(), &best_index);

	// write data for the best sensor to output variables
	if (best_index >= 0) {
		raw.gyro_integral_dt = _last_sensor_data[best_index].gyro_integral_dt;
		raw.timestamp = _last_sensor_data[best_index].timestamp;
		_gyro.last_best_vote = (uint8_t)best_index;
		_corrections.gyro_select = (uint8_t)best_index;
		for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
			raw.gyro_rad[axis_index] = _last_sensor_data[best_index].gyro_rad[axis_index];
			_corrections.gyro_offset[axis_index] = _gyro_offset[best_index][axis_index];
			_corrections.gyro_scale[axis_index] = _gyro_scale[best_index][axis_index];
		}
	}
}

void VotedSensorsUpdate::mag_poll(struct sensor_combined_s &raw)
{
	for (unsigned i = 0; i < _mag.subscription_count; i++) {
		bool mag_updated;
		orb_check(_mag.subscription[i], &mag_updated);

		if (mag_updated) {
			struct mag_report mag_report;

			orb_copy(ORB_ID(sensor_mag), _mag.subscription[i], &mag_report);

			if (mag_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// First publication with data
			if (_mag.priority[i] == 0) {
				int32_t priority = 0;
				orb_priority(_mag.subscription[i], &priority);
				_mag.priority[i] = (uint8_t)priority;
			}

			math::Vector<3> vect(mag_report.x, mag_report.y, mag_report.z);
			vect = _mag_rotation[i] * vect;

			_last_sensor_data[i].magnetometer_ga[0] = vect(0);
			_last_sensor_data[i].magnetometer_ga[1] = vect(1);
			_last_sensor_data[i].magnetometer_ga[2] = vect(2);

			_last_mag_timestamp[i] = mag_report.timestamp;
			_mag.voter.put(i, mag_report.timestamp, vect.data,
				       mag_report.error_count, _mag.priority[i]);
		}
	}

	int best_index;
	_mag.voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		raw.magnetometer_ga[0] = _last_sensor_data[best_index].magnetometer_ga[0];
		raw.magnetometer_ga[1] = _last_sensor_data[best_index].magnetometer_ga[1];
		raw.magnetometer_ga[2] = _last_sensor_data[best_index].magnetometer_ga[2];
		_mag.last_best_vote = (uint8_t)best_index;
	}
}

void VotedSensorsUpdate::baro_poll(struct sensor_combined_s &raw)
{
	bool got_update = false;

	for (unsigned uorb_index = 0; uorb_index < _baro.subscription_count; uorb_index++) {
		bool baro_updated;
		orb_check(_baro.subscription[uorb_index], &baro_updated);

		if (baro_updated) {
			struct baro_report baro_report;

			orb_copy(ORB_ID(sensor_baro), _baro.subscription[uorb_index], &baro_report);

			if (baro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			if (_thermal_correction_param.baro_tc_enable == 1) {
				// search through the available compensation parameter sets looking for one with a matching sensor ID and correct data if found
				for (unsigned param_index = 0; param_index < 3; param_index++) {
					if (baro_report.device_id == _thermal_correction_param.baro_cal_data[param_index].ID) {
						// get the offsets
						sensors_temp_comp::calc_thermal_offsets_1D(_thermal_correction_param.baro_cal_data[param_index], baro_report.temperature, _baro_offset[uorb_index]);

						// get the sensor scale factors and correct the data
						_baro_scale[uorb_index] = _thermal_correction_param.baro_cal_data[param_index].scale;
						baro_report.pressure = baro_report.pressure * _baro_scale[uorb_index] + _baro_offset[uorb_index];

						break;

					}
				}
			}

			// First publication with data
			if (_baro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_baro.subscription[uorb_index], &priority);
				_baro.priority[uorb_index] = (uint8_t)priority;
			}

			got_update = true;
			math::Vector<3> vect(baro_report.altitude, 0.f, 0.f);

			_last_sensor_data[uorb_index].baro_alt_meter = baro_report.altitude;
			_last_sensor_data[uorb_index].baro_temp_celcius = baro_report.temperature;
			_last_baro_pressure[uorb_index] = baro_report.pressure;

			_last_baro_timestamp[uorb_index] = baro_report.timestamp;
			_baro.voter.put(uorb_index, baro_report.timestamp, vect.data,
					baro_report.error_count, _baro.priority[uorb_index]);
		}
	}

	if (got_update) {
		int best_index;
		_baro.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0) {
			raw.baro_alt_meter = _last_sensor_data[best_index].baro_alt_meter;
			raw.baro_temp_celcius = _last_sensor_data[best_index].baro_temp_celcius;
			_last_best_baro_pressure = _last_baro_pressure[best_index];
			_baro.last_best_vote = (uint8_t)best_index;
		}
	}
}

bool VotedSensorsUpdate::check_failover(SensorData &sensor, const char *sensor_name)
{
	if (sensor.last_failover_count != sensor.voter.failover_count()) {

		uint32_t flags = sensor.voter.failover_state();

		if (flags == DataValidator::ERROR_FLAG_NO_ERROR) {
			//we switched due to a non-critical reason. No need to panic.
			PX4_INFO("%s sensor switch from #%i", sensor_name, sensor.voter.failover_index());

		} else {
			mavlink_log_emergency(&_mavlink_log_pub, "%s #%i fail: %s%s%s%s%s!",
					      sensor_name,
					      sensor.voter.failover_index(),
					      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
					      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
					      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TOUT" : ""),
					      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ECNT" : ""),
					      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " EDNST" : ""));
		}

		sensor.last_failover_count = sensor.voter.failover_count();
		return true;
	}

	return false;
}

bool VotedSensorsUpdate::check_vibration()
{
	bool ret = false;
	hrt_abstime cur_time = hrt_absolute_time();

	if (!_vibration_warning && (_gyro.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold ||
				    _accel.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold ||
				    _mag.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold)) {

		if (_vibration_warning_timestamp == 0) {
			_vibration_warning_timestamp = cur_time;

		} else if (hrt_elapsed_time(&_vibration_warning_timestamp) > 10000 * 1000) {
			_vibration_warning = true;
			mavlink_log_critical(&_mavlink_log_pub, "HIGH VIBRATION! g: %d a: %d m: %d",
					     (int)(100 * _gyro.voter.get_vibration_factor(cur_time)),
					     (int)(100 * _accel.voter.get_vibration_factor(cur_time)),
					     (int)(100 * _mag.voter.get_vibration_factor(cur_time)));
			ret = true;
		}

	} else {
		_vibration_warning_timestamp = 0;
	}

	return ret;
}

void VotedSensorsUpdate::init_sensor_class(const struct orb_metadata *meta, SensorData &sensor_data)
{
	unsigned group_count = orb_group_count(meta);

	if (group_count > SENSOR_COUNT_MAX) {
		group_count = SENSOR_COUNT_MAX;
	}

	for (unsigned i = 0; i < group_count; i++) {
		if (sensor_data.subscription[i] < 0) {
			sensor_data.subscription[i] = orb_subscribe_multi(meta, i);

			if (i > 0) {
				/* the first always exists, but for each further sensor, add a new validator */
				if (!sensor_data.voter.add_new_validator()) {
					PX4_ERR("failed to add validator for sensor %s %i", meta->o_name, i);
				}
			}
		}
	}

	sensor_data.subscription_count = group_count;
}

void VotedSensorsUpdate::print_status()
{
	PX4_INFO("gyro status:");
	_gyro.voter.print();
	PX4_INFO("accel status:");
	_accel.voter.print();
	PX4_INFO("mag status:");
	_mag.voter.print();
	PX4_INFO("baro status:");
	_baro.voter.print();
}

bool
VotedSensorsUpdate::apply_gyro_calibration(DevHandle &h, const struct gyro_calibration_s *gcal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(GYROIOCSSCALE, (long unsigned int)gcal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
VotedSensorsUpdate::apply_accel_calibration(DevHandle &h, const struct accel_calibration_s *acal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(ACCELIOCSSCALE, (long unsigned int)acal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
VotedSensorsUpdate::apply_mag_calibration(DevHandle &h, const struct mag_calibration_s *mcal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(MAGIOCSSCALE, (long unsigned int)mcal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

void VotedSensorsUpdate::sensors_poll(sensor_combined_s &raw)
{
	accel_poll(raw);
	gyro_poll(raw);
	mag_poll(raw);
	baro_poll(raw);

	// publish sensor corrections
	if (_sensor_correction_pub == nullptr) {
		_sensor_correction_pub = orb_advertise(ORB_ID(sensor_correction), &_corrections);

	} else {
		orb_publish(ORB_ID(sensor_correction), _sensor_correction_pub, &_corrections);

	}
}

void VotedSensorsUpdate::check_failover()
{
	check_failover(_accel, "Accel");
	check_failover(_gyro, "Gyro");
	check_failover(_mag, "Mag");
	check_failover(_baro, "Baro");
}

void VotedSensorsUpdate::set_relative_timestamps(sensor_combined_s &raw)
{
	if (_last_accel_timestamp[_accel.last_best_vote]) {
		raw.accelerometer_timestamp_relative = (int32_t)(_last_accel_timestamp[_accel.last_best_vote] - raw.timestamp);
	}

	if (_last_mag_timestamp[_mag.last_best_vote]) {
		raw.magnetometer_timestamp_relative = (int32_t)(_last_mag_timestamp[_mag.last_best_vote] - raw.timestamp);
	}

	if (_last_baro_timestamp[_baro.last_best_vote]) {
		raw.baro_timestamp_relative = (int32_t)(_last_baro_timestamp[_baro.last_best_vote] - raw.timestamp);
	}
}

void
VotedSensorsUpdate::calc_accel_inconsistency(sensor_preflight_s &preflt)
{
	float accel_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (unsigned sensor_index = 0; sensor_index < _accel.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if ((_accel.priority[sensor_index] > 0) && (sensor_index != _accel.last_best_vote)) {

			float accel_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison agains the primary

			// calculate accel_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_accel_diff[axis_index][check_index] = 0.95f * _accel_diff[axis_index][check_index] + 0.05f *
								       (_last_sensor_data[_accel.last_best_vote].accelerometer_m_s2[axis_index] -
									_last_sensor_data[sensor_index].accelerometer_m_s2[axis_index]);
				accel_diff_sum_sq += _accel_diff[axis_index][check_index] * _accel_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (accel_diff_sum_sq > accel_diff_sum_max_sq) {
				accel_diff_sum_max_sq = accel_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// skip check if less than 2 sensors
	if (check_index < 1) {
		preflt.accel_inconsistency_m_s_s = 0.0f;

	} else {
		// get the vector length of the largest difference and write to the combined sensor struct
		preflt.accel_inconsistency_m_s_s = sqrtf(accel_diff_sum_max_sq);
	}
}

void VotedSensorsUpdate::calc_gyro_inconsistency(sensor_preflight_s &preflt)
{
	float gyro_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (unsigned sensor_index = 0; sensor_index < _gyro.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if ((_gyro.priority[sensor_index] > 0) && (sensor_index != _gyro.last_best_vote)) {

			float gyro_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison against the primary

			// calculate gyro_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_gyro_diff[axis_index][check_index] = 0.95f * _gyro_diff[axis_index][check_index] + 0.05f *
								      (_last_sensor_data[_gyro.last_best_vote].gyro_rad[axis_index] -
								       _last_sensor_data[sensor_index].gyro_rad[axis_index]);
				gyro_diff_sum_sq += _gyro_diff[axis_index][check_index] * _gyro_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (gyro_diff_sum_sq > gyro_diff_sum_max_sq) {
				gyro_diff_sum_max_sq = gyro_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// skip check if less than 2 sensors
	if (check_index < 1) {
		preflt.gyro_inconsistency_rad_s = 0.0f;

	} else {
		// get the vector length of the largest difference and write to the combined sensor struct
		preflt.gyro_inconsistency_rad_s = sqrtf(gyro_diff_sum_max_sq);
	}
}


