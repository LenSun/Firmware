/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file temperature_accel_calibration.cpp
 * Implementation of the Temperature Calibration for onboard accelerometer sensors.
 *
 * @author Siddharth Bharat Purohit
 * @author Paul Riseborough
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>
#include <vector>
#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <platforms/px4_defines.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_accel.h>
#include "polyfit.hpp"
#include "temperature_calibration.h"

#define TC_PRINT_DEBUG 0
#if TC_PRINT_DEBUG
#define TC_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define TC_DEBUG(fmt, ...)
#endif


#define SENSOR_COUNT_MAX		3

extern "C" __EXPORT int tempcal_main(int argc, char *argv[]);


class Tempcalaccel;

namespace tempcalaccel
{
Tempcalaccel *instance = nullptr;
}


class Tempcalaccel : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Tempcalaccel();

	/**
	 * Destructor, also kills task.
	 */
	~Tempcalaccel();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void do_temperature_accel_calibration(int argc, char *argv[]);

	void		task_main();

	void print_status();

	void exit() { _task_should_exit = true; }

private:
	bool	_task_should_exit = false;
	int	_control_task = -1;		// task handle for task
};

Tempcalaccel::Tempcalaccel():
	SuperBlock(NULL, "Tempcalaccel")
{
}

Tempcalaccel::~Tempcalaccel()
{

}

void Tempcalaccel::task_main()
{
	// subscribe to relevant topics
	int accel_sub[SENSOR_COUNT_MAX];
	float accel_sample_filt[SENSOR_COUNT_MAX][4];
	polyfitter<4> P[SENSOR_COUNT_MAX][3];
	px4_pollfd_struct_t fds[SENSOR_COUNT_MAX] = {};
	unsigned _hot_soak_sat[SENSOR_COUNT_MAX] = {};
	unsigned num_accel = orb_group_count(ORB_ID(sensor_accel));
	unsigned num_samples[SENSOR_COUNT_MAX] = {0};
	uint32_t device_ids[SENSOR_COUNT_MAX] = {};

	if (num_accel > SENSOR_COUNT_MAX) {
		num_accel = SENSOR_COUNT_MAX;
	}

	bool _cold_soaked[SENSOR_COUNT_MAX] = {false};
	bool _hot_soaked[SENSOR_COUNT_MAX] = {false};
	bool _tempcal_complete[SENSOR_COUNT_MAX] = {false};
	float _low_temp[SENSOR_COUNT_MAX];
	float _high_temp[SENSOR_COUNT_MAX] = {0};
	float _ref_temp[SENSOR_COUNT_MAX];

	for (unsigned i = 0; i < num_accel; i++) {
		accel_sub[i] = orb_subscribe_multi(ORB_ID(sensor_accel), i);
		fds[i].fd = accel_sub[i];
		fds[i].events = POLLIN;
	}

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_accel_s accel_data = {};

	/* reset all driver level calibrations */
	int param_set_result = PX4_OK;
	char param_str[30];
	float offset = 0.0f;
	float scale = 1.0f;
	for (unsigned s = 0; s < num_accel; s++) {
		(void)sprintf(param_str, "CAL_ACC%u_XOFF", s);
		param_set_result = param_set(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_YOFF", s);
		param_set_result = param_set(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_ZOFF", s);
		param_set_result = param_set(param_find(param_str), &offset);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_XSCALE", s);
		param_set_result = param_set(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_YSCALE", s);
		param_set_result = param_set(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
		(void)sprintf(param_str, "CAL_ACC%u_ZSCALE", s);
		param_set_result = param_set(param_find(param_str), &scale);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}
	}

	while (!_task_should_exit) {
		int ret = px4_poll(fds, num_accel, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		for (unsigned i = 0; i < num_accel; i++) {
			if (_hot_soaked[i]) {
				continue;
			}

			if (fds[i].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_accel), accel_sub[i], &accel_data);

				device_ids[i] = accel_data.device_id;

				accel_sample_filt[i][0] = accel_data.x;
				accel_sample_filt[i][1] = accel_data.y;
				accel_sample_filt[i][2] = accel_data.z;
				accel_sample_filt[i][3] = accel_data.temperature;

				if (!_cold_soaked[i]) {
					_cold_soaked[i] = true;
					_low_temp[i] = accel_sample_filt[i][3];	//Record the low temperature
					_ref_temp[i] = accel_sample_filt[i][3] + 12.0f;
				}

				num_samples[i]++;
			}
		}

		for (unsigned i = 0; i < num_accel; i++) {
			if (_hot_soaked[i]) {
				continue;
			}

			if (accel_sample_filt[i][3] > _high_temp[i]) {
				_high_temp[i] = accel_sample_filt[i][3];
				_hot_soak_sat[i] = 0;

			} else {
				continue;
			}

			//TODO: Hot Soak Saturation
			if (_hot_soak_sat[i] == 10 || (_high_temp[i] - _low_temp[i]) > 24.0f) {
				_hot_soaked[i] = true;
			}

			if (i == 0) {
				TC_DEBUG("\n%.20f,%.20f,%.20f,%.20f, %.6f, %.6f, %.6f\n\n", (double)accel_sample_filt[i][0],
					 (double)accel_sample_filt[i][1],
					 (double)accel_sample_filt[i][2], (double)accel_sample_filt[i][3], (double)_low_temp[i], (double)_high_temp[i],
					 (double)(_high_temp[i] - _low_temp[i]));
			}

			//update linear fit matrices
			accel_sample_filt[i][3] -= _ref_temp[i];
			P[i][0].update((double)accel_sample_filt[i][3], (double)accel_sample_filt[i][0]);
			P[i][1].update((double)accel_sample_filt[i][3], (double)accel_sample_filt[i][1]);
			P[i][2].update((double)accel_sample_filt[i][3], (double)accel_sample_filt[i][2]);
			num_samples[i] = 0;
		}

		for (unsigned i = 0; i < num_accel; i++) {
			if (_hot_soaked[i] && !_tempcal_complete[i]) {
				double res[3][4] = {0.0f};
				P[i][0].fit(res[0]);
				res[0][3] = 0.0; // normalise the correction to be zero at the reference temperature
				PX4_WARN("Result Accel %d Axis 0: %.20f %.20f %.20f %.20f", i, (double)res[0][0], (double)res[0][1], (double)res[0][2],
					 (double)res[0][3]);
				P[i][1].fit(res[1]);
				res[1][3] = 0.0; // normalise the correction to be zero at the reference temperature
				PX4_WARN("Result Accel %d Axis 1: %.20f %.20f %.20f %.20f", i, (double)res[1][0], (double)res[1][1], (double)res[1][2],
					 (double)res[1][3]);
				P[i][2].fit(res[2]);
				res[2][3] = 0.0; // normalise the correction to be zero at the reference temperature
				PX4_WARN("Result Accel %d Axis 2: %.20f %.20f %.20f %.20f", i, (double)res[2][0], (double)res[2][1], (double)res[2][2],
					 (double)res[2][3]);
				_tempcal_complete[i] = true;

				float param = 0.0f;

				sprintf(param_str, "TC_A%d_ID", i);
				param_set_result = param_set(param_find(param_str), &device_ids[i]);

				if (param_set_result != PX4_OK) {
					PX4_ERR("unable to reset %s", param_str);
				}

				for (unsigned j = 0; j < 3; j++) {
					for (unsigned m = 0; m <= 3; m++) {
						sprintf(param_str, "TC_A%d_X%d_%d", i, (3-m), j);
						param = (float)res[j][m];
						param_set_result = param_set(param_find(param_str), &param);

						if (param_set_result != PX4_OK) {
							PX4_ERR("unable to reset %s", param_str);
						}
					}

					sprintf(param_str, "TC_A%d_TMAX", i);
					param = _high_temp[i];
					param_set_result = param_set(param_find(param_str), &param);

					if (param_set_result != PX4_OK) {
						PX4_ERR("unable to reset %s", param_str);
					}

					sprintf(param_str, "TC_A%d_TMIN", i);
					param = _low_temp[i];
					param_set_result = param_set(param_find(param_str), &param);

					if (param_set_result != PX4_OK) {
						PX4_ERR("unable to reset %s", param_str);
					}

					sprintf(param_str, "TC_A%d_TREF", i);
					param = _ref_temp[i];
					param_set_result = param_set(param_find(param_str), &param);

					if (param_set_result != PX4_OK) {
						PX4_ERR("unable to reset %s", param_str);
					}
				}

			}
		}

		// Enable use of the thermal compensation
		sprintf(param_str, "TC_A_ENABLE");
		int32_t temp_val = 1;
		param_set_result = param_set(param_find(param_str), &temp_val);
		if (param_set_result != PX4_OK) {
			PX4_ERR("unable to reset %s", param_str);
		}

	}

	for (unsigned i = 0; i < num_accel; i++) {
		orb_unsubscribe(accel_sub[i]);
	}

	delete tempcalaccel::instance;
	tempcalaccel::instance = nullptr;
	PX4_INFO("Tempcalaccel process stopped");
}

void Tempcalaccel::do_temperature_accel_calibration(int argc, char *argv[])
{
	tempcalaccel::instance->task_main();
}

int Tempcalaccel::start()
{

	ASSERT(_control_task == -1);
	_control_task = px4_task_spawn_cmd("accel_temp_calib",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Tempcalaccel::do_temperature_accel_calibration,
					   nullptr);

	if (_control_task < 0) {
		delete tempcalaccel::instance;
		tempcalaccel::instance = nullptr;
		PX4_ERR("start failed");
		return -errno;
	}

	return 0;
}

int run_temperature_accel_calibration()
{
	PX4_INFO("Starting accel thermal calibration task");
	tempcalaccel::instance = new Tempcalaccel();

	if (tempcalaccel::instance == nullptr) {
		PX4_ERR("alloc failed");
		return 1;
	}

	return tempcalaccel::instance->start();
}
