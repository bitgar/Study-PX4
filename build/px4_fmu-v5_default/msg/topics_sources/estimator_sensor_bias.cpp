/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file estimator_sensor_bias.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_estimator_sensor_bias_fields[] = "uint64_t timestamp;uint64_t timestamp_sample;uint32_t gyro_device_id;float[3] gyro_bias;float gyro_bias_limit;float[3] gyro_bias_variance;uint32_t accel_device_id;float[3] accel_bias;float accel_bias_limit;float[3] accel_bias_variance;uint32_t mag_device_id;float[3] mag_bias;float mag_bias_limit;float[3] mag_bias_variance;bool gyro_bias_valid;bool accel_bias_valid;bool mag_bias_valid;uint8_t[5] _padding0;";

ORB_DEFINE(estimator_sensor_bias, struct estimator_sensor_bias_s, 115, __orb_estimator_sensor_bias_fields, static_cast<uint8_t>(ORB_ID::estimator_sensor_bias));


void print_message(const estimator_sensor_bias_s &message)
{

	PX4_INFO_RAW(" estimator_sensor_bias_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	
	PX4_INFO_RAW("\ttimestamp_sample: %" PRIu64 "  (%" PRIu64 " us before timestamp)\n", message.timestamp_sample, message.timestamp - message.timestamp_sample);
	
	char gyro_device_id_buffer[80];
device::Device::device_id_print_buffer(gyro_device_id_buffer, sizeof(gyro_device_id_buffer), message.gyro_device_id);
PX4_INFO_RAW("\tgyro_device_id: %" PRId32 " (%s) \n", message.gyro_device_id, gyro_device_id_buffer);
	PX4_INFO_RAW("\tgyro_bias: [%.4f, %.4f, %.4f]\n", (double)message.gyro_bias[0], (double)message.gyro_bias[1], (double)message.gyro_bias[2]);
	PX4_INFO_RAW("\tgyro_bias_limit: %.4f\n", (double)message.gyro_bias_limit);
	PX4_INFO_RAW("\tgyro_bias_variance: [%.4f, %.4f, %.4f]\n", (double)message.gyro_bias_variance[0], (double)message.gyro_bias_variance[1], (double)message.gyro_bias_variance[2]);
	char accel_device_id_buffer[80];
device::Device::device_id_print_buffer(accel_device_id_buffer, sizeof(accel_device_id_buffer), message.accel_device_id);
PX4_INFO_RAW("\taccel_device_id: %" PRId32 " (%s) \n", message.accel_device_id, accel_device_id_buffer);
	PX4_INFO_RAW("\taccel_bias: [%.4f, %.4f, %.4f]\n", (double)message.accel_bias[0], (double)message.accel_bias[1], (double)message.accel_bias[2]);
	PX4_INFO_RAW("\taccel_bias_limit: %.4f\n", (double)message.accel_bias_limit);
	PX4_INFO_RAW("\taccel_bias_variance: [%.4f, %.4f, %.4f]\n", (double)message.accel_bias_variance[0], (double)message.accel_bias_variance[1], (double)message.accel_bias_variance[2]);
	char mag_device_id_buffer[80];
device::Device::device_id_print_buffer(mag_device_id_buffer, sizeof(mag_device_id_buffer), message.mag_device_id);
PX4_INFO_RAW("\tmag_device_id: %" PRId32 " (%s) \n", message.mag_device_id, mag_device_id_buffer);
	PX4_INFO_RAW("\tmag_bias: [%.4f, %.4f, %.4f]\n", (double)message.mag_bias[0], (double)message.mag_bias[1], (double)message.mag_bias[2]);
	PX4_INFO_RAW("\tmag_bias_limit: %.4f\n", (double)message.mag_bias_limit);
	PX4_INFO_RAW("\tmag_bias_variance: [%.4f, %.4f, %.4f]\n", (double)message.mag_bias_variance[0], (double)message.mag_bias_variance[1], (double)message.mag_bias_variance[2]);
	PX4_INFO_RAW("\tgyro_bias_valid: %s\n", (message.gyro_bias_valid ? "True" : "False"));
	PX4_INFO_RAW("\taccel_bias_valid: %s\n", (message.accel_bias_valid ? "True" : "False"));
	PX4_INFO_RAW("\tmag_bias_valid: %s\n", (message.mag_bias_valid ? "True" : "False"));
	
}
