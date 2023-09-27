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

/* Auto-generated by genmsg_cpp from file power_monitor.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_power_monitor_fields[] = "uint64_t timestamp;float voltage_v;float current_a;float power_w;int16_t rconf;int16_t rsv;int16_t rbv;int16_t rp;int16_t rc;int16_t rcal;int16_t me;int16_t al;uint8_t[4] _padding0;";

ORB_DEFINE(power_monitor, struct power_monitor_s, 36, __orb_power_monitor_fields, static_cast<uint8_t>(ORB_ID::power_monitor));


void print_message(const power_monitor_s &message)
{

	PX4_INFO_RAW(" power_monitor_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\tvoltage_v: %.4f\n", (double)message.voltage_v);
	PX4_INFO_RAW("\tcurrent_a: %.4f\n", (double)message.current_a);
	PX4_INFO_RAW("\tpower_w: %.4f\n", (double)message.power_w);
	PX4_INFO_RAW("\trconf: %d\n", message.rconf);
	PX4_INFO_RAW("\trsv: %d\n", message.rsv);
	PX4_INFO_RAW("\trbv: %d\n", message.rbv);
	PX4_INFO_RAW("\trp: %d\n", message.rp);
	PX4_INFO_RAW("\trc: %d\n", message.rc);
	PX4_INFO_RAW("\trcal: %d\n", message.rcal);
	PX4_INFO_RAW("\tme: %d\n", message.me);
	PX4_INFO_RAW("\tal: %d\n", message.al);
	
}
