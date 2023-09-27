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

/* Auto-generated by genmsg_cpp from file esc_status.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_esc_status_fields[] = "uint64_t timestamp;uint16_t counter;uint8_t esc_count;uint8_t esc_connectiontype;uint8_t esc_online_flags;uint8_t esc_armed_flags;uint8_t[2] _padding0;esc_report[8] esc;";

ORB_DEFINE(esc_status, struct esc_status_s, 272, __orb_esc_status_fields, static_cast<uint8_t>(ORB_ID::esc_status));


void print_message(const esc_status_s &message)
{

	PX4_INFO_RAW(" esc_status_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\tcounter: %u\n", message.counter);
	PX4_INFO_RAW("\tesc_count: %u\n", message.esc_count);
	PX4_INFO_RAW("\tesc_connectiontype: %u\n", message.esc_connectiontype);
	PX4_INFO_RAW("\tesc_online_flags: %u (0b", message.esc_online_flags);
	for (int i = (sizeof(message.esc_online_flags) * 8) - 1; i >= 0; i--) { PX4_INFO_RAW("%lu%s", (unsigned long) message.esc_online_flags >> i & 1, ((unsigned)i < (sizeof(message.esc_online_flags) * 8) - 1 && i % 4 == 0 && i > 0) ? "'" : ""); }
	PX4_INFO_RAW(")\n");
	PX4_INFO_RAW("\tesc_armed_flags: %u (0b", message.esc_armed_flags);
	for (int i = (sizeof(message.esc_armed_flags) * 8) - 1; i >= 0; i--) { PX4_INFO_RAW("%lu%s", (unsigned long) message.esc_armed_flags >> i & 1, ((unsigned)i < (sizeof(message.esc_armed_flags) * 8) - 1 && i % 4 == 0 && i > 0) ? "'" : ""); }
	PX4_INFO_RAW(")\n");
		PX4_INFO_RAW("\tpx4/esc_report[8] esc[0]");
 print_message(message.esc[0]);
PX4_INFO_RAW("\tpx4/esc_report[8] esc[1]");
 print_message(message.esc[1]);
PX4_INFO_RAW("\tpx4/esc_report[8] esc[2]");
 print_message(message.esc[2]);
PX4_INFO_RAW("\tpx4/esc_report[8] esc[3]");
 print_message(message.esc[3]);
PX4_INFO_RAW("\tpx4/esc_report[8] esc[4]");
 print_message(message.esc[4]);
PX4_INFO_RAW("\tpx4/esc_report[8] esc[5]");
 print_message(message.esc[5]);
PX4_INFO_RAW("\tpx4/esc_report[8] esc[6]");
 print_message(message.esc[6]);
PX4_INFO_RAW("\tpx4/esc_report[8] esc[7]");
 print_message(message.esc[7]);

}
