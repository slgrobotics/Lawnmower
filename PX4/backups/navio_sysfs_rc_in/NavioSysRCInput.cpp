/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include "NavioSysRCInput.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace time_literals;

#define DRV_RC_DEVTYPE_RCNAVIO2	0x8a

#define TEENSY_BUS				1       // 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
#define TEENSY_ADDR				0x48	// I2C adress
#define TEENSY_REG				0x00
#define I2C_BUS_FREQUENCY		400000

namespace navio_sysfs_rc_in
{

NavioSysRCInput::NavioSysRCInput() :
	I2C(DRV_RC_DEVTYPE_RCNAVIO2, MODULE_NAME, TEENSY_BUS, TEENSY_ADDR, I2C_BUS_FREQUENCY),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))

{
	PX4_INFO("NavioSysRCInput::NavioSysRCInput()");
	_isRunning = true;
	//memset(data, 0, sizeof(data));

};

NavioSysRCInput::~NavioSysRCInput()
{
	ScheduleClear();

	_isRunning = false;

	PX4_INFO("NavioSysRCInput::~NavioSysRCInput()");

	perf_free(_publish_interval_perf);
}

int NavioSysRCInput::navio_rc_init()
{
	PX4_INFO("NavioSysRCInput::navio_rc_init()");

	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	ScheduleNow();

	return ret;
}

int NavioSysRCInput::start()
{
	PX4_INFO("NavioSysRCInput::start()");

	navio_rc_init();

	_should_exit.store(false);

	ScheduleOnInterval(10_ms); // 100 Hz

	return PX4_OK;
}

void NavioSysRCInput::stop()
{
	PX4_INFO("NavioSysRCInput::stop()");

	_should_exit.store(true);
}

void NavioSysRCInput::Run()
{
	if (_should_exit.load()) {
		ScheduleClear();
		return;
	}

	/*
	char connected_buf[12] {};
	int ret_connected = ::pread(_connected_fd, connected_buf, sizeof(connected_buf) - 1, 0);

	if (ret_connected < 0) {
		return;
	}
	*/

	uint64_t timestamp_sample = hrt_absolute_time();
	input_rc_s data{};
	data.rc_lost = 0;

	data.values[0] = 1230;
	data.values[1] = 1502;
	data.values[2] = 1503;

	uint8_t _block[18];

	int ret = transfer(nullptr, 0, _block, 18);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_ERR("NavioSysRCInput::Run() - I2C transfer() returned %d", ret);
		DEVICE_LOG("i2c::transfer returned %d", ret);
		return;
	}

	for(int i=0; i < 9; ++i) {
		int j = i << 1;
		data.values[i] = ((uint16_t)_block[j]) + (((uint16_t)_block[j+1]) << 8);
	}

	// data.values[0-6] - channel 1..7 values, must be within 800..2200 range
	// data.values[7] - channel 8 value, programmed on the receiver to be 1999 on fail (also 1999 when second left switch down)
	// data.values[8] - milliseconds since Teensy last received valid PPM signal from the receiver

	// check if all channels are within the 800..2200 range:
	for (int i = 0; i < 8; ++i) {
		if (data.values[i] < 800 || data.values[i] > 2200) {
			perf_count(_comms_errors);
			return; // skip this cycle
		}
	}

	// R/C channel 8 is also failsafe indicator, as programmed in the receiver.
	// The second left switch still works, emulating failsafe in down position.
	// values for switch up: 999 down: 1999
	// the "ch9" value is milliseconds since Teensy last received valid PPM signal from the receiver, normally 1..20

	data.rc_lost = (data.values[7] > 1700 && data.values[7] <= 2200) || (data.values[8] > 100);
	_lastRcLost = data.rc_lost;

	data.timestamp_last_signal = timestamp_sample;
	data.channel_count = CHANNELS;
	data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
	data.timestamp = hrt_absolute_time();

	_input_rc_pub.publish(data);
	perf_count(_publish_interval_perf);
}

int NavioSysRCInput::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Teensy on I2C bus: %d addr: 0x%x  R/C Lost: %s", TEENSY_BUS, TEENSY_ADDR, _lastRcLost ? "true" : "false");

	perf_print_counter(_publish_interval_perf);
	perf_print_counter(_comms_errors);

	return 0;
}

}; // namespace navio_sysfs_rc_in
