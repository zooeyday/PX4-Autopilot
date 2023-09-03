/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

#include "lighthousedocker.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>



/**
 * Local functions in support of the shell command.
 */
namespace lighthousedocker
{

LHDOCKER *g_dev{nullptr};

// int reset(const char *port);
// int start(const char *port);
// int status();
// int stop();
// int usage();



/**
 * Start the driver.
 */
static int start(const char *port)
{

	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	if (port == nullptr) {
		PX4_ERR("no device specified");
		return PX4_ERROR;
	}

	// Instantiate the driver.
	g_dev = new LHDOCKER(port);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print the driver status.
 */
static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();
	return PX4_OK;
}

/**
 * Stop the driver
 */
static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	} else {
		PX4_ERR("driver not stopped");
		return PX4_ERROR;
	}
	PX4_INFO("driver stopped");
	return PX4_OK;
}


/**
 * Reset the driver.
 */
static int reset(const char *port)
{
	if (stop() == PX4_OK) {
		return start(port);
	}

	return PX4_ERROR;
}



static int usage()
{
	// PX4_INFO("usage: lighthousedocker command [options]");
	// PX4_INFO("command:");
	// PX4_INFO("\treset|start|status|stop");
	// PX4_INFO("options:");
	// PX4_INFO("\t-d --device_path");


	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the lighthousedocker.

Most boards are configured to enable/start the driver on a specified UART using the SENS_LHD_CFG parameter.

Setup/usage information:

### Examples

Attempt to start driver on a specified serial device.
$ lighthousedocker start -d /dev/ttyS2
View driver status
$ lighthousedocker status
Stop driver
$ lighthousedocker stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lighthousedocker", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d',"/dev/ttyS2", "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "View driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d',"/dev/ttyS2", "<file:dev>", "Serial device", false);

	return PX4_OK;
}

} // namespace


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int lighthousedocker_main(int argc, char *argv[])
{
	// uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return lighthousedocker::usage();
		}
	}

	if (myoptind >= argc) {
		return lighthousedocker::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return lighthousedocker::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return lighthousedocker::start(device_path);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return lighthousedocker::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return lighthousedocker::stop();
	}

	return lighthousedocker::usage();
}
