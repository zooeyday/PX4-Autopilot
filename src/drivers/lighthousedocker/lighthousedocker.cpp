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

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>


// LHDOCKER::LHDOCKER(const char *port, uint8_t rotation) :
LHDOCKER::LHDOCKER(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

}


LHDOCKER::~LHDOCKER()
{
	// Ensure we are truly inactive.
	stop();

	//free((char *)_port);
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}



void
LHDOCKER::decodeInit()
{
	_decode_state = LHD_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_payload_index = 0;
}



void
LHDOCKER::addByteToChecksum(const uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
LHDOCKER::calcChecksum(const uint8_t *buffer, const uint16_t length, lhd_checksum_t *checksum)
{
	for (uint16_t i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}


/**
 * Add payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
LHDOCKER::payloadRxAdd(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	p_buf[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}



/**
 * Finish payload rx
 */
void	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
LHDOCKER::payloadRxDone()
{

        irlock_report_s orb_report{};

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	orb_report.timestamp = timestamp_sample;

	orb_report.signature  = _buf.lhd_payload_rx.status;
	orb_report.pos_x     = _buf.lhd_payload_rx.rel_pos_x;
	orb_report.pos_y     = _buf.lhd_payload_rx.rel_pos_y;
	orb_report.size_x    = _buf.lhd_payload_rx.rel_pos_z;
	orb_report.size_y    = _buf.lhd_payload_rx.rel_hdg;

	_irlock_report_topic.publish(orb_report);


}



int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
LHDOCKER::parseChar(const uint8_t b)
{
	int ret = 0;

	switch (_decode_state) {


	/* Expecting Sync1 */
	case LHD_DECODE_SYNC1:
		if (b == LHD_SYNC1) {	// Sync1 found --> expecting Sync2
			_decode_state = LHD_DECODE_SYNC2;
		}
		break;

	/* Expecting Sync2 */
	case LHD_DECODE_SYNC2:
		if (b == LHD_SYNC2) {	// Sync2 found --> expecting payload
			_decode_state = LHD_DECODE_PAYLOAD;

		} else {		// Sync1 not followed by Sync2: reset parser
			decodeInit();
		}

		break;

	/* Expecting payload */
	case LHD_DECODE_PAYLOAD:

		addByteToChecksum(b);
        	ret = payloadRxAdd(b);


		if (ret < 0) {
			// payload not handled, discard message
			decodeInit();

		} else if (ret > 0) {
			// payload complete, expecting checksum
			_decode_state = LHD_DECODE_CHKSUM1;

		} else {
			// expecting more payload, stay in state LHD_DECODE_PAYLOAD
		}

		ret = 0;
		break;

	/* Expecting first checksum byte */
	case LHD_DECODE_CHKSUM1:
		if (_rx_ck_a != b) {
			PX4_DEBUG("LHD checksum err");
			decodeInit();

		} else {
			_decode_state = LHD_DECODE_CHKSUM2;
		}

		break;

	/* Expecting second checksum byte */
	case LHD_DECODE_CHKSUM2:
		if (_rx_ck_b != b) {
			PX4_DEBUG("LHD checksum err");

		} else {
			payloadRxDone();	// finish payload processing
			ret = 0;
		}

		decodeInit();
		break;

	default:
		break;
	}

	return ret;
}



// bool
// LHDOCKER::sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
// {
// 	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
// 	ubx_checksum_t checksum = {0, 0};

// 	// Populate header
// 	header.msg	= msg;
// 	header.length	= length;

// 	// Calculate checksum
// 	calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

// 	if (payload != nullptr) {
// 		calcChecksum(payload, length, &checksum);
// 	}

// 	// Send message
// 	if (write((void *)&header, sizeof(header)) != sizeof(header)) {
// 		return false;
// 	}

// 	if (payload && write((void *)payload, length) != length) {
// 		return false;
// 	}

// 	if (write((void *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
// 		return false;
// 	}

// 	return true;
// }



int
LHDOCKER::collect()
{
	perf_begin(_sample_perf);



	uint8_t buf[LHD_READ_BUFFER_SIZE];


	// Read from the sensor UART buffer.
	int ret = ::read(_file_descriptor, &buf[0], sizeof(buf));

	if (ret > 0) {
		/* pass received bytes to the packet decoder */
		for (int i = 0; i < ret; i++) {
			 parseChar(buf[i]);
			//LHD_DEBUG("parsed %d: 0x%x", i, buf[i]);
		}
	} else if (ret == -1 && errno == EAGAIN) {
		return -EAGAIN;

	} else {

		PX4_ERR("read error: %i, errno: %i", ret, errno);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}



	perf_end(_sample_perf);

	return PX4_OK;
}




int
LHDOCKER::init()
{
	start();

	return PX4_OK;
}

int
LHDOCKER::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}


void
LHDOCKER::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
LHDOCKER::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	// Perform collection.
	collect();
}

void
LHDOCKER::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(LHDOCK_MEASURE_INTERVAL);

	PX4_INFO("driver started");
}

void
LHDOCKER::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();

	// Ensure the serial port is closed.
	::close(_file_descriptor);
	// _file_descriptor = -1;

}
