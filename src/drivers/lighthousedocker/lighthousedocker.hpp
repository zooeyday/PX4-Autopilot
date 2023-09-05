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

/**
 * @file cm8jl65.cpp
 * @author Claudio Micheli <claudio@auterion.com>
 *
 * Driver for the Lanbao PSK-CM8JL65-CC5 distance sensor.
 * Make sure to disable MAVLINK messages (MAV_0_CONFIG PARAMETER)
 * on the serial port you connect the sensor,i.e TELEM2.
 *
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


// #include <cstdint>
// #include <cstring>
// #include <drivers/drv_hrt.h>
// #include <px4_platform_common/defines.h>
// #include <px4_platform_common/log.h>

// #include <matrix/math.hpp>
// #include <mathlib/mathlib.h>
// #include <matrix/Matrix.hpp>
// #include <lib/conversion/rotation.h>

// #include <cstdint>
// #include <cstring>
// #include <drivers/drv_hrt.h>
// #include <px4_platform_common/defines.h>
// #include <px4_platform_common/log.h>


#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>


#include <uORB/topics/irlock_report.h>


/**
 * Frame format definition
 * | BYTE|  BYTE|  BYTE  |             float               |             float               |             float               |             float               |       uint16_t        |
 * |  1B |  1B  |  B     |              4B                 |              4B                 |              4B                 |              4B                 |           2B          |
 * | sync|  sync|  status|            position_x           |          position_y             |           position_z            |           heading               |           crc         |
 * | 0xAA| 0x33 | STATUS | POS_X_1 POS_X_2 POS_X_3 POS_X_4 | POS_Y_1 POS_Y_2 POS_Y_3 POS_Y_4 | POS_Z_1 POS_Z_2 POS_Z_3 POS_Z_4 | HDG_1    HDG_2   HDG_3   HDG_4  | crc16_LSB | crc16_MSB |
 *
 * Frame data saved for CRC calculation
 */


#define LHD_READ_BUFFER_SIZE  30
#define LHD_SYNC1             0xAA
#define LHD_SYNC2             0x33

using namespace time_literals;

/* Configuration Constants */
static constexpr uint32_t LHDOCK_MEASURE_INTERVAL{16666_us};	// 50ms default sensor conversion time.





/*** LHD protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* General: Header */
typedef struct {
	uint8_t  sync1;
	uint8_t  sync2;
} lhd_header_t;

/* General: Checksum */
typedef struct {
	uint8_t ck_a;
	uint8_t ck_b;
} lhd_checksum_t ;


typedef struct {
	uint8_t status; /**< status */
	float rel_pos_x; /**< relative position x lhd to target [m] */
	float rel_pos_y; /**< relative position y lhd to target [m] */
	float rel_pos_z; /**< relative position z lhd to target [m] */
	float rel_hdg; /**< relative heading lhd to target [m] */
} lhd_payload_rx_t;


typedef struct {
	uint8_t status; /**< status */
	float ned_pos_n; /**< relative position x lhd to target [m] */
	float ned_pos_e; /**< relative position y lhd to target [m] */
	float ned_pos_d; /**< relative position z lhd to target [m] */
	float roll; /**< relative heading lhd to target [m] */
	float pitch; /**< relative heading lhd to target [m] */
	float yaw; /**< relative heading lhd to target [m] */
} lhd_payload_tx_t;


/* General message and payload buffer union */
typedef union {
	lhd_payload_rx_t lhd_payload_rx;
} lhd_rx_buf_t;



/* General message and payload buffer union */
typedef union {
	lhd_payload_tx_t lhd_payload_tx;
} lhd_tx_buf_t;



#pragma pack(pop)
/*** END OF LHD protocol binary message and payload definitions ***/


typedef enum {
	LHD_DECODE_SYNC1 = 0,
	LHD_DECODE_SYNC2,
	LHD_DECODE_PAYLOAD,
	LHD_DECODE_CHKSUM1,
	LHD_DECODE_CHKSUM2,
} lhd_decode_state_t;






class LHDOCKER : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	// LHDOCKER(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	LHDOCKER(const char *portG);

	/** Virtual destructor */
	virtual ~LHDOCKER() override;

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	int init();


	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */



private:


	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void addByteToChecksum(const uint8_t);

	/**
	 * Calculate & add checksum for given buffer
	 */
	void calcChecksum(const uint8_t *buffer, const uint16_t length, lhd_checksum_t *checksum);


	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit(void);


	/**
	 * Parse the binary UBX packet
	 */
	int parseChar(const uint8_t b);


	/**
	 * Start payload rx
	 */
	int payloadRxInit(void);

	/**
	 * Add payload rx byte
	 */
	int payloadRxAdd(const uint8_t b);

	/**
	 * Finish payload rx
	 */
	void payloadRxDone(void);


	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();


	/**
	 * Send a message
	 * @return true on success, false on write error (errno set)
	 */
	bool send(const uint16_t msg, const uint8_t *payload, const uint16_t length);

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
        void stop();


	void get_topics_and_send();

	bool msgsend_to_lhd(const lhd_tx_buf_t *payload, const uint8_t length);


	uORB::Subscription _vehicleLocalPositionSub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitudeSub{ORB_ID(vehicle_attitude)};

	// keep track of which topics we have received
	bool _vehicleLocalPosition_valid{false};
	bool _vehicleAttitude_valid{false};

	vehicle_local_position_s	_vehicleLocalPosition{};
	vehicle_attitude_s		_vehicleAttitude{};






	char 			_port[20] {};

	int 			_file_descriptor{-1};


	lhd_tx_buf_t            _tx_buf{};
	lhd_rx_buf_t            _rx_buf{};

	lhd_decode_state_t      _decode_state{};
	uint8_t 		_rx_ck_a{0};
	uint8_t 		_rx_ck_b{0};
	uint8_t 		_rx_payload_index = 0;
	uint8_t 		_rx_payload_length = 17;
	uint8_t 		_tx_payload_length = 25;



	perf_counter_t 		_comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t 		_sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	uORB::Publication<irlock_report_s> _irlock_report_topic{ORB_ID(irlock_report)};
};
