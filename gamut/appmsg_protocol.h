/*
 * Software License Agreement (BSD 3-clause License)
 *
 *  Copyright (c) 2017, iZotope, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of iZotope Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

/* workaround for u-boot: it has it's own definitions for things in the
 * standard libs. Need to be able to exclude them in the u-boot build.
 */
#ifndef APPMSG_PROTOCOL_NO_INCLUDES
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#endif

enum msg_type {
	MSG_DBGLED,
	MSG_LED_RING_UPDATE,
	MSG_ECHO,
	MSG_BUTTON,
	MSG_TOUCH,
	MSG_STATUS,
	MSG_POWER_LED_COLOR,
	MSG_VERSION,
	MSG_POWER_ONOFF,
	MSG_ANIMATION,
	MSG_RAW_TOUCH,
	MSG_I2C,
	MSG_ERROR,
	MSG_TRACE_REPORT,
	MSG_MCU_REVISION,

    // This must be the last item in this enum!
    MSG_NUM_MSG_TYPES,
};

#ifndef __IZPLATFORM_WIN__
#define MSG_STRUCT_ATTRS __attribute__(( packed ))
/* need to turn off the alignment inefficiency warning
   we need to take the hit for things sent over the wire */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
#else
#define MSG_STRUCT_ATTRS
#endif

/* below are the message specific payload structs and defines */
/*****************************************************************************/
/* MSG_DBGLED payload */ 
enum led {
	LED0,
	LED1,
	LED2,

	// This must be the last item in this enum!
	LED_NUM_LEDS,
};

struct msg_dbgled {
	uint8_t id;
	uint8_t state;
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_LED_RING_UPDATE */
#define LED_RING_PIXEL_COUNT 31
#define LED_BUTTON_PIXEL_COUNT 5

/* MSG_LED_RING_UPDATE payload */
struct pixel {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

struct msg_led_update {
	struct pixel pixels[LED_RING_PIXEL_COUNT + LED_BUTTON_PIXEL_COUNT];
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_ECHO
 * no struct needed for echo message.
 * it is just a payload of data of any length up to max packet size
 * i.MX6 sends a payload and the Atmel will reply with that same payload
 */

/*****************************************************************************/
/* MSG_BUTTON payload */
enum button {
	BTN_PLAY,
	BTN_REC,
	BTN_NEW,
	BTN_LEARN,
	BTN_VOLUME,
	BTN_POWER,

	// This must be the last item in this enum!
	BTN_NUM_BUTTONS,
};

/* state is a bitfield where the bit position is from enum button above */
struct msg_button {
	uint32_t state;
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_TOUCH payload */
#define TOUCH_MAX_VALUE (0xFFF)

/* a touch position */
struct msg_touch {
	uint16_t value;
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_STATUS reply structure from atmel to iMX */
enum power_system_state {
	POWER_UNKNOWN = 0, /* initial state or leaving LOW_BATT_BOOT */
	/* normal states */
	POWER_PLUGGED_IN_FULLY_CHARGED,
	POWER_PLUGGED_IN_CHARGING,
	POWER_BATTERY_HIGH_CHARGE, /* 51% - 100% */
	POWER_BATTERY_MED_CHARGE, /* 26% - 50% */
	POWER_BATTERY_LOW_CHARGE, /* 0% - 25% */
	POWER_LOW_BATT_BOOT, /* set when the system is trying to boot with too low a battery */
};

struct msg_status_reply {
	uint16_t light_sensor_value;
	uint16_t power_3p3v_value;
	uint16_t power_vsys_value;
	uint16_t power_phantom_48v_value;
	uint8_t battery_charging;  // !0 means changed, 0 means not charged
	uint8_t battery_power_ok; // !0 means battery is ok, 0 means not ok
	uint16_t raw_touch_count; //! Count of raw_touch events, which can be used to detect 
							  //! if touch data is coming through.
	uint8_t power_system_state; // state taken from the enum power_system_state above
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_POWER_LED_COLOR structure */
struct msg_power_led_color {
	uint8_t enable_override; //!< If 0, atmel reverts to its normal behavior.
	uint8_t rgb[3];			 //!< rgb color
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_VERSION
 * No struct needed
 * iMX6 -> Atmel:
 *  - request to have the version of the Atmel firmware sent back
 * Atmel -> iMX6:
 *  - a string holding the Git SHA the Atmel firmware was built from
 */

/*****************************************************************************/
/* MSG_POWER_ONOFF
 * No struct needed, no payload
 * iMX6 -> Atmel:
 *  - starts a power off by 'pressing' the power on/off button
 */

/*****************************************************************************/
/* MSG_ANIMATION payload */
enum animation_id {
	ANIMATION_BOOT,
	ANIMATION_CRITICAL_BATTERY,
	ANIMATION_SHUTDOWN,
	ANIMATION_ERROR,
	ANIMATION_ATMEL_UPGRADE,
	ANIMATION_NUM_ANIMATIONS
};

struct msg_animation {
	uint8_t id;
	uint8_t state; // 0 => turn all animations off, 1 => turn specified animation on
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_RAW_TOUCH */
/* Atmel -> iMX6
 *  - sends back a struct of the raw sensor readings and reference values
 * iMX6 -> Atmel
 *  - sensor[0] holds the state to set raw touch to
 */
/* iMX6 -> Atmel raw touch states */
enum raw_touch_state {
	RAW_TOUCH_DISABLED,
	RAW_TOUCH_ALL_RAW_SENSORS,
	// this must be the last item in this enum!
	RAW_TOUCH_NUM_STATES,
};

/* Atmel -> iMX6 structure */
struct msg_raw_touch {
	uint16_t sensor[16];
	uint16_t reference[16];
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_I2C
* perform arbitrary i2c reads/writes on the atmel I2C bus
* iMX6 -> Atmel:
*  - uses msg_i2c to setup a read/write operation
* Atmel -> iMX6:
*  - sends back the resuts of a previous i2c operation
*/
enum i2c_op {
	I2C_OP_READ,
	I2C_OP_WRITE,
	NUM_I2C_OPS
};

struct msg_i2c {
	uint8_t op; /* I2C_OP_READ or I2C_OP_WRITE */
	uint8_t device_address; /* Address of device on i2c bus */
	uint8_t reg_address; /* Address on the device */
	uint8_t success; /* 1 => op successful, 0 => op unsuccessful */
	uint8_t value; /* value to write for I2C_OP_WRITE, value read for I2_OP_READ */
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_ERROR
 * No struct needed for MSG_ERROR.
 * i.MX6->Atmel:
 *  - no payload: request for the current error state bit vector
 *  - payload: clear the current error state bit vector using the payload as a mask
 * Atmel->i.MX6:
 *  - payload is a bit vector of the current error state
 */

/*****************************************************************************/
/* MSG_TRACE_REPORT
 * Request or send trace reports
 * iMX6 -> Atmel:
 *  - request for trace reports
 * Atmel -> iMX6:
 *  - an individual trace report
 */
#define TRACE_MESSAGE_SIZE 64 /* Bound by APPMSG_MAX_TX_SIZE */

enum trace_id {
	TRACE_FAN_FAULT, //!< FAN issued a fault interrupt
	TRACE_FAN_READ, //!< Register values from a FAN read
	TRACE_FAN_WRITE, //!< Value written to a FAN register
	TRACE_POWER_STATUS, //!< Vsys reading after a FAN interrupt
	TRACE_SLEEP, //!< Atmel went to sleep or woke up
	TRACE_I2C_FAILURE, //!< An i2c operation failed
	TRACE_NOBAT_RECOVERY_ATTEMPTED, //!< We attempted to do SW-based NOBAT recovery
	NUM_TRACE
};

struct msg_trace_report {
	//! filter to only this trace id, or -1 for all traces
	int id;
	
	//! Trace item message, or empty string to indicate done with trace report
	char msg[TRACE_MESSAGE_SIZE];
	
	//! Trace item time recorded, in microseconds
	uint64_t usec;
} MSG_STRUCT_ATTRS;

/*****************************************************************************/
/* MSG_MCU_REVISION
* Get the MCU revision
* iMX6 -> Atmel:
*  - no payload
* Atmel -> iMX6:
*  - sends back the board revision as a 4-bit code
*/
struct msg_mcu_revision_reply {
	uint8_t revision;
};

/* pop warnings for the packed structs above */
#ifndef __IZPLATFORM_WIN__
#pragma GCC diagnostic pop
#endif

// These magic bytes appear before the SHA in the atmel firmware .bin 
#define IZOTOPE_GITHEAD_MAGIC_BYTES { 0xc1, 0xea, 0x7d, 0xa1 }
