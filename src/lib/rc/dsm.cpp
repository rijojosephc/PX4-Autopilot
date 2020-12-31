/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file dsm.cpp
 *
 * Serial protocol decoder for the Spektrum DSM* family of protocols.
 *
 * Decodes into the global PPM buffer and updates accordingly.
 */

#include <px4_platform_common/px4_config.h>
#include <board_config.h>
#include <px4_platform_common/defines.h>

#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

#include "dsm.h"
#include "spektrum_rssi.h"
#include "common_rc.h"

#include <include/containers/Bitset.hpp>
#include <lib/mathlib/mathlib.h>

#if defined(__PX4_NUTTX)
#include <nuttx/arch.h>
#define dsm_udelay(arg)    up_udelay(arg)
#else
#define dsm_udelay(arg) px4_usleep(arg)
#endif

//#define DSM_DEBUG

static int dsm_fd = -1;						/**< File handle to the DSM UART */

static uint8_t dsm_partial_frame_count;	/**< Count of bytes received for current dsm frame */
static unsigned dsm_frame_drops;		/**< Count of incomplete DSM frames */
static uint8_t dsm_chan_count;                /**< DSM channel count */

static bool dsm_valid_system(uint8_t system)
{
	switch (system) {
	case 0x00: // SURFACE DSM1
		return true;

	case 0x01: // DSM2 1024 22ms
		return true;

	case 0x02: // DSM2 1024 (MC24)
		return true;

	case 0x12: // DSM2 2048 11ms
		return true;

	case 0x23: // SURFACE DSM2 16.5ms
		PX4_DEBUG("ERROR system: SURFACE DSM2 16.5ms unsupported\n", system);
		// untested
		return true;

	case 0x50: // DSM MARINE
		// untested
		PX4_DEBUG("ERROR system: DSM MARINE unsupported\n", system);
		return true;

	case 0x92: // DSMJ
		// untested
		PX4_DEBUG("ERROR system: DSMJ (%X) unsupported\n", system);
		return true;

	case 0xA2: // DSMX 22ms or DSMR 11ms or DSMR 22ms
		return true;

	case 0xA4: // DSMR 5.5ms
		// unsupported
		PX4_DEBUG("ERROR system: DSMR 5.5ms (%X) unsupported\n", system);
		return true;

	case 0xB2: // DSMX 11ms
		return true;

	case 0xAE: // NOT_BOUND
		PX4_DEBUG("ERROR system: NOT_BOUND (%X) unsupported\n", system);
		return false;

	default:
		break;
	}

	// ERROR
	PX4_DEBUG("ERROR system: %X unsupported\n", system);
	return false;
}

static constexpr bool dsm_1024(uint8_t system)
{
	// 1024 Mode: This format is used only by DSM2/22ms mode. All other modes use 2048 data.
	if (system == 0x01) {
		return true;
	}

	return false;
}

/**
 * Attempt to decode a single channel raw channel datum (1024)
 *
 * @param[in] raw 16 bit raw channel value from dsm frame
 * @param[out] channel pointer to returned channel number
 * @param[out] value pointer to returned channel value
 * @return true=raw value successfully decoded
 */
static bool dsm_decode_channel_1024(uint16_t raw, uint8_t &channel, uint16_t &value)
{
	// 1024 Mode: This format is used only by DSM2/22ms mode. All other modes use 2048 data.
	//  Bits 15-10 Channel ID
	//  Bits 9-0   Servo Position
	static constexpr uint16_t MASK_1024_CHANID = 0xFC00;
	static constexpr uint16_t MASK_1024_SXPOS = 0x03FF;

	channel = (raw & MASK_1024_CHANID) >> 10; // 6 bits

	const uint16_t servo_position = (raw & MASK_1024_SXPOS); // 10 bits

	// PWM_OUT = (ServoPosition x 1.166μs) + Offset
	static constexpr uint16_t offset = 903; // microseconds
	value = roundf(servo_position * 1.166f) + offset;

	if (channel > DSM_MAX_CHANNEL_COUNT) {
		return false;
	}

	// Spektrum range is 903μs to 2097μs (Specification for Spektrum Remote Receiver Interfacing Rev G 9.1)
	if (value < 903 || value > 2097) {
		// if the value is unrealistic, fail the parsing entirely
		return false;
	}

	return true;
}

/**
 * Attempt to decode a single channel raw channel datum (2048)
 *
 * @param[in] raw 16 bit raw channel value from dsm frame
 * @param[out] channel pointer to returned channel number
 * @param[out] value pointer to returned channel value
 * @return true=raw value successfully decoded
 */
static bool dsm_decode_channel_2048(uint16_t raw, uint8_t &channel, uint16_t &value)
{
	// 2048 Mode
	//  Bits 15    Servo Phase
	//  Bits 14-11 Channel ID
	//  Bits 10-0  Servo Position

	// from Spektrum Remote Receiver Interfacing Rev G Page 6
	const bool phase = raw & (2 >> 15); // the phase is part of the X-Plus address (bit 15)
	uint8_t chan = (raw >> 11) & 0x0F;
	uint16_t servo_position = 0;

	if (chan < 12 && !phase) {
		// Normal channels
		static constexpr uint16_t MASK_2048_SXPOS = 0x07FF;
		servo_position = (raw & MASK_2048_SXPOS);

	} else if (chan == 12) {
		// XPlus channels
		chan += ((raw >> 9) & 0x03);

		if (phase) {
			chan += 4;
		}

		if (chan > DSM_MAX_CHANNEL_COUNT) {
			PX4_DEBUG("invalid x-plus channel: %d\n", chan);
			return false;
		}

		servo_position = (raw & 0x01FF) << 2;

	} else {
		PX4_DEBUG("invalid channel: %d\n", chan);
		return false;
	}

	channel = chan;

	// PWM_OUT = (ServoPosition x 0.583μs) + Offset
	static constexpr uint16_t offset = 903; // microseconds
	value = roundf(servo_position * 0.583f) + offset;

	// Spektrum range is 903μs to 2097μs (Specification for Spektrum Remote Receiver Interfacing Rev G 9.1)
	if (value < 903 || value > 2097) {
		// if the value is unrealistic, fail the parsing entirely
		return false;
	}

	return true;
}

int dsm_config(int fd)
{
#ifdef SPEKTRUM_POWER_CONFIG
	// Enable power controls for Spektrum receiver
	SPEKTRUM_POWER_CONFIG();
#endif
#ifdef SPEKTRUM_POWER
	// enable power on DSM connector
	SPEKTRUM_POWER(true);
#endif

	int ret = -1;

	if (fd >= 0) {
		struct termios t;

		/* 115200bps, no parity, one stop bit */
		tcgetattr(fd, &t);
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcsetattr(fd, TCSANOW, &t);

		ret = 0;
	}

	return ret;
}

/**
 * Initialize the DSM receive functionality
 *
 * Open the UART for receiving DSM frames and configure it appropriately
 *
 * @param[in] device Device name of DSM UART
 */
int dsm_init(const char *device)
{
	if (dsm_fd < 0) {
		dsm_fd = open(device, O_RDWR | O_NONBLOCK);
	}

	int ret = dsm_config(dsm_fd);

	if (!ret) {
		return dsm_fd;

	} else {
		return -1;
	}
}

void dsm_deinit()
{
#ifdef SPEKTRUM_POWER_PASSIVE
	// Turn power controls to passive
	SPEKTRUM_POWER_PASSIVE();
#endif

	if (dsm_fd >= 0) {
		close(dsm_fd);
	}

	dsm_fd = -1;
}

#if defined(SPEKTRUM_POWER)
/**
 * Handle DSM satellite receiver bind mode handler
 *
 * @param[in] cmd commands - dsm_bind_power_down, dsm_bind_power_up, dsm_bind_set_rx_out, dsm_bind_send_pulses, dsm_bind_reinit_uart
 * @param[in] pulses Number of pulses for dsm_bind_send_pulses command
 */
void dsm_bind(uint16_t cmd, int pulses)
{
	if (dsm_fd < 0) {
		return;
	}

	switch (cmd) {
	case DSM_CMD_BIND_POWER_DOWN:
		// power down DSM satellite
#if defined(DSM_DEBUG)
		printf("DSM: DSM_CMD_BIND_POWER_DOWN\n");
#endif
		SPEKTRUM_POWER(false);
		break;

	case DSM_CMD_BIND_POWER_UP:
		// power up DSM satellite
#if defined(DSM_DEBUG)
		printf("DSM: DSM_CMD_BIND_POWER_UP\n");
#endif
		SPEKTRUM_POWER(true);
		break;

	case DSM_CMD_BIND_SET_RX_OUT:
		// Set UART RX pin to active output mode
#if defined(DSM_DEBUG)
		printf("DSM: DSM_CMD_BIND_SET_RX_OUT\n");
#endif
		SPEKTRUM_RX_AS_GPIO_OUTPUT();
		break;

	case DSM_CMD_BIND_SEND_PULSES:
		// Pulse RX pin a number of times
#if defined(DSM_DEBUG)
		printf("DSM: DSM_CMD_BIND_SEND_PULSES\n");
#endif

		for (int i = 0; i < pulses; i++) {
			dsm_udelay(120);
			SPEKTRUM_OUT(false);
			dsm_udelay(120);
			SPEKTRUM_OUT(true);
		}

		break;

	case DSM_CMD_BIND_REINIT_UART:
		// Restore USART RX pin to RS232 receive mode
#if defined(DSM_DEBUG)
		printf("DSM: DSM_CMD_BIND_REINIT_UART\n");
#endif
		SPEKTRUM_RX_AS_UART();
		break;

	}
}
#endif

/**
 * Called periodically to check for input data from the DSM UART
 *
 * The DSM* protocol doesn't provide any explicit framing,
 * so we detect dsm frame boundaries by the inter-dsm frame delay.
 * The minimum dsm frame spacing is 11ms; with 16 bytes at 115200bps
 * dsm frame transmission time is ~1.4ms.
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a dsm frame.
 * In the case where byte(s) are dropped from a dsm frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 * Upon receiving a full dsm frame we attempt to decode it.
 *
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned, high order bit 0:10 bit data, 1:11 bit data
 * @param[out] n_butes number of bytes read
 * @param[out] bytes pointer to the buffer of read bytes
 * @param[out] rssi value in percent, if supported, or 127
 * @return true=decoded raw channel values updated, false=no update
 */
bool dsm_input(int fd, uint16_t *values, uint16_t *num_values, bool *dsm_11_bit, uint8_t *n_bytes, uint8_t **bytes,
	       int8_t *rssi, uint8_t max_values)
{
	/*
	 * Fetch bytes, but no more than we would need to complete
	 * a complete frame.
	 */
	dsm_buf_t &dsm_buf = rc_decode_buf.dsm.buf; // DSM_BUFFER_SIZE DSM dsm frame receive buffer

	int ret = read(fd, &dsm_buf[0], sizeof(dsm_buf) / sizeof(dsm_buf[0]));

	/* if the read failed for any reason, just give up here */
	if (ret < 1) {
		return false;

	} else {
		*n_bytes = ret;
		*bytes = &dsm_buf[0];
	}

	/*
	 * Try to decode something with what we got
	 */
	return dsm_parse(&dsm_buf[0], ret, values, num_values, dsm_11_bit, &dsm_frame_drops, rssi, max_values);
}

bool dsm_parse(const uint8_t *frame, const unsigned len, uint16_t *values, uint16_t *num_values, bool *dsm_11_bit,
	       unsigned *frame_drops, int8_t *rssi_percent, uint8_t max_channels)
{
	dsm_frame_t &dsm_frame = rc_decode_buf.dsm.frame; // DSM_BUFFER_SIZE DSM dsm frame receive buffer

	for (unsigned d = 0; d < len; d++) {
		if (dsm_partial_frame_count > DSM_FRAME_SIZE) {
			// reset
			dsm_partial_frame_count = 0;
		}

		dsm_frame[dsm_partial_frame_count++] = frame[d];

		// require valid system byte (dsm_frame[1] to continue
		if ((dsm_partial_frame_count == 2) && !dsm_valid_system(dsm_frame[1])) {
			dsm_partial_frame_count = 0;
		}

		if (dsm_partial_frame_count == DSM_FRAME_SIZE) {
			// fades:    dsm_frame[0] fades
			// system:   dsm_frame[1] system
			// servo[0]: dsm_frame[2] + dsm_frame[3]
			//   ...
			// servo[7]: dsm_frame[14] + dsm_frame[15]

			px4::Bitset<DSM_MAX_CHANNEL_COUNT> channels_found;

			const uint8_t system = dsm_frame[1];

			bool dsm_frame_valid = false;

			for (unsigned i = 2; i < dsm_partial_frame_count; i = i + 2) {
				bool valid_channel_decode = false;

				uint16_t raw = (dsm_frame[i] << 8) | dsm_frame[i + 1];

				if (raw == 0 || raw == 0xFFFF) {
					// ignored
					continue;
				}

				uint8_t channel = 0;
				uint16_t value = 0;

				if (dsm_1024(system)) {
					valid_channel_decode = dsm_decode_channel_1024(raw, channel, value);
					PX4_DEBUG("%d DSM 1024 ch: %d/%d, value: %d, raw: %X\n", valid_channel_decode, channel, dsm_chan_count, value, raw);

				} else {
					valid_channel_decode = dsm_decode_channel_2048(raw, channel, value);
					PX4_DEBUG("%d DSM 2048 ch: %d/%d, value: %d, raw: %X\n", valid_channel_decode, channel, dsm_chan_count, value, raw);
				}

				if (valid_channel_decode) {
					// abort if channel already found, no duplicate channels per DSM frame
					if (channels_found[channel]) {
						valid_channel_decode = false;

					} else {
						channels_found.set(channel);
					}
				}

				if (!valid_channel_decode) {
					// invalid data, reset
					dsm_frame_valid = false;

#if defined(DSM_DEBUG)
					fprintf(stderr,
						"invalid DSM dsm_frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x, resetting\n",
						dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7],
						dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);
#endif // DSM_DEBUG

					// look for next available valid system byte
					bool valid_system_byte_found = false;

					for (unsigned j = 2; j < dsm_partial_frame_count; j++) {
						if (dsm_valid_system(dsm_frame[j])) {
							PX4_DEBUG("found another valid system byte %X at %d\n", dsm_frame[j], j);

							// shift new found system (+ fades) to beginning of dsm_frame buffer
							dsm_partial_frame_count = dsm_partial_frame_count - j - 1;
							memmove(&dsm_frame[0], &dsm_frame[j - 1], dsm_partial_frame_count);

							// zero any remaining bytes
							for (unsigned k = dsm_partial_frame_count + 1; k < DSM_FRAME_SIZE; k++) {
								dsm_frame[k] = 0;
							}

							valid_system_byte_found = true;
							break;
						}
					}

					// otherwise fully reset parse
					if (!valid_system_byte_found) {
						dsm_partial_frame_count = 0;
						(*frame_drops)++;
					}

					break;

				} else {

					dsm_frame_valid = true;

					// update the decoded channel count
					if (channel == dsm_chan_count + 1) {
						dsm_chan_count = channel + 1;
					}

					/*
					* Store the decoded channel into the R/C input buffer, taking into
					* account the different ideas about channel assignement that we have.
					*
					* Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
					* but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
					*/
					switch (channel) {
					case 0:
						channel = 2; // Spektrum Throttle (0) -> 2
						break;

					case 1:
						channel = 0; // Spektrum Aileron (1) -> 0
						break;

					case 2:
						channel = 1; // Spektrum Elevator (2) -> 1

					default:
						break;
					}

					if (channel < max_channels) {
						values[channel] = value;
					}
				}
			}

			if (dsm_frame_valid) {
				/*
				* The first byte represents the rssi in dBm on telemetry receivers with updated
				* firmware, or fades on others. If the value is less than zero, it's rssi.
				* We have other ways to detect bad link metrics, so ignore positive values,
				* but rssi dBm is a useful value.
				*/

				// The SPM4649T with firmware version 1.1RC9 or later will have RSSI in place of fades
				if (rssi_percent) {
					/*
					* RSSI is a signed integer between -42dBm and -92dBm
					* If signal is lost, the value is -128
					*/
					const int8_t dbm = (int8_t)dsm_frame[0];

					if (dbm == -128) {
						*rssi_percent = 0;

					} else if ((dbm >= -92) && (dbm <= -42)) {
						*rssi_percent = spek_dbm_to_percent(dbm);

					} else {
						// if we don't know the rssi, anything over 100 will invalidate it
						*rssi_percent = 127;
					}
				}

				if (dsm_chan_count > *num_values) {
					*num_values = dsm_chan_count;
				}

				// reset parse
				dsm_partial_frame_count = 0;

				return true;
			}
		}
	}

	// return false as default
	return false;
}
