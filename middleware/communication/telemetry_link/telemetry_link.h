#ifndef TELEMETRY_LINK_H
#define TELEMETRY_LINK_H

#include <stdbool.h>
#include <stdint.h>

enum {
  PARAM_REQUEST_READ = 20, PARAM_REQUEST_LIST = 21, PARAM_VALUE = 22, PARAM_SET = 23,
  MISSION_REQUEST_LIST = 43, MISSION_COUNT = 44, MISSION_CLEAR_ALL = 45,
  MISSION_ACK = 47, RC_CHANNELS = 65, COMMAND_INT = 75, COMMAND_LONG = 76,
  COMMAND_ACK = 77, STATUSTEXT = 253, AUTOPILOT_VERSION = 148, EXTENDED_SYS_STATE = 245
};

typedef void (*TelemetryReceive)(uint8_t id, const uint8_t *payload,
                                 uint8_t system, uint8_t component);
/* The decoded payload is zero-filled to 35 bytes and valid during the callback. */
void telemetry_link_poll_receive(TelemetryReceive receive);
uint32_t telemetry_link_poll_rc(void);
/* CRSF decode: returns the number of new RC frames and copies the latest
 * 16 channels plus uplink link quality. */
uint32_t telemetry_link_poll_rc_channels(uint16_t channels[16], uint8_t *link_quality);
/* Request receiver bind through the telemetry/RC device adapter. */
bool telemetry_link_send_rc_bind(void);
/* Request receiver binding without exposing the ELRS device API to applications. */
bool telemetry_link_request_rc_bind(void);
void telemetry_link_poll_transmit(void);
void telemetry_link_send(uint32_t id, unsigned crc_extra, const uint8_t *payload, unsigned length);
bool telemetry_link_tx_idle(void);
void telemetry_link_print_diagnostics(void);

/* Select HIL ownership of the shared radio UART before starting the I/O tasks.
 * ELRS reception stays on its own UART. */
void telemetry_link_release_radio(void);

#endif
