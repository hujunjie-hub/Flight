#include "telemetry_link/telemetry_link.h"
#include "mavlink/mavlink_frame.h"
#include "mavlink/mavlink_rtcm.h"
#include "telemetry/data_radio/data_radio.h"
#include "telemetry/elrs/elrs.h"
#include <rtthread.h>
#include <string.h>

static uint32_t radio_rx, tx_dropped;
static uint8_t tx_sequence;
static uint8_t radio_active;
static uint8_t radio_released;
enum { TX_CAPACITY = 8, TX_FRAME_SIZE = 128 };
static struct rt_messagequeue tx_queue;
rt_align(RT_ALIGN_SIZE) static uint8_t tx_pool[RT_MQ_BUF_SIZE(TX_FRAME_SIZE, TX_CAPACITY)];
static uint8_t tx_frame[TX_FRAME_SIZE];
static unsigned tx_pending;

static uint8_t rx_frame[280]; /* 10-byte header, 255-byte payload, CRC, signature */
static unsigned rx_length;
static uint32_t last_rx_tick;

static uint16_t read_u16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (uint16_t)p[1] << 8);
}

static void receive_byte(uint8_t byte, TelemetryReceive receive)
{
  if (!rx_length && byte != MAVLINK_V2_MAGIC && byte != 0xfeu) return;
  rx_frame[rx_length++] = byte;
  if (rx_length < 3) return;
  const bool v2 = rx_frame[0] == MAVLINK_V2_MAGIC;
  const unsigned header = v2 ? MAVLINK_V2_HEADER_SIZE : 6u;
  const unsigned length = rx_frame[1];
  const unsigned end = header + length;
  if (rx_length < end + 2u + ((v2 && (rx_frame[2] & 1u)) ? 13u : 0u)) return;
  rx_length = 0;
  if (v2 && (rx_frame[2] || rx_frame[8] || rx_frame[9])) return;
  const uint8_t system = rx_frame[v2 ? 5 : 3], component = rx_frame[v2 ? 6 : 4];
  if (!system || system == 1 || !component) return;
  const uint8_t id = rx_frame[v2 ? 7 : 5];
  unsigned extra, base, max;
  switch (id) {
    case PARAM_REQUEST_READ: extra = 214; base = max = 20; break;
    case PARAM_REQUEST_LIST: extra = 159; base = max = 2; break;
    case PARAM_SET: extra = 168; base = max = 23; break;
    case MISSION_REQUEST_LIST: extra = 132; base = 2; max = 3; break;
    case MISSION_COUNT: extra = 221; base = 4; max = 9; break;
    case MISSION_CLEAR_ALL: extra = 232; base = 2; max = 3; break;
    case COMMAND_INT: extra = 158; base = max = 35; break;
    case COMMAND_LONG: extra = 152; base = max = 33; break;
#ifndef FLIGHT_SIM_LINK
    case MAVLINK_GPS_RTCM_DATA: extra = 35; base = max = 2u + MAVLINK_RTCM_CHUNK; break;
#endif
    default: return;
  }
  if ((!v2 && length != base) || (v2 && (!length || length > max))) return;
  uint16_t crc = 0xffffu;
  for (unsigned i = 1; i < end; ++i) crc = mavlink_crc_accumulate(crc, rx_frame[i]);
  if (mavlink_crc_accumulate(crc, (uint8_t)extra) != read_u16(rx_frame + end)) return;
  uint8_t payload[2 + MAVLINK_RTCM_CHUNK] = {0}; /* MAVLink 2 trims trailing zeroes. */
  memcpy(payload, rx_frame + header, length);
  receive(id, payload, system, component);
}

void telemetry_link_release_radio(void)
{
  radio_released = 1u;
  if(radio_active) {
    data_radio_close();
    radio_active=0u;
    (void)rt_mq_detach(&tx_queue);
    tx_pending = 0u;
    rx_length = 0u;
  }
}

void telemetry_link_send(uint32_t id,unsigned extra,const uint8_t *payload,unsigned len) {
  uint8_t frame[TX_FRAME_SIZE];
  if(!radio_active || tx_pending+data_radio_tx_busy()>=TX_CAPACITY) { tx_dropped++; return; }
  const size_t size=wire_encode(id,(uint8_t)extra,payload,len,tx_sequence++,frame,sizeof(frame));
  if(!size || rt_mq_send(&tx_queue,frame,size)!=RT_EOK) { tx_dropped++; return; }
  ++tx_pending;
}
void telemetry_link_poll_transmit(void) {
  if(!radio_active || data_radio_tx_busy() || !tx_pending) return;
  const rt_ssize_t expected=rt_mq_recv(&tx_queue,tx_frame,sizeof(tx_frame),0);
  if(expected<=0) return;
  --tx_pending;
  if(data_radio_write(tx_frame,(size_t)expected)!=(size_t)expected) tx_dropped++;
}
bool telemetry_link_tx_idle(void) {
  return radio_active && !tx_pending && !data_radio_tx_busy();
}

static void open_radio(void)
{
  if (!data_radio_open()) return;
  if (rt_mq_init(&tx_queue, "telem", tx_pool, TX_FRAME_SIZE, sizeof(tx_pool), RT_IPC_FLAG_FIFO) != RT_EOK)
  {
    data_radio_close();
    return;
  }
  radio_active = 1u;
}

uint32_t telemetry_link_poll_rc(void)
{
  uint8_t bytes[256];
  return elrs_open() ? (uint32_t)elrs_read(bytes, sizeof(bytes)) : 0u;
}

uint32_t telemetry_link_poll_rc_channels(uint16_t channels[16], uint8_t *link_quality)
{
  return elrs_open() ? elrs_poll_channels(channels, link_quality) : 0u;
}

bool telemetry_link_send_rc_bind(void)
{
  return elrs_send_bind();
}

void telemetry_link_poll_receive(TelemetryReceive receive) {
  uint8_t bytes[256];
  if (!radio_released) {
    if (!radio_active) open_radio();
    if (radio_active) {
      const uint32_t tick = rt_tick_get();
      if (tick - last_rx_tick > rt_tick_from_millisecond(100)) rx_length = 0;
      const size_t count = data_radio_read(bytes, sizeof(bytes));
      radio_rx += (uint32_t)count;
      if (count) last_rx_tick = tick;
      for (size_t i = 0; i < count; ++i) receive_byte(bytes[i], receive);
    }
  }
}

void telemetry_link_print_diagnostics(void) {
  rt_kprintf("Radio unauthenticated bytes=%u TX drops=%u\n",radio_rx,tx_dropped);
}
