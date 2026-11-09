#ifndef ALGORITHM_CHECKS_H
#define ALGORITHM_CHECKS_H
#include "messages/message_ids.h"
#include "ports/port_api.h"
#include <math.h>
#include <string.h>

static inline uint64_t min_time(uint64_t a, uint64_t b) {
  return a < b ? a : b;
}
static inline int header_valid(const ContractHeader *h, uint64_t now) {
  return h && h->schema_major == CONTRACT_SCHEMA_MAJOR &&
         h->schema_minor == CONTRACT_SCHEMA_MINOR &&
         h->profile_id == PRODUCT_PROFILE_ID && h->boot_id &&
         h->clock_domain_id == CLOCK_DOMAIN_FCU_MONOTONIC &&
         (h->validity_flags & 3u) == 3u &&
         h->timestamp_sample_us <= h->timestamp_publish_us &&
         h->timestamp_publish_us <= now && now < h->valid_until_us &&
         h->event_time_uncertainty_us <= 100u &&
         (h->timestamp_flags & TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED) != 0 &&
         h->timestamp_quality >= TIMESTAMP_QUALITY_HARDWARE_CAPTURED &&
         h->timestamp_quality <= TIMESTAMP_QUALITY_EXTERNAL_MAPPED;
}
static inline int same_clock(const ContractHeader *a,
                                const ContractHeader *b) {
  return a->boot_id == b->boot_id && a->time_epoch == b->time_epoch;
}
static inline void algorithm_header(ContractHeader *h,
                                       const ContractHeader *base,
                                       uint16_t type, uint16_t source,
                                       uint32_t seq, uint64_t now,
                                       uint64_t expiry) {
  *h = *base;
  h->message_type = type;
  h->source_id = source;
  h->sequence = seq;
  h->source_clock_domain_id = 0;
  h->source_mapping_epoch = 0;
  h->timestamp_publish_us = now;
  h->valid_until_us = expiry;
  h->validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;
}
static inline int finite_array(const float *v, unsigned n) {
  unsigned i;
  for (i = 0; i < n; ++i)
    if (!isfinite(v[i]))
      return 0;
  return 1;
}
static inline float clip(float x, float lo, float hi) {
  return fminf(hi, fmaxf(lo, x));
}
#endif
