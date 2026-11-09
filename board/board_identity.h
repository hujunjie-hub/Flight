/* Per-boot identity generated from the on-chip RNG. */
#ifndef BOARD_IDENTITY_H
#define BOARD_IDENTITY_H

#include <stdint.h>

uint64_t board_boot_id(void);
uint32_t board_reset_flags(void);

#endif /* BOARD_IDENTITY_H */
