#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "CANopen.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the firmware download object handlers for the CANopen slave. */
bool fw_server_init(CO_t *co);

/** Return the running firmware CRC as computed at server init. */
uint16_t fw_server_get_running_crc(void);

#ifdef __cplusplus
}
#endif
