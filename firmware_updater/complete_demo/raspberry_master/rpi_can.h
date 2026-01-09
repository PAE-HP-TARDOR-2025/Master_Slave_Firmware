/*
 * Minimal SocketCAN helpers for Raspberry Pi.
 * Provides init/send/recv wrappers used by the CANopen driver layer.
*/

#ifndef RPI_CAN_H
#define RPI_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Open the SocketCAN interface (e.g., "can0") and return a socket fd, or -1 on error. */
int rpi_can_init(const char* ifname, int bitrate_kbps);

/** Send a CAN frame. Returns true on success. */
bool rpi_can_send(int sock, uint32_t id, const uint8_t* data, uint8_t len);

/** Receive a CAN frame (blocking). Returns number of bytes in data, or -1 on error. */
int rpi_can_recv(int sock, uint32_t* id, uint8_t* data, size_t max_len);

/** Close the socket. */
void rpi_can_close(int sock);

#ifdef __cplusplus
}
#endif

#endif /* RPI_CAN_H */
