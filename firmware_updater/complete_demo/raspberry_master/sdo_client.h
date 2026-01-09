/**
 * @file sdo_client.h
 * @brief Lightweight SDO Client for CANopen firmware updates
 *
 * This is a standalone SDO client implementation using raw SocketCAN frames.
 * It implements SDO expedited and segmented transfers as defined in CiA 301.
 *
 * Key Implementation Notes (Lessons Learned from ESP32 Development):
 * - SDO download uses COB-ID 0x600 + nodeId (TX) and 0x580 + nodeId (RX)
 * - Expedited transfers for data <= 4 bytes
 * - Segmented transfers for data > 4 bytes
 * - Toggle bit must alternate for segmented transfers
 * - Timeout handling is critical for reliable operation
 */

#ifndef SDO_CLIENT_H
#define SDO_CLIENT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SDO Command Specifiers (CiA 301) */
#define SDO_CCS_DOWNLOAD_INIT_REQ    0x20  /* Client Command Specifier: Download initiate request */
#define SDO_CCS_DOWNLOAD_SEG_REQ     0x00  /* Client Command Specifier: Download segment request */
#define SDO_CCS_UPLOAD_INIT_REQ      0x40  /* Client Command Specifier: Upload initiate request */
#define SDO_CCS_UPLOAD_SEG_REQ       0x60  /* Client Command Specifier: Upload segment request */

#define SDO_SCS_DOWNLOAD_INIT_RESP   0x60  /* Server Command Specifier: Download initiate response */
#define SDO_SCS_DOWNLOAD_SEG_RESP    0x20  /* Server Command Specifier: Download segment response */
#define SDO_SCS_UPLOAD_INIT_RESP     0x40  /* Server Command Specifier: Upload initiate response */
#define SDO_SCS_UPLOAD_SEG_RESP      0x00  /* Server Command Specifier: Upload segment response */

#define SDO_ABORT                    0x80  /* Abort transfer */

/* SDO Abort Codes */
#define SDO_ABORT_NONE               0x00000000
#define SDO_ABORT_TOGGLE_ERROR       0x05030000
#define SDO_ABORT_TIMEOUT            0x05040000
#define SDO_ABORT_INVALID_CS         0x05040001
#define SDO_ABORT_OBJ_NOT_EXIST      0x06020000
#define SDO_ABORT_WRITE_ONLY         0x06010001
#define SDO_ABORT_READ_ONLY          0x06010002

/* Configuration */
#define SDO_TIMEOUT_MS               3000   /* Timeout for SDO transfers */
#define SDO_MAX_RETRIES              3      /* Number of retries on timeout */

/**
 * Initialize the SDO client with a SocketCAN socket
 * @param can_socket The open SocketCAN socket descriptor
 */
void sdo_client_init(int can_socket);

/**
 * Perform an SDO download (write to slave)
 * Automatically selects expedited or segmented transfer based on data length.
 *
 * @param nodeId    Target node ID (1-127)
 * @param index     Object dictionary index
 * @param subIndex  Object dictionary subindex
 * @param data      Data to write
 * @param len       Length of data
 * @return true on success, false on failure
 */
bool sdo_download(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                  const uint8_t* data, size_t len);

/**
 * Perform an SDO upload (read from slave)
 * Automatically handles expedited or segmented responses.
 *
 * @param nodeId    Target node ID (1-127)
 * @param index     Object dictionary index
 * @param subIndex  Object dictionary subindex
 * @param data      Buffer to receive data
 * @param maxLen    Maximum bytes to read
 * @param actualLen Actual bytes received (output)
 * @return true on success, false on failure
 */
bool sdo_upload(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                uint8_t* data, size_t maxLen, size_t* actualLen);

/**
 * Get the last SDO abort code
 * @return The abort code from the last failed operation
 */
uint32_t sdo_get_last_abort_code(void);

#ifdef __cplusplus
}
#endif

#endif /* SDO_CLIENT_H */
