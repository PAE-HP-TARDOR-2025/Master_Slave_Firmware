/**
 * @file sdo_client.c
 * @brief Lightweight SDO Client Implementation for CANopen
 *
 * This implementation provides SDO expedited and segmented transfers
 * using raw SocketCAN frames. It's designed for firmware update operations
 * where the Raspberry Pi acts as the CANopen master.
 *
 * Key Implementation Details:
 * - TX COB-ID: 0x600 + nodeId (SDO request to slave)
 * - RX COB-ID: 0x580 + nodeId (SDO response from slave)
 * - Expedited: data <= 4 bytes in single frame
 * - Segmented: data > 4 bytes, 7 bytes per segment with toggle bit
 *
 * CRC-16 CCITT (X.25) polynomial 0x1021, init 0xFFFF - same as slave
 */

#include "sdo_client.h"
#include "rpi_can.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <sys/select.h>
#include <unistd.h>

#define SDO_LOG(fmt, ...) printf("[SDO] " fmt, ##__VA_ARGS__)
#define SDO_ERR(fmt, ...) fprintf(stderr, "[SDO-ERR] " fmt, ##__VA_ARGS__)
#define SDO_DBG(fmt, ...) /* printf("[SDO-DBG] " fmt, ##__VA_ARGS__) */

static int g_can_socket = -1;
static uint32_t g_last_abort_code = SDO_ABORT_NONE;

void sdo_client_init(int can_socket) {
    g_can_socket = can_socket;
    g_last_abort_code = SDO_ABORT_NONE;
}

uint32_t sdo_get_last_abort_code(void) {
    return g_last_abort_code;
}

/**
 * Get current time in milliseconds (monotonic clock)
 */
static uint64_t get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/**
 * Wait for an SDO response with timeout
 * @param expectedCobId The COB-ID we expect (0x580 + nodeId)
 * @param response      Buffer for received 8-byte response
 * @param timeoutMs     Timeout in milliseconds
 * @return true if response received, false on timeout
 */
static bool wait_for_response(uint32_t expectedCobId, uint8_t response[8], uint32_t timeoutMs) {
    uint64_t startTime = get_time_ms();
    
    while ((get_time_ms() - startTime) < timeoutMs) {
        /* Use select with short timeout to check for data */
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(g_can_socket, &readfds);
        
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;  /* 10ms polling interval */
        
        int ret = select(g_can_socket + 1, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            SDO_ERR("select() failed: %s\n", strerror(errno));
            return false;
        }
        
        if (ret > 0 && FD_ISSET(g_can_socket, &readfds)) {
            uint32_t rxId = 0;
            uint8_t rxData[8] = {0};
            int len = rpi_can_recv(g_can_socket, &rxId, rxData, sizeof(rxData));
            
            if (len > 0) {
                SDO_DBG("RX: id=0x%03X dlc=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
                        rxId, len, rxData[0], rxData[1], rxData[2], rxData[3],
                        rxData[4], rxData[5], rxData[6], rxData[7]);
                
                if (rxId == expectedCobId) {
                    memcpy(response, rxData, 8);
                    return true;
                }
                /* Not our message, continue waiting */
            }
        }
    }
    
    SDO_ERR("Timeout waiting for response (COB-ID 0x%03X)\n", expectedCobId);
    g_last_abort_code = SDO_ABORT_TIMEOUT;
    return false;
}

/**
 * Send an SDO request frame
 */
static bool send_sdo_request(uint8_t nodeId, const uint8_t data[8]) {
    uint32_t cobId = 0x600 + nodeId;  /* SDO client -> server */
    
    SDO_DBG("TX: id=0x%03X data=[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
            cobId, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    
    return rpi_can_send(g_can_socket, cobId, data, 8);
}

/**
 * Parse SDO abort code from response
 */
static uint32_t parse_abort_code(const uint8_t response[8]) {
    return (uint32_t)response[4] |
           ((uint32_t)response[5] << 8) |
           ((uint32_t)response[6] << 16) |
           ((uint32_t)response[7] << 24);
}

bool sdo_download(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                  const uint8_t* data, size_t len) {
    if (g_can_socket < 0) {
        SDO_ERR("SDO client not initialized\n");
        return false;
    }
    
    g_last_abort_code = SDO_ABORT_NONE;
    uint32_t rxCobId = 0x580 + nodeId;  /* SDO server -> client */
    uint8_t frame[8] = {0};
    uint8_t response[8] = {0};
    
    SDO_LOG("Download: node=%u idx=0x%04X sub=%u len=%zu\n", nodeId, index, subIndex, len);
    
    if (len <= 4) {
        /* ===== EXPEDITED DOWNLOAD ===== */
        /* CCS = 0x20, with size indicator (s=1) and expedited (e=1) */
        /* Byte 0: ccs=001, n=4-len, e=1, s=1 => 0x23 for 4 bytes, 0x27 for 3, 0x2B for 2, 0x2F for 1 */
        uint8_t n = (uint8_t)(4 - len);
        frame[0] = 0x23 | (n << 2);  /* 0x23 + n*4 */
        frame[1] = (uint8_t)(index & 0xFF);
        frame[2] = (uint8_t)((index >> 8) & 0xFF);
        frame[3] = subIndex;
        
        /* Copy data to bytes 4-7 */
        for (size_t i = 0; i < len; i++) {
            frame[4 + i] = data[i];
        }
        
        if (!send_sdo_request(nodeId, frame)) {
            SDO_ERR("Failed to send expedited download request\n");
            return false;
        }
        
        if (!wait_for_response(rxCobId, response, SDO_TIMEOUT_MS)) {
            return false;
        }
        
        /* Check response: expect 0x60 (download initiate response) */
        if ((response[0] & 0xE0) == SDO_ABORT) {
            g_last_abort_code = parse_abort_code(response);
            SDO_ERR("Download aborted: 0x%08X\n", g_last_abort_code);
            return false;
        }
        
        if ((response[0] & 0xE0) != SDO_SCS_DOWNLOAD_INIT_RESP) {
            SDO_ERR("Unexpected response: 0x%02X (expected 0x60)\n", response[0]);
            return false;
        }
        
        SDO_LOG("Expedited download complete\n");
        return true;
        
    } else {
        /* ===== SEGMENTED DOWNLOAD ===== */
        /* 1. Send download initiate with size indication */
        frame[0] = 0x21;  /* ccs=001, n=0, e=0, s=1 (size indicated) */
        frame[1] = (uint8_t)(index & 0xFF);
        frame[2] = (uint8_t)((index >> 8) & 0xFF);
        frame[3] = subIndex;
        /* Bytes 4-7: data size in little-endian */
        frame[4] = (uint8_t)(len & 0xFF);
        frame[5] = (uint8_t)((len >> 8) & 0xFF);
        frame[6] = (uint8_t)((len >> 16) & 0xFF);
        frame[7] = (uint8_t)((len >> 24) & 0xFF);
        
        if (!send_sdo_request(nodeId, frame)) {
            SDO_ERR("Failed to send segmented download initiate\n");
            return false;
        }
        
        if (!wait_for_response(rxCobId, response, SDO_TIMEOUT_MS)) {
            return false;
        }
        
        if ((response[0] & 0xE0) == SDO_ABORT) {
            g_last_abort_code = parse_abort_code(response);
            SDO_ERR("Download initiate aborted: 0x%08X\n", g_last_abort_code);
            return false;
        }
        
        if ((response[0] & 0xE0) != SDO_SCS_DOWNLOAD_INIT_RESP) {
            SDO_ERR("Unexpected initiate response: 0x%02X\n", response[0]);
            return false;
        }
        
        /* 2. Send data segments */
        size_t offset = 0;
        uint8_t toggle = 0;  /* Toggle bit alternates: 0, 1, 0, 1, ... */
        
        while (offset < len) {
            size_t remaining = len - offset;
            size_t segLen = (remaining > 7) ? 7 : remaining;
            bool lastSegment = (remaining <= 7);
            
            /* Build segment request */
            uint8_t n = (uint8_t)(7 - segLen);  /* Number of empty bytes */
            frame[0] = (toggle << 4) | (n << 1) | (lastSegment ? 1 : 0);
            
            /* Copy segment data to bytes 1-7 */
            memset(&frame[1], 0, 7);
            memcpy(&frame[1], data + offset, segLen);
            
            if (!send_sdo_request(nodeId, frame)) {
                SDO_ERR("Failed to send segment at offset %zu\n", offset);
                return false;
            }
            
            if (!wait_for_response(rxCobId, response, SDO_TIMEOUT_MS)) {
                return false;
            }
            
            if ((response[0] & 0xE0) == SDO_ABORT) {
                g_last_abort_code = parse_abort_code(response);
                SDO_ERR("Segment download aborted at offset %zu: 0x%08X\n", offset, g_last_abort_code);
                return false;
            }
            
            /* Check toggle bit matches */
            uint8_t respToggle = (response[0] >> 4) & 0x01;
            if (respToggle != toggle) {
                SDO_ERR("Toggle bit mismatch at offset %zu: expected %u, got %u\n", offset, toggle, respToggle);
                g_last_abort_code = SDO_ABORT_TOGGLE_ERROR;
                return false;
            }
            
            offset += segLen;
            toggle ^= 1;  /* Alternate toggle bit */
            
            /* Progress indicator for large transfers */
            if (offset % 10240 == 0 || lastSegment) {
                SDO_LOG("Download progress: %zu / %zu bytes\n", offset, len);
            }
        }
        
        SDO_LOG("Segmented download complete: %zu bytes\n", len);
        return true;
    }
}

bool sdo_upload(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                uint8_t* data, size_t maxLen, size_t* actualLen) {
    if (g_can_socket < 0) {
        SDO_ERR("SDO client not initialized\n");
        return false;
    }
    
    g_last_abort_code = SDO_ABORT_NONE;
    uint32_t rxCobId = 0x580 + nodeId;
    uint8_t frame[8] = {0};
    uint8_t response[8] = {0};
    
    *actualLen = 0;
    
    SDO_LOG("Upload: node=%u idx=0x%04X sub=%u maxLen=%zu\n", nodeId, index, subIndex, maxLen);
    
    /* Send upload initiate request */
    frame[0] = SDO_CCS_UPLOAD_INIT_REQ;  /* 0x40 */
    frame[1] = (uint8_t)(index & 0xFF);
    frame[2] = (uint8_t)((index >> 8) & 0xFF);
    frame[3] = subIndex;
    /* Bytes 4-7: reserved (0) */
    
    if (!send_sdo_request(nodeId, frame)) {
        SDO_ERR("Failed to send upload initiate request\n");
        return false;
    }
    
    if (!wait_for_response(rxCobId, response, SDO_TIMEOUT_MS)) {
        return false;
    }
    
    /* Check for abort */
    if ((response[0] & 0xE0) == SDO_ABORT) {
        g_last_abort_code = parse_abort_code(response);
        SDO_ERR("Upload aborted: 0x%08X\n", g_last_abort_code);
        return false;
    }
    
    /* Parse response type */
    uint8_t scs = response[0] & 0xE0;
    
    if (scs == SDO_SCS_UPLOAD_INIT_RESP) {
        /* Check expedited bit (e) and size indicator bit (s) */
        bool expedited = (response[0] & 0x02) != 0;
        bool sizeIndicated = (response[0] & 0x01) != 0;
        
        if (expedited) {
            /* ===== EXPEDITED UPLOAD ===== */
            /* Data is in bytes 4-7, n indicates empty bytes */
            uint8_t n = (response[0] >> 2) & 0x03;
            size_t dataLen = 4 - n;
            
            if (dataLen > maxLen) {
                dataLen = maxLen;
            }
            
            memcpy(data, &response[4], dataLen);
            *actualLen = dataLen;
            
            SDO_LOG("Expedited upload complete: %zu bytes\n", *actualLen);
            return true;
            
        } else {
            /* ===== SEGMENTED UPLOAD ===== */
            /* Size is in bytes 4-7 if sizeIndicated */
            size_t totalSize = 0;
            if (sizeIndicated) {
                totalSize = (size_t)response[4] |
                            ((size_t)response[5] << 8) |
                            ((size_t)response[6] << 16) |
                            ((size_t)response[7] << 24);
                SDO_LOG("Segmented upload, total size: %zu bytes\n", totalSize);
            }
            
            /* Request segments */
            size_t offset = 0;
            uint8_t toggle = 0;
            bool complete = false;
            
            while (!complete) {
                /* Send segment request */
                memset(frame, 0, 8);
                frame[0] = SDO_CCS_UPLOAD_SEG_REQ | (toggle << 4);  /* 0x60 | toggle */
                
                if (!send_sdo_request(nodeId, frame)) {
                    SDO_ERR("Failed to send segment request at offset %zu\n", offset);
                    return false;
                }
                
                if (!wait_for_response(rxCobId, response, SDO_TIMEOUT_MS)) {
                    return false;
                }
                
                if ((response[0] & 0xE0) == SDO_ABORT) {
                    g_last_abort_code = parse_abort_code(response);
                    SDO_ERR("Segment upload aborted at offset %zu: 0x%08X\n", offset, g_last_abort_code);
                    return false;
                }
                
                /* Verify toggle bit */
                uint8_t respToggle = (response[0] >> 4) & 0x01;
                if (respToggle != toggle) {
                    SDO_ERR("Toggle mismatch at offset %zu\n", offset);
                    g_last_abort_code = SDO_ABORT_TOGGLE_ERROR;
                    return false;
                }
                
                /* Extract segment data */
                uint8_t n = (response[0] >> 1) & 0x07;  /* Number of empty bytes */
                size_t segLen = 7 - n;
                complete = (response[0] & 0x01) != 0;   /* c bit = last segment */
                
                /* Copy data if there's room */
                if (offset + segLen <= maxLen) {
                    memcpy(data + offset, &response[1], segLen);
                } else if (offset < maxLen) {
                    memcpy(data + offset, &response[1], maxLen - offset);
                }
                
                offset += segLen;
                toggle ^= 1;
            }
            
            *actualLen = (offset > maxLen) ? maxLen : offset;
            SDO_LOG("Segmented upload complete: %zu bytes\n", *actualLen);
            return true;
        }
    }
    
    SDO_ERR("Unexpected response SCS: 0x%02X\n", scs);
    return false;
}
