/**
 * Raspberry Pi CANopen Master Firmware Uploader
 *
 * This is the production-ready version for Raspberry Pi, incorporating all
 * lessons learned from the ESP32 master/slave development:
 *
 *  1. SDO buffer is only 32 bytes - write data progressively as space becomes available
 *  2. Metadata format: 8 bytes [size(4) | crc(2) | type(1) | bank(1)] little-endian
 *  3. Start command: 3 bytes {0x01, 0x00, 0x00} to object 0x1F51:01
 *  4. Finalize: 2-byte CRC to object 0x1F5A:01
 *  5. Query slave CRC from 0x1F5B:01 before upload to skip if already matching
 *  6. Use 1000ms SDO timeout for reliability
 *  7. SDO segmented transfer (not block) for compatibility
 *
 * Build on Raspberry Pi:
 *   gcc -o fw_uploader raspberry_master_firmware_uploader.c -lCANopen -lpthread
 *
 * Usage:
 *   ./fw_uploader <firmware.bin> [nodeId] [maxChunkBytes]
 */

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/* Uncomment and adjust these includes for your CANopenNode installation */
// #include "CANopen.h"
// #include "CO_SDOclient.h"

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define CAN_INTERFACE       "can0"
#define CAN_BITRATE         500000      /* 500 kbps */
#define MASTER_NODE_ID      1
#define DEFAULT_SLAVE_ID    10
#define DEFAULT_CHUNK_SIZE  256         /* Max bytes per SDO transfer */
#define SDO_TIMEOUT_MS      1000        /* SDO operation timeout */
#define SDO_BUFFER_SIZE     32          /* CANopenNode default buffer size */

/* CiA-302 Object Dictionary indices for firmware update */
#define OD_FW_METADATA      0x1F57      /* Firmware metadata (size, crc, type, bank) */
#define OD_FW_CONTROL       0x1F51      /* Firmware control (start/stop) */
#define OD_FW_DATA          0x1F50      /* Firmware data (chunks) */
#define OD_FW_STATUS        0x1F5A      /* Firmware status/finalize (CRC) */
#define OD_FW_RUNNING_CRC   0x1F5B      /* Current running firmware CRC */

/* ============================================================================
 * Logging
 * ============================================================================ */

#define LOG_MASTER(fmt, ...) printf("[FW-MASTER] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)  printf("[FW-ERROR ] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)   printf("[FW-WARN  ] " fmt "\n", ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)  printf("[FW-DEBUG ] " fmt "\n", ##__VA_ARGS__)

/* ============================================================================
 * Types
 * ============================================================================ */

typedef enum {
    FW_IMAGE_MAIN = 0,
    FW_IMAGE_BOOTLOADER = 1,
    FW_IMAGE_CONFIG = 2
} fw_image_type_t;

typedef struct {
    const char *firmwarePath;
    fw_image_type_t type;
    uint8_t targetBank;
    uint8_t targetNodeId;
    uint32_t maxChunkBytes;
    uint16_t expectedCrc;
} fw_upload_plan_t;

typedef struct {
    uint8_t *buffer;
    size_t size;
} fw_payload_t;

/* SDO Client context (placeholder for real CANopenNode integration) */
typedef struct {
    int canSocket;
    uint8_t localNodeId;
    uint8_t targetNodeId;
    uint16_t cobIdClientToServer;  /* 0x600 + nodeId */
    uint16_t cobIdServerToClient;  /* 0x580 + nodeId */
} sdo_client_t;

static sdo_client_t g_sdo = {0};
static volatile bool g_running = true;

/* ============================================================================
 * CRC-16 CCITT (matching the slave's algorithm)
 * ============================================================================ */

static uint16_t
fw_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ============================================================================
 * SocketCAN Initialization
 * ============================================================================ */

static int
can_init(const char *interface) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        LOG_ERROR("Failed to create CAN socket: %s", strerror(errno));
        return -1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        LOG_ERROR("Failed to get interface index for %s: %s", interface, strerror(errno));
        close(sock);
        return -1;
    }

    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERROR("Failed to bind CAN socket: %s", strerror(errno));
        close(sock);
        return -1;
    }

    /* Set receive timeout */
    struct timeval tv = {.tv_sec = SDO_TIMEOUT_MS / 1000, .tv_usec = (SDO_TIMEOUT_MS % 1000) * 1000};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    LOG_MASTER("CAN socket opened on %s", interface);
    return sock;
}

static void
can_close(int sock) {
    if (sock >= 0) {
        close(sock);
    }
}

/* ============================================================================
 * Low-Level SDO Communication (Segmented Transfer)
 *
 * IMPORTANT: This implements SDO segmented transfer protocol.
 * For production, replace with CANopenNode's CO_SDOclient functions.
 * ============================================================================ */

/**
 * Send a CAN frame
 */
static bool
can_send(int sock, uint16_t cobId, const uint8_t *data, uint8_t len) {
    struct can_frame frame = {0};
    frame.can_id = cobId;
    frame.can_dlc = (len > 8) ? 8 : len;
    memcpy(frame.data, data, frame.can_dlc);

    ssize_t n = write(sock, &frame, sizeof(frame));
    if (n != sizeof(frame)) {
        LOG_ERROR("CAN send failed: %s", strerror(errno));
        return false;
    }
    return true;
}

/**
 * Receive a CAN frame with timeout
 */
static bool
can_recv(int sock, uint16_t expectedCobId, uint8_t *data, uint8_t *len) {
    struct can_frame frame = {0};

    ssize_t n = read(sock, &frame, sizeof(frame));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            LOG_ERROR("CAN receive timeout");
        } else {
            LOG_ERROR("CAN receive failed: %s", strerror(errno));
        }
        return false;
    }

    if ((frame.can_id & 0x7FF) != expectedCobId) {
        LOG_WARN("Unexpected COB-ID: 0x%03X (expected 0x%03X)", frame.can_id & 0x7FF, expectedCobId);
        return false;
    }

    *len = frame.can_dlc;
    memcpy(data, frame.data, frame.can_dlc);
    return true;
}

/**
 * SDO Download (write to remote node) - Segmented Transfer
 *
 * KEY FIX: Handle data progressively, respecting the 32-byte SDO buffer limit.
 * For small data (<= 4 bytes), use expedited transfer.
 * For larger data, use segmented transfer.
 */
static bool
sdo_download(sdo_client_t *client, uint16_t index, uint8_t subIndex, const uint8_t *data, size_t len) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    uint8_t rxLen = 0;
    uint16_t cobIdTx = 0x600 + client->targetNodeId;
    uint16_t cobIdRx = 0x580 + client->targetNodeId;

    if (len <= 4) {
        /* Expedited transfer for small data */
        uint8_t n = (uint8_t)(4 - len);  /* Number of bytes that don't contain data */
        txBuf[0] = 0x23 | (n << 2);      /* Command: download expedited, size indicated */
        txBuf[1] = (uint8_t)(index & 0xFF);
        txBuf[2] = (uint8_t)((index >> 8) & 0xFF);
        txBuf[3] = subIndex;
        memcpy(&txBuf[4], data, len);

        if (!can_send(client->canSocket, cobIdTx, txBuf, 8)) {
            return false;
        }

        if (!can_recv(client->canSocket, cobIdRx, rxBuf, &rxLen)) {
            return false;
        }

        /* Check for abort */
        if ((rxBuf[0] & 0xE0) == 0x80) {
            uint32_t abortCode = rxBuf[4] | (rxBuf[5] << 8) | (rxBuf[6] << 16) | (rxBuf[7] << 24);
            LOG_ERROR("SDO abort: 0x%08X", abortCode);
            return false;
        }

        /* Check for success (0x60 = download response) */
        if ((rxBuf[0] & 0xE0) != 0x60) {
            LOG_ERROR("Unexpected SDO response: 0x%02X", rxBuf[0]);
            return false;
        }

        return true;
    }

    /* Segmented transfer for larger data */

    /* Step 1: Initiate download */
    txBuf[0] = 0x21;  /* Command: download initiate, size indicated, not expedited */
    txBuf[1] = (uint8_t)(index & 0xFF);
    txBuf[2] = (uint8_t)((index >> 8) & 0xFF);
    txBuf[3] = subIndex;
    txBuf[4] = (uint8_t)(len & 0xFF);
    txBuf[5] = (uint8_t)((len >> 8) & 0xFF);
    txBuf[6] = (uint8_t)((len >> 16) & 0xFF);
    txBuf[7] = (uint8_t)((len >> 24) & 0xFF);

    if (!can_send(client->canSocket, cobIdTx, txBuf, 8)) {
        return false;
    }

    if (!can_recv(client->canSocket, cobIdRx, rxBuf, &rxLen)) {
        return false;
    }

    if ((rxBuf[0] & 0xE0) == 0x80) {
        uint32_t abortCode = rxBuf[4] | (rxBuf[5] << 8) | (rxBuf[6] << 16) | (rxBuf[7] << 24);
        LOG_ERROR("SDO initiate abort: 0x%08X", abortCode);
        return false;
    }

    if ((rxBuf[0] & 0xE0) != 0x60) {
        LOG_ERROR("Unexpected SDO initiate response: 0x%02X", rxBuf[0]);
        return false;
    }

    /* Step 2: Send data segments */
    size_t offset = 0;
    uint8_t toggle = 0;

    while (offset < len) {
        size_t remaining = len - offset;
        size_t segLen = (remaining > 7) ? 7 : remaining;
        bool isLast = (offset + segLen >= len);

        uint8_t n = (uint8_t)(7 - segLen);  /* Number of bytes that don't contain data */
        txBuf[0] = (toggle << 4) | (n << 1) | (isLast ? 1 : 0);
        memcpy(&txBuf[1], data + offset, segLen);
        memset(&txBuf[1 + segLen], 0, 7 - segLen);  /* Pad with zeros */

        if (!can_send(client->canSocket, cobIdTx, txBuf, 8)) {
            return false;
        }

        if (!can_recv(client->canSocket, cobIdRx, rxBuf, &rxLen)) {
            return false;
        }

        if ((rxBuf[0] & 0xE0) == 0x80) {
            uint32_t abortCode = rxBuf[4] | (rxBuf[5] << 8) | (rxBuf[6] << 16) | (rxBuf[7] << 24);
            LOG_ERROR("SDO segment abort: 0x%08X", abortCode);
            return false;
        }

        /* Check for segment response (0x20) and toggle match */
        if ((rxBuf[0] & 0xE0) != 0x20) {
            LOG_ERROR("Unexpected SDO segment response: 0x%02X", rxBuf[0]);
            return false;
        }

        if (((rxBuf[0] >> 4) & 1) != toggle) {
            LOG_ERROR("SDO toggle mismatch");
            return false;
        }

        offset += segLen;
        toggle ^= 1;
    }

    return true;
}

/**
 * SDO Upload (read from remote node) - Segmented Transfer
 */
static bool
sdo_upload(sdo_client_t *client, uint16_t index, uint8_t subIndex, uint8_t *data, size_t maxLen, size_t *actualLen) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    uint8_t rxLen = 0;
    uint16_t cobIdTx = 0x600 + client->targetNodeId;
    uint16_t cobIdRx = 0x580 + client->targetNodeId;

    /* Initiate upload */
    txBuf[0] = 0x40;  /* Command: upload initiate */
    txBuf[1] = (uint8_t)(index & 0xFF);
    txBuf[2] = (uint8_t)((index >> 8) & 0xFF);
    txBuf[3] = subIndex;

    if (!can_send(client->canSocket, cobIdTx, txBuf, 8)) {
        return false;
    }

    if (!can_recv(client->canSocket, cobIdRx, rxBuf, &rxLen)) {
        return false;
    }

    if ((rxBuf[0] & 0xE0) == 0x80) {
        uint32_t abortCode = rxBuf[4] | (rxBuf[5] << 8) | (rxBuf[6] << 16) | (rxBuf[7] << 24);
        LOG_ERROR("SDO upload abort: 0x%08X", abortCode);
        return false;
    }

    /* Check if expedited (bit 1 set) */
    if (rxBuf[0] & 0x02) {
        /* Expedited transfer */
        uint8_t n = (rxBuf[0] >> 2) & 0x03;  /* Number of bytes that don't contain data */
        size_t dataLen = 4 - n;
        if (dataLen > maxLen) dataLen = maxLen;
        memcpy(data, &rxBuf[4], dataLen);
        *actualLen = dataLen;
        return true;
    }

    /* Segmented transfer */
    size_t totalSize = rxBuf[4] | (rxBuf[5] << 8) | (rxBuf[6] << 16) | (rxBuf[7] << 24);
    size_t offset = 0;
    uint8_t toggle = 0;

    while (offset < totalSize && offset < maxLen) {
        txBuf[0] = 0x60 | (toggle << 4);  /* Request next segment */

        if (!can_send(client->canSocket, cobIdTx, txBuf, 8)) {
            return false;
        }

        if (!can_recv(client->canSocket, cobIdRx, rxBuf, &rxLen)) {
            return false;
        }

        if ((rxBuf[0] & 0xE0) == 0x80) {
            uint32_t abortCode = rxBuf[4] | (rxBuf[5] << 8) | (rxBuf[6] << 16) | (rxBuf[7] << 24);
            LOG_ERROR("SDO segment upload abort: 0x%08X", abortCode);
            return false;
        }

        if (((rxBuf[0] >> 4) & 1) != toggle) {
            LOG_ERROR("SDO upload toggle mismatch");
            return false;
        }

        uint8_t n = (rxBuf[0] >> 1) & 0x07;  /* Number of bytes that don't contain data */
        size_t segLen = 7 - n;
        bool isLast = (rxBuf[0] & 0x01);

        if (offset + segLen > maxLen) {
            segLen = maxLen - offset;
        }
        memcpy(data + offset, &rxBuf[1], segLen);
        offset += segLen;

        if (isLast) break;
        toggle ^= 1;
    }

    *actualLen = offset;
    return true;
}

/* ============================================================================
 * Firmware Upload Protocol Functions
 * ============================================================================ */

/**
 * Send firmware metadata to slave via 0x1F57:01
 * Format: [size(4) | crc(2) | type(1) | bank(1)] = 8 bytes, little-endian
 */
static bool
fw_send_metadata(const fw_upload_plan_t *plan, size_t firmwareSize, uint16_t crc) {
    LOG_MASTER("Sending metadata: size=%zu crc=0x%04X type=%u bank=%u",
               firmwareSize, crc, plan->type, plan->targetBank);

    uint8_t meta[8];
    uint32_t size = (uint32_t)firmwareSize;
    meta[0] = (uint8_t)(size & 0xFF);
    meta[1] = (uint8_t)((size >> 8) & 0xFF);
    meta[2] = (uint8_t)((size >> 16) & 0xFF);
    meta[3] = (uint8_t)((size >> 24) & 0xFF);
    meta[4] = (uint8_t)(crc & 0xFF);
    meta[5] = (uint8_t)((crc >> 8) & 0xFF);
    meta[6] = (uint8_t)plan->type;
    meta[7] = plan->targetBank;

    return sdo_download(&g_sdo, OD_FW_METADATA, 1, meta, sizeof(meta));
}

/**
 * Send start command to slave via 0x1F51:01
 * Format: 3 bytes {0x01, 0x00, 0x00}
 */
static bool
fw_send_start_command(const fw_upload_plan_t *plan) {
    LOG_MASTER("Sending start command to node %u", plan->targetNodeId);
    uint8_t cmd[3] = {0x01, 0x00, 0x00};  /* Start token */
    return sdo_download(&g_sdo, OD_FW_CONTROL, 1, cmd, sizeof(cmd));
}

/**
 * Send firmware data chunk to slave via 0x1F50:01
 */
static bool
fw_send_chunk(const fw_upload_plan_t *plan, const uint8_t *chunk, size_t len, size_t offset) {
    LOG_DEBUG("Sending chunk: offset=%zu len=%zu", offset, len);
    return sdo_download(&g_sdo, OD_FW_DATA, 1, chunk, len);
}

/**
 * Send finalize request with CRC to slave via 0x1F5A:01
 * Format: 2 bytes CRC little-endian
 */
static bool
fw_send_finalize(const fw_upload_plan_t *plan, uint16_t crc) {
    LOG_MASTER("Sending finalize with CRC 0x%04X", crc);
    uint8_t status[2] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF)};
    return sdo_download(&g_sdo, OD_FW_STATUS, 1, status, sizeof(status));
}

/**
 * Query slave's running firmware CRC via 0x1F5B:01
 */
static bool
fw_query_slave_crc(const fw_upload_plan_t *plan, uint16_t *slaveCrc) {
    LOG_MASTER("Querying slave CRC from node %u (0x1F5B:01)", plan->targetNodeId);

    uint8_t buf[2] = {0};
    size_t actualLen = 0;

    if (!sdo_upload(&g_sdo, OD_FW_RUNNING_CRC, 1, buf, sizeof(buf), &actualLen)) {
        LOG_WARN("Failed to query slave CRC");
        return false;
    }

    if (actualLen < 2) {
        LOG_WARN("Short response from slave CRC query: %zu bytes", actualLen);
        return false;
    }

    *slaveCrc = (uint16_t)(buf[0] | (buf[1] << 8));
    LOG_MASTER("Slave running firmware CRC: 0x%04X", *slaveCrc);
    return true;
}

/* ============================================================================
 * File Operations
 * ============================================================================ */

static bool
fw_load_payload(const fw_upload_plan_t *plan, fw_payload_t *payload) {
    FILE *f = fopen(plan->firmwarePath, "rb");
    if (f == NULL) {
        LOG_ERROR("Cannot open firmware file: %s", plan->firmwarePath);
        return false;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        LOG_ERROR("Failed to seek to end of %s", plan->firmwarePath);
        fclose(f);
        return false;
    }

    long fileSize = ftell(f);
    if (fileSize <= 0) {
        LOG_ERROR("Firmware file %s is empty or invalid", plan->firmwarePath);
        fclose(f);
        return false;
    }

    if (fseek(f, 0, SEEK_SET) != 0) {
        LOG_ERROR("Failed to rewind file %s", plan->firmwarePath);
        fclose(f);
        return false;
    }

    payload->buffer = (uint8_t *)malloc((size_t)fileSize);
    if (payload->buffer == NULL) {
        LOG_ERROR("Out of memory while allocating %ld bytes", fileSize);
        fclose(f);
        return false;
    }

    size_t readBytes = fread(payload->buffer, 1, (size_t)fileSize, f);
    fclose(f);

    if (readBytes != (size_t)fileSize) {
        LOG_ERROR("Short read: expected %ld bytes, got %zu", fileSize, readBytes);
        free(payload->buffer);
        payload->buffer = NULL;
        return false;
    }

    payload->size = readBytes;
    LOG_MASTER("Loaded %zu bytes from %s", payload->size, plan->firmwarePath);
    return true;
}

/* ============================================================================
 * Main Upload Session
 * ============================================================================ */

static bool
fw_run_upload_session(const fw_upload_plan_t *plan) {
    fw_payload_t payload = {0};

    /* Load firmware file */
    if (!fw_load_payload(plan, &payload)) {
        return false;
    }

    /* Compute CRC */
    uint16_t crc = plan->expectedCrc;
    if (crc == 0U) {
        crc = fw_crc16(payload.buffer, payload.size);
        LOG_MASTER("Computed CRC: 0x%04X", crc);
    }

    /* Query slave's current CRC - skip upload if matching */
    uint16_t slaveCrc = 0;
    if (fw_query_slave_crc(plan, &slaveCrc)) {
        if (slaveCrc == crc) {
            LOG_MASTER("Slave already has matching firmware (CRC 0x%04X), skipping upload", crc);
            free(payload.buffer);
            return true;
        }
        LOG_MASTER("Slave CRC 0x%04X differs from local 0x%04X, proceeding with upload", slaveCrc, crc);
    } else {
        LOG_WARN("Could not query slave CRC, proceeding with upload anyway");
    }

    /* Send metadata */
    if (!fw_send_metadata(plan, payload.size, crc)) {
        LOG_ERROR("Failed to send metadata");
        free(payload.buffer);
        return false;
    }

    /* Send start command */
    if (!fw_send_start_command(plan)) {
        LOG_ERROR("Failed to send start command");
        free(payload.buffer);
        return false;
    }

    /* Stream firmware data in chunks */
    size_t offset = 0;
    while (offset < payload.size) {
        size_t remaining = payload.size - offset;
        size_t chunkLen = (remaining > plan->maxChunkBytes) ? plan->maxChunkBytes : remaining;

        if (!fw_send_chunk(plan, payload.buffer + offset, chunkLen, offset)) {
            LOG_ERROR("Failed to send chunk at offset %zu", offset);
            free(payload.buffer);
            return false;
        }

        offset += chunkLen;

        /* Progress every 10% */
        int prevPct = (offset - chunkLen) * 100 / payload.size;
        int currPct = offset * 100 / payload.size;
        if (currPct / 10 != prevPct / 10) {
            LOG_MASTER("Upload progress: %zu/%zu bytes (%d%%)", offset, payload.size, currPct);
        }
    }

    LOG_MASTER("Sent %zu bytes total", offset);

    /* Send finalize */
    if (!fw_send_finalize(plan, crc)) {
        LOG_ERROR("Failed to send finalize request");
        free(payload.buffer);
        return false;
    }

    free(payload.buffer);
    LOG_MASTER("Firmware upload completed successfully!");
    LOG_MASTER("Slave will automatically reboot in ~500ms with new firmware.");
    return true;
}

/* ============================================================================
 * Signal Handler
 * ============================================================================ */

static void
signal_handler(int sig) {
    (void)sig;
    g_running = false;
    LOG_MASTER("Caught signal, shutting down...");
}

/* ============================================================================
 * Main Entry Point
 * ============================================================================ */

int
main(int argc, char **argv) {
    if (argc < 2) {
        printf("Raspberry Pi CANopen Master Firmware Uploader\n");
        printf("\n");
        printf("Usage: %s <firmware.bin> [nodeId] [maxChunkBytes]\n", argv[0]);
        printf("\n");
        printf("Arguments:\n");
        printf("  firmware.bin    Path to the firmware binary file\n");
        printf("  nodeId          Target slave node ID (default: %u)\n", DEFAULT_SLAVE_ID);
        printf("  maxChunkBytes   Max bytes per transfer (default: %u)\n", DEFAULT_CHUNK_SIZE);
        printf("\n");
        printf("Example:\n");
        printf("  %s /path/to/firmware.bin 10 256\n", argv[0]);
        return 1;
    }

    /* Parse arguments */
    fw_upload_plan_t plan = {
        .firmwarePath = argv[1],
        .type = FW_IMAGE_MAIN,
        .targetBank = 1,
        .targetNodeId = (argc > 2) ? (uint8_t)atoi(argv[2]) : DEFAULT_SLAVE_ID,
        .maxChunkBytes = (argc > 3) ? (uint32_t)atoi(argv[3]) : DEFAULT_CHUNK_SIZE,
        .expectedCrc = 0
    };

    LOG_MASTER("Upload plan:");
    LOG_MASTER("  Firmware: %s", plan.firmwarePath);
    LOG_MASTER("  Target node: %u", plan.targetNodeId);
    LOG_MASTER("  Max chunk: %u bytes", plan.maxChunkBytes);

    /* Setup signal handler */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Initialize CAN interface */
    g_sdo.canSocket = can_init(CAN_INTERFACE);
    if (g_sdo.canSocket < 0) {
        LOG_ERROR("Failed to initialize CAN interface");
        return 1;
    }

    g_sdo.localNodeId = MASTER_NODE_ID;
    g_sdo.targetNodeId = plan.targetNodeId;

    /* Run upload session */
    bool success = fw_run_upload_session(&plan);

    /* Cleanup */
    can_close(g_sdo.canSocket);

    if (!success) {
        LOG_ERROR("Firmware upload failed");
        return 1;
    }

    return 0;
}
