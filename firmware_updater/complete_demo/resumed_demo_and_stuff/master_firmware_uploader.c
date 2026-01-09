/*
 * CANopen Master Firmware Uploader - Reference Implementation
 *
 * This is a cross-platform reference implementation incorporating all lessons learned
 * from ESP32 master/slave development:
 *
 *  1. SDO buffer is only 32 bytes - write data progressively as space becomes available
 *  2. Metadata format: 8 bytes [size(4) | crc(2) | type(1) | bank(1)] little-endian
 *  3. Start command: 3 bytes {0x01, 0x00, 0x00} to object 0x1F51:01
 *  4. Finalize: 2-byte CRC to object 0x1F5A:01
 *  5. Query slave CRC from 0x1F5B:01 before upload to skip if already matching
 *  6. Use 1000ms SDO timeout for reliability
 *  7. SDO segmented transfer (not block) for compatibility
 *
 * Replace the stubbed "send_*" helpers with real CANopenNode SDO client calls.
 * For Raspberry Pi, see: raspberry_master_firmware_uploader.c
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define log_master(fmt, ...) printf("[FW-MASTER] " fmt, ##__VA_ARGS__)
#define log_error(fmt, ...)  printf("[FW-ERROR ] " fmt, ##__VA_ARGS__)
#define log_warn(fmt, ...)   printf("[FW-WARN  ] " fmt, ##__VA_ARGS__)
#define log_debug(fmt, ...)  printf("[FW-DEBUG ] " fmt, ##__VA_ARGS__)

#define RETURN_IF_FALSE(cond, msg, ...)                                                                                \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            log_error(msg "\n", ##__VA_ARGS__);                                                                       \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

enum {
    FW_META_INDEX = 0x1F57,     /* Metadata: size, crc, type, bank */
    FW_CTRL_INDEX = 0x1F51,     /* Control: start command */
    FW_DATA_INDEX = 0x1F50,     /* Data: firmware chunks */
    FW_STATUS_INDEX = 0x1F5A,   /* Status: finalize with CRC */
    FW_RUNNING_CRC_INDEX = 0x1F5B  /* Query running firmware CRC */
};

typedef enum {
    FW_IMAGE_MAIN = 0,
    FW_IMAGE_BOOTLOADER = 1,
    FW_IMAGE_CONFIG = 2
} fw_image_type_t;

typedef struct {
    const char* firmwarePath;
    fw_image_type_t type;
    uint8_t targetBank;
    uint8_t targetNodeId;
    uint32_t maxChunkBytes;
    uint16_t expectedCrc;
    uint32_t sdoTimeoutMs;  /* SDO operation timeout (recommend 1000ms) */
} fw_upload_plan_t;

typedef struct {
    uint8_t* buffer;
    size_t size;
} fw_payload_t;

/* Read the firmware file from disk into memory so it can be sent over the bus. */
static bool
fw_load_payload(const fw_upload_plan_t* plan, fw_payload_t* payload) {
    FILE* f = fopen(plan->firmwarePath, "rb");
    RETURN_IF_FALSE(f != NULL, "Cannot open firmware file %s", plan->firmwarePath);

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        log_error("Failed to seek to end of %s\n", plan->firmwarePath);
        return false;
    }

    long fileSize = ftell(f);
    RETURN_IF_FALSE(fileSize > 0, "Firmware file %s is empty", plan->firmwarePath);

    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        log_error("Failed to rewind file %s\n", plan->firmwarePath);
        return false;
    }

    payload->buffer = (uint8_t*)malloc((size_t)fileSize);
    RETURN_IF_FALSE(payload->buffer != NULL, "Out of memory while reading firmware");

    size_t readBytes = fread(payload->buffer, 1, (size_t)fileSize, f);
    fclose(f);
    RETURN_IF_FALSE(readBytes == (size_t)fileSize, "Short read: expected %ld bytes, got %zu", fileSize, readBytes);

    payload->size = readBytes;
    log_master("Loaded %zu bytes from %s\n", payload->size, plan->firmwarePath);
    return true;
}

/* Calculate the CRC-16 checksum used by the slave to validate transferred data. */
static uint16_t
fw_crc16(const uint8_t* data, size_t len) {
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

/* Push the metadata record (size, checksum, type, bank) into object 0x1F57:01.
 * Format: 8 bytes [size(4) | crc(2) | type(1) | bank(1)] little-endian */
static bool
send_metadata_to_slave(const fw_upload_plan_t* plan, const fw_payload_t* payload, uint16_t crc) {
    log_master("Sending metadata to slave node %u\n", plan->targetNodeId);
    log_master(" - image bytes : %zu\n", payload->size);
    log_master(" - crc         : 0x%04X\n", crc);
    log_master(" - image type  : %u\n", plan->type);
    log_master(" - bank        : %u\n", plan->targetBank);

    /* Metadata format: 8 bytes little-endian
     * [0-3] = size (4 bytes)
     * [4-5] = crc (2 bytes)
     * [6]   = type (1 byte)
     * [7]   = bank (1 byte)
     */
    uint8_t meta[8];
    uint32_t size = (uint32_t)payload->size;
    meta[0] = (uint8_t)(size & 0xFF);
    meta[1] = (uint8_t)((size >> 8) & 0xFF);
    meta[2] = (uint8_t)((size >> 16) & 0xFF);
    meta[3] = (uint8_t)((size >> 24) & 0xFF);
    meta[4] = (uint8_t)(crc & 0xFF);
    meta[5] = (uint8_t)((crc >> 8) & 0xFF);
    meta[6] = (uint8_t)plan->type;
    meta[7] = plan->targetBank;

    /* Replace with: sdo_download(plan->targetNodeId, 0x1F57, 1, meta, sizeof(meta)) */
    bool linkOk = true;  /* STUB - implement real SDO download */
    RETURN_IF_FALSE(linkOk, "Metadata write failed");
    return true;
}

/* Tell the slave to erase flash and enter download mode via object 0x1F51:01.
 * Format: 3 bytes {0x01, 0x00, 0x00} */
static bool
send_start_command(const fw_upload_plan_t* plan) {
    log_master("Issuing start command through object 0x1F51:01\n");
    
    uint8_t cmd[3] = {0x01, 0x00, 0x00};  /* Start token */
    /* Replace with: sdo_download(plan->targetNodeId, 0x1F51, 1, cmd, sizeof(cmd)) */
    bool linkOk = true;  /* STUB - implement real SDO download */
    RETURN_IF_FALSE(linkOk, "Control write failed");
    return true;
}

/* Transfer one data chunk to object 0x1F50:01.
 * 
 * IMPORTANT: SDO buffer is only 32 bytes by default! For CANopenNode, you must:
 *   1. Call CO_SDOclientDownloadInitiate() once
 *   2. In a loop: CO_SDOclientDownloadBufWrite() to fill buffer progressively
 *   3. Call CO_SDOclientDownload() repeatedly until complete
 * 
 * See ESP32 master's sdo_download() for working implementation. */
static bool
send_chunk_to_slave(const fw_upload_plan_t* plan, const uint8_t* chunk, size_t len, size_t offset) {
    log_debug("Sending chunk offset %zu size %zu\n", offset, len);
    /* Replace with: sdo_download(plan->targetNodeId, 0x1F50, 1, chunk, len) */
    bool linkOk = true;  /* STUB - implement real SDO download */
    RETURN_IF_FALSE(linkOk, "Chunk transfer failed");
    return true;
}

/* Request final verification via object 0x1F5A:01.
 * Format: 2-byte CRC little-endian */
static bool
send_finalize_request(const fw_upload_plan_t* plan, uint16_t crc) {
    log_master("Sending finalize request with crc 0x%04X\n", crc);
    
    uint8_t status[2] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF)};
    /* Replace with: sdo_download(plan->targetNodeId, 0x1F5A, 1, status, sizeof(status)) */
    bool linkOk = true;  /* STUB - implement real SDO download */
    RETURN_IF_FALSE(linkOk, "Finalize write failed");
    return true;
}

/* Query slave's running firmware CRC via SDO upload from 0x1F5B:01.
 * Returns 2-byte CRC in little-endian. */
static bool
query_slave_crc(const fw_upload_plan_t* plan, uint16_t* slaveCrc) {
    log_master("Querying slave CRC from node %u (0x1F5B:01)\n", plan->targetNodeId);
    
    /* Replace with: sdo_upload(plan->targetNodeId, 0x1F5B, 1, buf, sizeof(buf), &actualLen) */
    uint8_t buf[2] = {0};
    bool linkOk = false;  /* STUB - implement real SDO upload */
    
    if (!linkOk) {
        log_warn("Failed to query slave CRC (stub)\n");
        return false;
    }
    
    *slaveCrc = (uint16_t)(buf[0] | (buf[1] << 8));
    log_master("Slave running firmware CRC: 0x%04X\n", *slaveCrc);
    return true;
}

/* Iterate through the entire image, chunk by chunk, while keeping offsets aligned. */
static bool
fw_stream_payload(const fw_upload_plan_t* plan, const fw_payload_t* payload) {
    size_t offset = 0;
    while (offset < payload->size) {
        size_t remaining = payload->size - offset;
        size_t len = remaining < plan->maxChunkBytes ? remaining : plan->maxChunkBytes;
        if (!send_chunk_to_slave(plan, payload->buffer + offset, len, offset)) {
            return false;
        }
        offset += len;
        
        /* Progress every 10% */
        int prevPct = (offset - len) * 100 / payload->size;
        int currPct = offset * 100 / payload->size;
        if (currPct / 10 != prevPct / 10) {
            log_master("Upload progress: %zu/%zu bytes (%d%%)\n", offset, payload->size, currPct);
        }
    }
    return true;
}

/* High-level driver that loads the binary, computes CRC, and performs the full transaction.
 * Queries slave CRC first and skips upload if firmware already matches. */
static bool
fw_run_upload_session(const fw_upload_plan_t* plan) {
    fw_payload_t payload = {0};
    if (!fw_load_payload(plan, &payload)) {
        return false;
    }

    uint16_t crc = plan->expectedCrc;
    if (crc == 0U) {
        crc = fw_crc16(payload.buffer, payload.size);
        log_master("Computed CRC: 0x%04X\n", crc);
    }

    /* Query slave's current CRC - skip upload if matching */
    uint16_t slaveCrc = 0;
    if (query_slave_crc(plan, &slaveCrc)) {
        if (slaveCrc == crc) {
            log_master("Slave already has matching firmware (CRC 0x%04X), skipping upload\n", crc);
            free(payload.buffer);
            return true;
        }
        log_master("Slave CRC 0x%04X differs from local 0x%04X, proceeding with upload\n", slaveCrc, crc);
    } else {
        log_warn("Could not query slave CRC, proceeding with upload anyway\n");
    }

    bool ok = send_metadata_to_slave(plan, &payload, crc) && 
              send_start_command(plan) &&
              fw_stream_payload(plan, &payload) && 
              send_finalize_request(plan, crc);

    free(payload.buffer);
    
    if (ok) {
        log_master("Firmware upload completed successfully!\n");
        log_master("Slave will automatically reboot in ~500ms with new firmware.\n");
    }
    
    return ok;
}

/* Command-line entry point that prepares the upload plan and reports result codes. */
int
main(int argc, char** argv) {
    if (argc < 2) {
        printf("CANopen Master Firmware Uploader - Reference Implementation\n");
        printf("\n");
        printf("Usage: %s <firmware.bin> [nodeId] [bank] [maxChunkBytes]\n", argv[0]);
        printf("\n");
        printf("Arguments:\n");
        printf("  firmware.bin    Path to the firmware binary file\n");
        printf("  nodeId          Target slave node ID (default: 10)\n");
        printf("  bank            Target flash bank (default: 1)\n");
        printf("  maxChunkBytes   Max bytes per transfer (default: 256)\n");
        printf("\n");
        printf("Example:\n");
        printf("  %s firmware.bin 10 1 256\n", argv[0]);
        return -1;
    }

    fw_upload_plan_t plan = {
        .firmwarePath = argv[1],
        .type = FW_IMAGE_MAIN,
        .targetBank = argc > 3 ? (uint8_t)atoi(argv[3]) : 1U,
        .targetNodeId = argc > 2 ? (uint8_t)atoi(argv[2]) : 10U,
        .maxChunkBytes = argc > 4 ? (uint32_t)atoi(argv[4]) : 256U,
        .expectedCrc = 0U,
        .sdoTimeoutMs = 1000U
    };

    log_master("Upload plan:\n");
    log_master("  Firmware: %s\n", plan.firmwarePath);
    log_master("  Target node: %u\n", plan.targetNodeId);
    log_master("  Target bank: %u\n", plan.targetBank);
    log_master("  Max chunk: %u bytes\n", plan.maxChunkBytes);
    log_master("  SDO timeout: %u ms\n", plan.sdoTimeoutMs);

    if (!fw_run_upload_session(&plan)) {
        log_error("Firmware upload sequence failed\n");
        return -1;
    }

    return 0;
}
