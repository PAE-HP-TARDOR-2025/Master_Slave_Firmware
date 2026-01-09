/**
 * @file fw_master_update.c
 * @brief Firmware Update Logic for CANopen Master
 *
 * This file contains the core firmware update logic shared between
 * ESP32 and Raspberry Pi masters. It handles:
 * - Loading firmware from file
 * - Computing CRC-16 checksum
 * - Coordinating the upload session
 * - CRC comparison to skip unnecessary updates
 *
 * The actual SDO transport functions (fw_master_send_*, fw_master_query_*)
 * must be implemented by the platform-specific code.
 *
 * CRC-16 CCITT (X.25):
 * - Polynomial: 0x1021
 * - Initial value: 0xFFFF
 * - No final XOR
 * - Process MSB first
 */

#include "fw_master_update.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FW_MASTER_LOG
#define FW_MASTER_LOG(fmt, ...) printf("[FW-MASTER] " fmt, ##__VA_ARGS__)
#endif

#ifndef FW_MASTER_ERR
#define FW_MASTER_ERR(fmt, ...) fprintf(stderr, "[FW-ERROR ] " fmt, ##__VA_ARGS__)
#endif

#define RETURN_IF_FALSE(cond, msg, ...)   \
    do {                                  \
        if (!(cond)) {                    \
            FW_MASTER_ERR(msg "\n", ##__VA_ARGS__); \
            return false;                 \
        }                                 \
    } while (0)

/**
 * Load firmware payload from file
 */
bool fw_master_load_payload(const fw_upload_plan_t* plan, fw_payload_t* payload) {
    FILE* f = fopen(plan->firmwarePath, "rb");
    RETURN_IF_FALSE(f != NULL, "Cannot open firmware file %s", plan->firmwarePath);

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        FW_MASTER_ERR("Failed to seek to end of %s\n", plan->firmwarePath);
        return false;
    }

    long fileSize = ftell(f);
    RETURN_IF_FALSE(fileSize > 0, "Firmware file %s is empty", plan->firmwarePath);

    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        FW_MASTER_ERR("Failed to rewind file %s\n", plan->firmwarePath);
        return false;
    }

    payload->buffer = (uint8_t*)malloc((size_t)fileSize);
    RETURN_IF_FALSE(payload->buffer != NULL, "Out of memory while reading firmware");

    size_t readBytes = fread(payload->buffer, 1, (size_t)fileSize, f);
    fclose(f);
    RETURN_IF_FALSE(readBytes == (size_t)fileSize, "Short read: expected %ld bytes, got %zu", fileSize, readBytes);

    payload->size = readBytes;
    FW_MASTER_LOG("Loaded %zu bytes from %s\n", payload->size, plan->firmwarePath);
    return true;
}

/**
 * Compute CRC-16 CCITT checksum
 *
 * This MUST match the slave's CRC computation exactly:
 * - Polynomial: 0x1021
 * - Initial: 0xFFFF
 * - Process byte MSB first
 * - No final inversion
 */
uint16_t fw_master_crc16(const uint8_t* data, size_t len) {
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

/**
 * Stream firmware payload to slave in chunks
 */
bool fw_master_stream_payload(const fw_upload_plan_t* plan, const fw_payload_t* payload) {
    size_t offset = 0;
    size_t lastProgress = 0;
    
    FW_MASTER_LOG("Streaming %zu bytes in %zu-byte chunks\n", payload->size, plan->maxChunkBytes);
    
    while (offset < payload->size) {
        size_t remaining = payload->size - offset;
        size_t chunkLen = (remaining < plan->maxChunkBytes) ? remaining : plan->maxChunkBytes;
        
        if (!fw_master_send_chunk(plan, payload->buffer + offset, chunkLen, offset)) {
            FW_MASTER_ERR("Failed to send chunk at offset %zu\n", offset);
            return false;
        }
        
        offset += chunkLen;
        
        /* Progress indicator every 10% */
        size_t progress = (offset * 100) / payload->size;
        if (progress >= lastProgress + 10 || offset == payload->size) {
            FW_MASTER_LOG("Progress: %zu / %zu bytes (%zu%%)\n", offset, payload->size, progress);
            lastProgress = progress;
        }
    }
    
    FW_MASTER_LOG("Payload streaming complete\n");
    return true;
}

/**
 * Run a complete firmware upload session
 *
 * Sequence:
 * 1. Load firmware from file
 * 2. Compute CRC-16
 * 3. Send metadata (size, CRC, type, bank)
 * 4. Send start command
 * 5. Stream firmware data in chunks
 * 6. Send finalize request with CRC
 */
bool fw_master_run_upload_session(const fw_upload_plan_t* plan) {
    fw_payload_t payload = {0};
    
    if (!fw_master_load_payload(plan, &payload)) {
        return false;
    }

    uint16_t crc = plan->expectedCrc;
    if (crc == 0U) {
        crc = fw_master_crc16(payload.buffer, payload.size);
        FW_MASTER_LOG("Computed CRC: 0x%04X\n", crc);
    }

    FW_MASTER_LOG("Starting upload session to node %u\n", plan->targetNodeId);
    
    bool ok = fw_master_send_metadata(plan, &payload, crc);
    if (!ok) {
        FW_MASTER_ERR("Failed to send metadata\n");
        goto cleanup;
    }
    
    ok = fw_master_send_start_command(plan);
    if (!ok) {
        FW_MASTER_ERR("Failed to send start command\n");
        goto cleanup;
    }
    
    ok = fw_master_stream_payload(plan, &payload);
    if (!ok) {
        FW_MASTER_ERR("Failed to stream payload\n");
        goto cleanup;
    }
    
    ok = fw_master_send_finalize_request(plan, crc);
    if (!ok) {
        FW_MASTER_ERR("Failed to send finalize request\n");
        goto cleanup;
    }
    
    FW_MASTER_LOG("Upload session complete!\n");

cleanup:
    free(payload.buffer);
    return ok;
}

/**
 * Run upload only if slave firmware CRC differs
 *
 * This is the recommended entry point for firmware updates:
 * 1. Load firmware and compute CRC
 * 2. Query slave's running firmware CRC
 * 3. Compare CRCs - skip if match
 * 4. If different, run full upload session
 */
bool fw_master_run_upload_if_needed(const fw_upload_plan_t* plan) {
    fw_payload_t payload = {0};
    
    if (!fw_master_load_payload(plan, &payload)) {
        return false;
    }

    uint16_t localCrc = plan->expectedCrc;
    if (localCrc == 0U) {
        localCrc = fw_master_crc16(payload.buffer, payload.size);
        FW_MASTER_LOG("Local firmware CRC: 0x%04X\n", localCrc);
    }

    /* Query slave CRC */
    uint16_t slaveCrc = 0U;
    if (fw_master_query_slave_crc(plan, &slaveCrc)) {
        if (slaveCrc == localCrc) {
            FW_MASTER_LOG("Slave already running firmware with CRC 0x%04X - skipping upload\n", slaveCrc);
            free(payload.buffer);
            return true;  /* Success - no update needed */
        }
        FW_MASTER_LOG("CRC mismatch: slave=0x%04X, local=0x%04X - proceeding with upload\n", slaveCrc, localCrc);
    } else {
        FW_MASTER_LOG("Could not query slave CRC - proceeding with upload\n");
    }

    /* Run the upload */
    FW_MASTER_LOG("Starting upload session to node %u\n", plan->targetNodeId);
    
    bool ok = fw_master_send_metadata(plan, &payload, localCrc);
    if (!ok) {
        FW_MASTER_ERR("Failed to send metadata\n");
        goto cleanup;
    }
    
    ok = fw_master_send_start_command(plan);
    if (!ok) {
        FW_MASTER_ERR("Failed to send start command\n");
        goto cleanup;
    }
    
    ok = fw_master_stream_payload(plan, &payload);
    if (!ok) {
        FW_MASTER_ERR("Failed to stream payload\n");
        goto cleanup;
    }
    
    ok = fw_master_send_finalize_request(plan, localCrc);
    if (!ok) {
        FW_MASTER_ERR("Failed to send finalize request\n");
        goto cleanup;
    }
    
    FW_MASTER_LOG("Upload session complete!\n");

cleanup:
    free(payload.buffer);
    return ok;
}
