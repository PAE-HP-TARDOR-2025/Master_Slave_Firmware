#include "fw_master_update.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FW_MASTER_LOG
#define FW_MASTER_LOG(fmt, ...) printf("[FW-MASTER] " fmt, ##__VA_ARGS__)
#endif

#ifndef FW_MASTER_ERR
#define FW_MASTER_ERR(fmt, ...) printf("[FW-ERROR ] " fmt, ##__VA_ARGS__)
#endif

#define RETURN_IF_FALSE(cond, msg, ...)                                                              \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            FW_MASTER_ERR(msg "\n", ##__VA_ARGS__);                                                 \
            return false;                                                                            \
        }                                                                                            \
    } while (0)

bool
fw_master_load_payload(const fw_upload_plan_t* plan, fw_payload_t* payload) {
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

uint16_t
fw_master_crc16(const uint8_t* data, size_t len) {
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

__attribute__((weak)) bool
fw_master_send_metadata(const fw_upload_plan_t* plan, const fw_payload_t* payload, uint16_t crc) {
    FW_MASTER_LOG("Sending metadata to slave node %u\n", plan->targetNodeId);
    FW_MASTER_LOG(" - image bytes : %zu\n", payload->size);
    FW_MASTER_LOG(" - crc         : 0x%04X\n", crc);
    FW_MASTER_LOG(" - image type  : %u\n", plan->type);
    FW_MASTER_LOG(" - bank        : %u\n", plan->targetBank);
    FW_MASTER_LOG(" - version     : %u\n", plan->firmwareVersion);

    /* Replace this stub with real CO_SDOclientDownload* calls to index 0x1F57.
     * Metadata format: [size(4) | crc(2) | type(1) | bank(1) | version(2)] = 10 bytes */
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Metadata write failed (stub)");
    return true;
}

__attribute__((weak)) bool
fw_master_send_start_command(const fw_upload_plan_t* plan) {
    FW_MASTER_LOG("Issuing start command through object 0x1F51\n");
    /* Replace with real SDO write of the start token. */
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Control write failed (stub)");
    return true;
}

__attribute__((weak)) bool
fw_master_send_chunk(const fw_upload_plan_t* plan, const uint8_t* chunk, size_t len, size_t offset) {
    (void)plan;
    FW_MASTER_LOG("Sending chunk offset %zu size %zu\n", offset, len);
    /* Replace with block download segments (CO_SDOclientDownload). */
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Chunk transfer failed (stub)");
    return true;
}

__attribute__((weak)) bool
fw_master_send_finalize_request(const fw_upload_plan_t* plan, uint16_t crc) {
    (void)plan;
    FW_MASTER_LOG("Sending finalize request with crc 0x%04X\n", crc);
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Finalize write failed (stub)");
    return true;
}

bool
fw_master_stream_payload(const fw_upload_plan_t* plan, const fw_payload_t* payload) {
    size_t offset = 0;
    while (offset < payload->size) {
        size_t remaining = payload->size - offset;
        size_t len = remaining < plan->maxChunkBytes ? remaining : plan->maxChunkBytes;
        if (!fw_master_send_chunk(plan, payload->buffer + offset, len, offset)) {
            return false;
        }
        offset += len;
    }
    return true;
}

bool
fw_master_run_upload_session(const fw_upload_plan_t* plan) {
    fw_payload_t payload = {0};
    if (!fw_master_load_payload(plan, &payload)) {
        return false;
    }

    uint16_t crc = plan->expectedCrc;
    if (crc == 0U) {
        crc = fw_master_crc16(payload.buffer, payload.size);
        FW_MASTER_LOG("Auto-computed crc: 0x%04X\n", crc);
    }

    bool ok = fw_master_send_metadata(plan, &payload, crc) && fw_master_send_start_command(plan) &&
              fw_master_stream_payload(plan, &payload) && fw_master_send_finalize_request(plan, crc);

    free(payload.buffer);
    return ok;
}

__attribute__((weak)) bool
fw_master_query_slave_crc(const fw_upload_plan_t* plan, uint16_t* slaveCrc) {
    FW_MASTER_LOG("Querying running firmware CRC from slave node %u (0x1F5B:01)\n", plan->targetNodeId);
    /* TODO: Replace this stub with a real SDO upload from index 0x1F5B subindex 1.
     * On success, parse the 2-byte response into *slaveCrc and return true.
     * On failure (e.g., object does not exist or timeout), return false.
     */
    (void)slaveCrc;
    return false; /* stub always fails so upload proceeds */
}

__attribute__((weak)) bool
fw_master_query_slave_version(const fw_upload_plan_t* plan, uint16_t* slaveVersion) {
    FW_MASTER_LOG("Querying running firmware version from slave node %u (0x1F5C:01)\n", plan->targetNodeId);
    /* TODO: Replace this stub with a real SDO upload from index 0x1F5C subindex 1.
     * On success, parse the 2-byte response into *slaveVersion and return true.
     * On failure (e.g., object does not exist or timeout), return false.
     */
    (void)slaveVersion;
    return false; /* stub always fails so upload proceeds */
}

bool
fw_master_run_upload_if_needed(const fw_upload_plan_t* plan) {
    fw_payload_t payload = {0};
    if (!fw_master_load_payload(plan, &payload)) {
        return false;
    }

    uint16_t localCrc = plan->expectedCrc;
    if (localCrc == 0U) {
        localCrc = fw_master_crc16(payload.buffer, payload.size);
        FW_MASTER_LOG("Local firmware CRC: 0x%04X\n", localCrc);
    }

    uint16_t localVersion = plan->firmwareVersion;
    FW_MASTER_LOG("Local firmware version: %u\n", localVersion);

    /* Query slave CRC and version - upload only if BOTH match */
    uint16_t slaveCrc = 0U;
    uint16_t slaveVersion = 0U;
    bool crcQueried = fw_master_query_slave_crc(plan, &slaveCrc);
    bool verQueried = fw_master_query_slave_version(plan, &slaveVersion);

    if (crcQueried && verQueried) {
        FW_MASTER_LOG("Slave running: CRC=0x%04X, version=%u\n", slaveCrc, slaveVersion);
        if (slaveCrc == localCrc && slaveVersion == localVersion) {
            FW_MASTER_LOG("Slave firmware matches (CRC=0x%04X, ver=%u); skipping upload.\n", slaveCrc, slaveVersion);
            free(payload.buffer);
            return true;
        }
        if (slaveCrc == localCrc) {
            FW_MASTER_LOG("CRC matches but version differs (%u vs %u); uploading.\n", slaveVersion, localVersion);
        } else if (slaveVersion == localVersion) {
            FW_MASTER_LOG("Version matches but CRC differs (0x%04X vs 0x%04X); uploading.\n", slaveCrc, localCrc);
        } else {
            FW_MASTER_LOG("Both CRC and version differ; uploading.\n");
        }
    } else if (crcQueried) {
        FW_MASTER_LOG("Slave CRC=0x%04X (version query failed); proceeding with upload.\n", slaveCrc);
    } else if (verQueried) {
        FW_MASTER_LOG("Slave version=%u (CRC query failed); proceeding with upload.\n", slaveVersion);
    } else {
        FW_MASTER_LOG("Could not query slave CRC or version; proceeding with upload.\n");
    }

    bool ok = fw_master_send_metadata(plan, &payload, localCrc) && fw_master_send_start_command(plan) &&
              fw_master_stream_payload(plan, &payload) && fw_master_send_finalize_request(plan, localCrc);

    free(payload.buffer);
    return ok;
}
