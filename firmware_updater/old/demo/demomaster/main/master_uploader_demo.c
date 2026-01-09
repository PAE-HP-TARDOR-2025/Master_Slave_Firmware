#include "master_uploader_demo.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CANopen.h"
#include "CO_SDOclient.h"

#define log_master(fmt, ...) printf("[FW-MASTER] " fmt, ##__VA_ARGS__)
#define log_error(fmt, ...)  printf("[FW-ERROR ] " fmt, ##__VA_ARGS__)
#define log_warn(fmt, ...)   printf("[FW-WARN  ] " fmt, ##__VA_ARGS__)

#define RETURN_IF_FALSE(cond, msg, ...)                                                                                \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            log_error(msg "\n", ##__VA_ARGS__);                                                                       \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

#define SDO_TIMEOUT_US 60000U
#define SDO_POLL_US     1000U

enum {
    FW_META_INDEX = 0x1F57,
    FW_CTRL_INDEX = 0x1F51,
    FW_DATA_INDEX = 0x1F50,
    FW_STATUS_INDEX = 0x1F5A
};

typedef struct {
    FILE *file;
    size_t size;
} fw_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t imageBytes;
    uint16_t crc;
    uint8_t imageType;
    uint8_t bank;
} fw_metadata_record_t;

enum {
    FW_CTRL_CMD_START = 0x01
};

static CO_SDOclient_t *s_sdo_client = NULL;
static uint8_t s_bound_node_id = 0U;

static bool fw_master_select_target(uint8_t nodeId);
static bool fw_sdo_download(uint16_t index, uint8_t subIndex, const uint8_t *data, size_t len, const char *label);

bool fw_master_bind_sdo_client(CO_SDOclient_t *client) {
    s_sdo_client = client;
    s_bound_node_id = 0U;
    return s_sdo_client != NULL;
}

static bool fw_master_select_target(uint8_t nodeId) {
    RETURN_IF_FALSE(s_sdo_client != NULL, "SDO client not available");
    if (s_bound_node_id == nodeId) {
        return true;
    }

    CO_SDO_return_t ret =
        CO_SDOclient_setup(s_sdo_client, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    RETURN_IF_FALSE(ret == CO_SDO_RT_ok_communicationEnd, "CO_SDOclient_setup failed (ret=%d)", ret);

    s_bound_node_id = nodeId;
    return true;
}

static bool fw_sdo_download(uint16_t index, uint8_t subIndex, const uint8_t *data, size_t len, const char *label) {
    RETURN_IF_FALSE(s_sdo_client != NULL, "SDO client not available");

    CO_SDO_return_t ret = CO_SDOclientDownloadInitiate(s_sdo_client, index, subIndex, len, SDO_TIMEOUT_US, false);
    RETURN_IF_FALSE(ret == CO_SDO_RT_ok_communicationEnd, "SDO init failed for %s (ret=%d)", label, ret);

    size_t totalWritten = 0U;
    bool_t bufferPartial = false;
    if (len > 0U) {
        size_t written = CO_SDOclientDownloadBufWrite(s_sdo_client, data, len);
        totalWritten = written;
        bufferPartial = totalWritten < len;
    }

    do {
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
        ret = CO_SDOclientDownload(s_sdo_client, SDO_POLL_US, false, bufferPartial, &abortCode, NULL, NULL);
        if (ret < 0) {
            log_error("SDO download for %s aborted (0x%08X)", label, abortCode);
            return false;
        }

        if (ret > 0) {
            if (bufferPartial && totalWritten < len) {
                size_t written = CO_SDOclientDownloadBufWrite(s_sdo_client, data + totalWritten, len - totalWritten);
                totalWritten += written;
                bufferPartial = totalWritten < len;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    } while (ret > 0);

    return true;
}

static bool fw_open_payload(const fw_upload_plan_t *plan, fw_payload_t *payload) {
    payload->file = fopen(plan->firmwarePath, "rb");
    RETURN_IF_FALSE(payload->file != NULL, "Cannot open firmware file %s", plan->firmwarePath);

    if (fseek(payload->file, 0, SEEK_END) != 0) {
        fclose(payload->file);
        payload->file = NULL;
        log_error("Failed to seek to end of %s\n", plan->firmwarePath);
        return false;
    }

    long fileSize = ftell(payload->file);
    RETURN_IF_FALSE(fileSize > 0, "Firmware file %s is empty", plan->firmwarePath);

    if (fseek(payload->file, 0, SEEK_SET) != 0) {
        fclose(payload->file);
        payload->file = NULL;
        log_error("Failed to rewind file %s\n", plan->firmwarePath);
        return false;
    }

    payload->size = (size_t)fileSize;
    log_master("Prepared %zu-byte firmware image from %s\n", payload->size, plan->firmwarePath);
    return true;
}

static void fw_close_payload(fw_payload_t *payload) {
    if (payload->file != NULL) {
        fclose(payload->file);
        payload->file = NULL;
    }
    payload->size = 0U;
}

static uint16_t fw_crc16_update(uint16_t crc, const uint8_t *data, size_t len) {
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

static bool fw_crc16_stream(FILE *file, size_t fileSize, uint8_t *scratch, size_t scratchLen, uint16_t *outCrc) {
    RETURN_IF_FALSE(file != NULL, "Firmware file handle is NULL");
    RETURN_IF_FALSE(scratch != NULL && scratchLen > 0U, "Scratch buffer not available");

    uint16_t crc = 0xFFFFU;
    size_t remaining = fileSize;
    while (remaining > 0U) {
        size_t chunk = remaining < scratchLen ? remaining : scratchLen;
        size_t read = fread(scratch, 1, chunk, file);
        RETURN_IF_FALSE(read == chunk, "Short read while computing CRC");
        crc = fw_crc16_update(crc, scratch, chunk);
        remaining -= chunk;
    }

    RETURN_IF_FALSE(fseek(file, 0, SEEK_SET) == 0, "Failed to rewind firmware after CRC pass");
    *outCrc = crc;
    return true;
}

static bool send_metadata_to_slave(const fw_upload_plan_t *plan, size_t imageBytes, uint16_t crc) {
    log_master("Sending metadata to slave node %u\n", plan->targetNodeId);
    log_master(" - image bytes : %zu\n", imageBytes);
    log_master(" - crc         : 0x%04X\n", crc);
    log_master(" - image type  : %u\n", plan->type);
    log_master(" - bank        : %u\n", plan->targetBank);
    RETURN_IF_FALSE(fw_master_select_target(plan->targetNodeId), "Unable to reach node %u", plan->targetNodeId);

    const fw_metadata_record_t meta = {
        .imageBytes = (uint32_t)imageBytes,
        .crc = crc,
        .imageType = (uint8_t)plan->type,
        .bank = plan->targetBank};

    return fw_sdo_download(FW_META_INDEX, 1U, (const uint8_t *)&meta, sizeof(meta), "metadata");
}

static bool send_start_command(const fw_upload_plan_t *plan) {
    log_master("Issuing start command through object 0x1F51\n");
    RETURN_IF_FALSE(fw_master_select_target(plan->targetNodeId), "Unable to reach node %u", plan->targetNodeId);

    const uint8_t controlPayload[3] = {FW_CTRL_CMD_START, (uint8_t)plan->type, plan->targetBank};
    return fw_sdo_download(FW_CTRL_INDEX, 1U, controlPayload, sizeof(controlPayload), "start command");
}

static bool send_chunk_to_slave(const fw_upload_plan_t *plan, const uint8_t *chunk, size_t len, size_t offset) {
    log_master("Sending chunk offset %zu size %zu\n", offset, len);
    RETURN_IF_FALSE(fw_master_select_target(plan->targetNodeId), "Unable to reach node %u", plan->targetNodeId);
    return fw_sdo_download(FW_DATA_INDEX, 1U, chunk, len, "chunk");
}

static bool send_finalize_request(const fw_upload_plan_t *plan, uint16_t crc) {
    log_master("Sending finalize request with crc 0x%04X\n", crc);
    RETURN_IF_FALSE(fw_master_select_target(plan->targetNodeId), "Unable to reach node %u", plan->targetNodeId);
    uint8_t crcBytes[2] = {(uint8_t)(crc & 0xFFU), (uint8_t)(crc >> 8)};
    return fw_sdo_download(FW_STATUS_INDEX, 1U, crcBytes, sizeof(crcBytes), "finalize request");
}

static bool fw_stream_payload(const fw_upload_plan_t *plan,
                              fw_payload_t *payload,
                              uint8_t *chunkBuffer,
                              size_t chunkCapacity) {
    RETURN_IF_FALSE(payload->file != NULL, "Firmware file handle is NULL");
    RETURN_IF_FALSE(chunkBuffer != NULL && chunkCapacity > 0U, "Chunk buffer missing");

    size_t offset = 0;
    while (offset < payload->size) {
        size_t remaining = payload->size - offset;
        size_t toRead = remaining < chunkCapacity ? remaining : chunkCapacity;
        size_t read = fread(chunkBuffer, 1, toRead, payload->file);
        RETURN_IF_FALSE(read == toRead, "Short read while streaming firmware");
        if (!send_chunk_to_slave(plan, chunkBuffer, read, offset)) {
            return false;
        }
        offset += read;
    }
    return true;
}

bool fw_run_upload_session(const fw_upload_plan_t *plan) {
    RETURN_IF_FALSE(plan != NULL, "Upload plan is NULL");
    RETURN_IF_FALSE(s_sdo_client != NULL, "CANopen transport not bound");
    RETURN_IF_FALSE(fw_master_select_target(plan->targetNodeId), "Failed to select node %u", plan->targetNodeId);

    fw_payload_t payload = {0};
    if (!fw_open_payload(plan, &payload)) {
        return false;
    }

    RETURN_IF_FALSE(plan->maxChunkBytes > 0U, "Chunk size must be greater than zero");
    uint8_t *chunkBuffer = (uint8_t *)malloc(plan->maxChunkBytes);
    if (chunkBuffer == NULL) {
        fw_close_payload(&payload);
        log_error("Out of memory while allocating chunk buffer (%" PRIu32 " bytes)", plan->maxChunkBytes);
        return false;
    }

    uint16_t crc = plan->expectedCrc;
    if (crc == 0U) {
        if (!fw_crc16_stream(payload.file, payload.size, chunkBuffer, plan->maxChunkBytes, &crc)) {
            free(chunkBuffer);
            fw_close_payload(&payload);
            return false;
        }
        log_master("Auto-computed crc: 0x%04X\n", crc);
    } else {
        log_master("Using provided crc: 0x%04X\n", crc);
    }

    bool ok = send_metadata_to_slave(plan, payload.size, crc) && send_start_command(plan) &&
              fw_stream_payload(plan, &payload, chunkBuffer, plan->maxChunkBytes) &&
              send_finalize_request(plan, crc);

    free(chunkBuffer);
    fw_close_payload(&payload);
    return ok;
}
