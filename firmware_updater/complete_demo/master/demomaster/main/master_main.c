/**
 * ESP-IDF CANopen Master Firmware Uploader
 *
 * This application:
 *  - Initializes CANopenNode as a master
 *  - Mounts SPIFFS to access firmware binaries
 *  - Queries slave's running firmware CRC via SDO upload (0x1F5B:01)
 *  - If CRC differs, uploads new firmware via CiA-302 objects
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "CANopen.h"
#include "CO_SDOclient.h"
#include "OD.h"

#if CONFIG_MASTER_USE_SPIFFS
#include "esp_spiffs.h"
#endif

#include "fw_master_update.h"

static const char *TAG = "master_main";

/* CANopen interrupt handler (defined in CO_driver.c) */
extern void CO_CANinterrupt(CO_CANmodule_t *CANmodule);

#define NMT_CONTROL \
    (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_TIME        500U
#define SDO_SRV_TIMEOUT_TIME 1000U
#define SDO_CLI_TIMEOUT_TIME 3000U  /* Increased for block transfer */
#define SDO_CLI_BLOCK        true  /* Enable block transfer for faster OTA */

#ifndef CONFIG_MASTER_NUM_SLAVES
#define CONFIG_MASTER_NUM_SLAVES 1
#endif

#define MAX_SLAVES 8
#define PROGRESS_LOG_INTERVAL_MS 15000  /* Log progress every 15 seconds */

/* Per-slave upload context */
typedef struct {
    uint8_t nodeId;
    size_t totalBytes;
    size_t sentBytes;
    int64_t startTime;
    int64_t lastProgressLog;
    bool completed;
    bool failed;
    const char *errorMsg;
} slave_upload_ctx_t;

typedef struct {
    CO_t *co;
    CO_SDOclient_t *sdoClient;
    TaskHandle_t processTask;
    TaskHandle_t rxTask;
    TaskHandle_t uploaderTask;
    bool started;
    /* Multi-slave tracking */
    slave_upload_ctx_t slaveCtx[MAX_SLAVES];
    int numSlaves;
    SemaphoreHandle_t sdoMutex;  /* Mutex for SDO client access */
} master_ctx_t;

static master_ctx_t g_master = {0};

/* Forward declarations */
static bool master_canopen_init(void);
static void canopen_process_task(void *arg);
static void canopen_rx_task(void *arg);
static void uploader_task(void *arg);
static void progress_monitor_task(void *arg);

/* SDO client wrappers for fw_master_update */
static bool sdo_download(uint8_t nodeId, uint16_t index, uint8_t subIndex, const uint8_t *data, size_t len);
static bool sdo_upload(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t *data, size_t maxLen, size_t *actualLen);

static TickType_t wait_ticks(uint32_t ms) {
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return (ticks > 0) ? ticks : 1;
}

static void init_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

#if CONFIG_MASTER_USE_SPIFFS
static void init_spiffs(void) {
    const esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_MASTER_SPIFFS_BASE_PATH,
        .partition_label = CONFIG_MASTER_SPIFFS_PARTITION_LABEL,
        .max_files = 4,
        .format_if_mount_failed = false
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err == ESP_FAIL) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS");
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "SPIFFS partition not found");
    } else {
        ESP_ERROR_CHECK(err);
        size_t total = 0, used = 0;
        ESP_ERROR_CHECK(esp_spiffs_info(conf.partition_label, &total, &used));
        ESP_LOGI(TAG, "SPIFFS: total=%u used=%u bytes", (unsigned)total, (unsigned)used);
    }
}
#endif

/* ------------------------------------------------------------------------- */
/* SDO Client Wrappers                                                       */
/* ------------------------------------------------------------------------- */

static bool sdo_download(uint8_t nodeId, uint16_t index, uint8_t subIndex, const uint8_t *data, size_t len) {
    if (g_master.sdoClient == NULL) {
        ESP_LOGE(TAG, "SDO client not initialized");
        return false;
    }

    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeTransferred = 0;
    uint32_t timeoutMs = SDO_CLI_TIMEOUT_TIME;

    /* Setup SDO client for target node */
    CO_SDOclient_setup(g_master.sdoClient,
                       0x600 + nodeId,  /* COB-ID client->server */
                       0x580 + nodeId,  /* COB-ID server->client */
                       nodeId);

    CO_SDO_return_t ret = CO_SDOclientDownloadInitiate(g_master.sdoClient, index, subIndex, len, timeoutMs, SDO_CLI_BLOCK);
    if (ret < CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO download init failed: %d", ret);
        return false;
    }

    /* 
     * Write data progressively and process download in a loop.
     * SDO buffer is small (32 bytes by default), so we fill it as space becomes available.
     */
    size_t dataOffset = 0;
    bool bufferPartial = true;  /* Indicates more data to write */

    do {
        /* Fill buffer with as much data as it can accept */
        if (dataOffset < len) {
            size_t written = CO_SDOclientDownloadBufWrite(g_master.sdoClient, data + dataOffset, len - dataOffset);
            dataOffset += written;
            bufferPartial = (dataOffset < len);
        } else {
            bufferPartial = false;
        }

        ret = CO_SDOclientDownload(g_master.sdoClient, 1000, false, bufferPartial, &abortCode, &sizeTransferred, NULL);
        if (ret == CO_SDO_RT_waitingResponse) {
            vTaskDelay(wait_ticks(1));
        }
    } while (ret > CO_SDO_RT_ok_communicationEnd);

    if (ret != CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO download failed: ret=%d abort=0x%08lX", ret, (unsigned long)abortCode);
        return false;
    }

    return true;
}

static bool sdo_upload(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t *data, size_t maxLen, size_t *actualLen) {
    if (g_master.sdoClient == NULL) {
        ESP_LOGE(TAG, "SDO client not initialized");
        return false;
    }

    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeTransferred = 0;
    uint32_t timeoutMs = SDO_CLI_TIMEOUT_TIME;

    /* Setup SDO client for target node */
    CO_SDOclient_setup(g_master.sdoClient,
                       0x600 + nodeId,
                       0x580 + nodeId,
                       nodeId);

    CO_SDO_return_t ret = CO_SDOclientUploadInitiate(g_master.sdoClient, index, subIndex, timeoutMs, SDO_CLI_BLOCK);
    if (ret < CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO upload init failed: %d", ret);
        return false;
    }

    /* Process upload */
    size_t sizeIndicated = 0;
    do {
        ret = CO_SDOclientUpload(g_master.sdoClient, 1000, false, &abortCode, &sizeIndicated, &sizeTransferred, NULL);
        if (ret == CO_SDO_RT_waitingResponse) {
            vTaskDelay(wait_ticks(1));
        }
    } while (ret > CO_SDO_RT_ok_communicationEnd);

    if (ret != CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO upload failed: ret=%d abort=0x%08lX", ret, (unsigned long)abortCode);
        return false;
    }

    /* Read data from buffer */
    size_t dataSize = CO_SDOclientUploadBufRead(g_master.sdoClient, data, maxLen);
    if (actualLen != NULL) {
        *actualLen = dataSize;
    }

    return true;
}

/* ------------------------------------------------------------------------- */
/* FW Master Update Implementations                                          */
/* ------------------------------------------------------------------------- */

bool fw_master_send_metadata(const fw_upload_plan_t *plan, const fw_payload_t *payload, uint16_t crc) {
    ESP_LOGI(TAG, "Sending metadata: size=%zu crc=0x%04X type=%u bank=%u version=%u",
             payload->size, crc, plan->type, plan->targetBank, plan->firmwareVersion);

    /* Metadata format: [size(4) | crc(2) | type(1) | bank(1) | version(2)] = 10 bytes */
    uint8_t meta[10];
    uint32_t size = (uint32_t)payload->size;
    meta[0] = (uint8_t)(size & 0xFF);
    meta[1] = (uint8_t)((size >> 8) & 0xFF);
    meta[2] = (uint8_t)((size >> 16) & 0xFF);
    meta[3] = (uint8_t)((size >> 24) & 0xFF);
    meta[4] = (uint8_t)(crc & 0xFF);
    meta[5] = (uint8_t)((crc >> 8) & 0xFF);
    meta[6] = (uint8_t)plan->type;
    meta[7] = plan->targetBank;
    meta[8] = (uint8_t)(plan->firmwareVersion & 0xFF);
    meta[9] = (uint8_t)((plan->firmwareVersion >> 8) & 0xFF);

    return sdo_download(plan->targetNodeId, 0x1F57, 1, meta, sizeof(meta));
}

bool fw_master_send_start_command(const fw_upload_plan_t *plan) {
    ESP_LOGI(TAG, "Sending start command to node %u", plan->targetNodeId);
    uint8_t cmd[3] = {0x01, 0x00, 0x00}; /* Start token */
    return sdo_download(plan->targetNodeId, 0x1F51, 1, cmd, sizeof(cmd));
}

bool fw_master_send_chunk(const fw_upload_plan_t *plan, const uint8_t *chunk, size_t len, size_t offset) {
    ESP_LOGD(TAG, "Sending chunk: offset=%zu len=%zu", offset, len);
    return sdo_download(plan->targetNodeId, 0x1F50, 1, chunk, len);
}

bool fw_master_send_finalize_request(const fw_upload_plan_t *plan, uint16_t crc) {
    ESP_LOGI(TAG, "Sending finalize with CRC 0x%04X", crc);
    uint8_t status[2] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF)};
    return sdo_download(plan->targetNodeId, 0x1F5A, 1, status, sizeof(status));
}

bool fw_master_query_slave_crc(const fw_upload_plan_t *plan, uint16_t *slaveCrc) {
    ESP_LOGI(TAG, "Querying slave CRC from node %u (0x1F5B:01)", plan->targetNodeId);

    uint8_t buf[2] = {0};
    size_t actualLen = 0;

    if (!sdo_upload(plan->targetNodeId, 0x1F5B, 1, buf, sizeof(buf), &actualLen)) {
        ESP_LOGW(TAG, "Failed to query slave CRC");
        return false;
    }

    if (actualLen < 2) {
        ESP_LOGW(TAG, "Short response from slave CRC query: %zu bytes", actualLen);
        return false;
    }

    *slaveCrc = (uint16_t)(buf[0] | (buf[1] << 8));
    ESP_LOGI(TAG, "Slave running firmware CRC: 0x%04X", *slaveCrc);
    return true;
}

bool fw_master_query_slave_version(const fw_upload_plan_t *plan, uint16_t *slaveVersion) {
    ESP_LOGI(TAG, "Querying slave version from node %u (0x1F5C:01)", plan->targetNodeId);

    uint8_t buf[2] = {0};
    size_t actualLen = 0;

    if (!sdo_upload(plan->targetNodeId, 0x1F5C, 1, buf, sizeof(buf), &actualLen)) {
        ESP_LOGW(TAG, "Failed to query slave version");
        return false;
    }

    if (actualLen < 2) {
        ESP_LOGW(TAG, "Short response from slave version query: %zu bytes", actualLen);
        return false;
    }

    *slaveVersion = (uint16_t)(buf[0] | (buf[1] << 8));
    ESP_LOGI(TAG, "Slave running firmware version: %u", *slaveVersion);
    return true;
}

/* ------------------------------------------------------------------------- */
/* CANopen Tasks                                                             */
/* ------------------------------------------------------------------------- */

static void canopen_process_task(void *arg) {
    master_ctx_t *ctx = (master_ctx_t *)arg;
    int64_t last = esp_timer_get_time();

    while (true) {
        if (ctx->co != NULL) {
            int64_t now = esp_timer_get_time();
            uint32_t diffUs = (uint32_t)(now - last);
            last = now;

            CO_NMT_reset_cmd_t reset = CO_process(ctx->co, false, diffUs, NULL);
            if (reset != CO_RESET_NOT) {
                ESP_LOGW(TAG, "CANopen reset requested: %d", reset);
            }
        }
        vTaskDelay(wait_ticks(1));
    }
}

static void canopen_rx_task(void *arg) {
    master_ctx_t *ctx = (master_ctx_t *)arg;

    while (true) {
        if (ctx->co != NULL && ctx->co->CANmodule != NULL && ctx->co->CANmodule->CANnormal) {
            CO_CANinterrupt(ctx->co->CANmodule);
        } else {
            vTaskDelay(wait_ticks(10));
        }
    }
}

/* Progress monitor task - logs progress every 15 seconds */
static void progress_monitor_task(void *arg) {
    (void)arg;
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(PROGRESS_LOG_INTERVAL_MS));
        
        bool allDone = true;
        int64_t now = esp_timer_get_time();
        
        for (int i = 0; i < g_master.numSlaves; i++) {
            slave_upload_ctx_t *ctx = &g_master.slaveCtx[i];
            if (!ctx->completed && !ctx->failed) {
                allDone = false;
                int64_t elapsedMs = (now - ctx->startTime) / 1000;
                int percent = (ctx->totalBytes > 0) ? (int)(ctx->sentBytes * 100 / ctx->totalBytes) : 0;
                float rate = (elapsedMs > 0) ? (ctx->sentBytes * 1000.0f / elapsedMs) : 0;
                ESP_LOGW(TAG, "[Node %u] Progress: %zu/%zu bytes (%d%%) - %.1f KB/s - %lld sec elapsed",
                         ctx->nodeId, ctx->sentBytes, ctx->totalBytes, percent, rate / 1024.0f, elapsedMs / 1000);
            }
        }
        
        if (allDone) {
            ESP_LOGW(TAG, "All slave updates finished, stopping progress monitor");
            vTaskDelete(NULL);
            return;
        }
    }
}

/* Per-slave upload task arguments */
typedef struct {
    int slaveIndex;
    const char *firmwarePath;
    size_t fileSize;
    uint16_t crc;
    uint16_t firmwareVersion;
    uint32_t maxChunkBytes;
} slave_upload_args_t;

/* Per-slave upload task */
static void slave_upload_task(void *arg) {
    slave_upload_args_t *args = (slave_upload_args_t *)arg;
    slave_upload_ctx_t *ctx = &g_master.slaveCtx[args->slaveIndex];
    
    ctx->startTime = esp_timer_get_time();
    ctx->lastProgressLog = ctx->startTime;
    ctx->totalBytes = args->fileSize;
    ctx->sentBytes = 0;
    ctx->completed = false;
    ctx->failed = false;
    
    fw_upload_plan_t plan = {
        .firmwarePath = args->firmwarePath,
        .type = FW_IMAGE_MAIN,
        .targetBank = 1,
        .targetNodeId = ctx->nodeId,
        .maxChunkBytes = args->maxChunkBytes,
        .expectedCrc = args->crc,
        .firmwareVersion = args->firmwareVersion
    };
    
    ESP_LOGW(TAG, "[Node %u] Starting upload: %zu bytes, version %u", 
             ctx->nodeId, ctx->totalBytes, plan.firmwareVersion);

#if CONFIG_MASTER_SKIP_IF_CRC_MATCH
    /* Check if slave already has this firmware - need mutex for SDO access */
    xSemaphoreTake(g_master.sdoMutex, portMAX_DELAY);
    uint16_t slaveCrc = 0;
    uint16_t slaveVer = 0;
    bool crcQueried = fw_master_query_slave_crc(&plan, &slaveCrc);
    bool verQueried = fw_master_query_slave_version(&plan, &slaveVer);
    xSemaphoreGive(g_master.sdoMutex);

    if (crcQueried && verQueried) {
        if (slaveCrc == args->crc && slaveVer == plan.firmwareVersion) {
            ESP_LOGW(TAG, "[Node %u] Already has matching firmware (CRC=0x%04X, ver=%u), skipping", 
                     ctx->nodeId, slaveCrc, slaveVer);
            ctx->completed = true;
            ctx->sentBytes = ctx->totalBytes;
            free(args);
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGW(TAG, "[Node %u] Firmware differs: CRC=0x%04X->0x%04X, ver=%u->%u, proceeding", 
                 ctx->nodeId, slaveCrc, args->crc, slaveVer, plan.firmwareVersion);
    }
#endif

    /* Open firmware file for this slave's upload */
    FILE *f = fopen(args->firmwarePath, "rb");
    if (f == NULL) {
        ctx->failed = true;
        ctx->errorMsg = "Cannot open file";
        ESP_LOGE(TAG, "[Node %u] Cannot open firmware file", ctx->nodeId);
        free(args);
        vTaskDelete(NULL);
        return;
    }

    /* Create pseudo-payload for metadata */
    fw_payload_t payload = { .buffer = NULL, .size = ctx->totalBytes };

    /* Send metadata - need mutex for SDO */
    xSemaphoreTake(g_master.sdoMutex, portMAX_DELAY);
    bool metaOk = fw_master_send_metadata(&plan, &payload, args->crc);
    xSemaphoreGive(g_master.sdoMutex);
    
    if (!metaOk) {
        ctx->failed = true;
        ctx->errorMsg = "Metadata failed";
        ESP_LOGE(TAG, "[Node %u] Failed to send metadata", ctx->nodeId);
        fclose(f);
        free(args);
        vTaskDelete(NULL);
        return;
    }

    /* Send start command */
    xSemaphoreTake(g_master.sdoMutex, portMAX_DELAY);
    bool startOk = fw_master_send_start_command(&plan);
    xSemaphoreGive(g_master.sdoMutex);
    
    if (!startOk) {
        ctx->failed = true;
        ctx->errorMsg = "Start cmd failed";
        ESP_LOGE(TAG, "[Node %u] Failed to send start command", ctx->nodeId);
        fclose(f);
        free(args);
        vTaskDelete(NULL);
        return;
    }

    /* Stream firmware data */
    uint8_t *chunk = malloc(args->maxChunkBytes);
    if (chunk == NULL) {
        ctx->failed = true;
        ctx->errorMsg = "OOM";
        ESP_LOGE(TAG, "[Node %u] Failed to allocate chunk buffer", ctx->nodeId);
        fclose(f);
        free(args);
        vTaskDelete(NULL);
        return;
    }

    size_t bytesRead;
    size_t offset = 0;

    while ((bytesRead = fread(chunk, 1, args->maxChunkBytes, f)) > 0) {
        xSemaphoreTake(g_master.sdoMutex, portMAX_DELAY);
        bool chunkOk = fw_master_send_chunk(&plan, chunk, bytesRead, offset);
        xSemaphoreGive(g_master.sdoMutex);
        
        if (!chunkOk) {
            ctx->failed = true;
            ctx->errorMsg = "Chunk failed";
            ESP_LOGE(TAG, "[Node %u] Failed to send chunk at offset %zu", ctx->nodeId, offset);
            free(chunk);
            fclose(f);
            free(args);
            vTaskDelete(NULL);
            return;
        }
        
        offset += bytesRead;
        ctx->sentBytes = offset;
        
        /* Yield to allow other slaves to upload (round-robin fairness) */
        taskYIELD();
    }

    /* Send finalize */
    xSemaphoreTake(g_master.sdoMutex, portMAX_DELAY);
    bool finalizeOk = fw_master_send_finalize_request(&plan, args->crc);
    xSemaphoreGive(g_master.sdoMutex);
    
    if (!finalizeOk) {
        ctx->failed = true;
        ctx->errorMsg = "Finalize failed";
        ESP_LOGE(TAG, "[Node %u] Failed to send finalize", ctx->nodeId);
        free(chunk);
        fclose(f);
        free(args);
        vTaskDelete(NULL);
        return;
    }

    int64_t endTime = esp_timer_get_time();
    int64_t elapsedMs = (endTime - ctx->startTime) / 1000;
    float rate = (elapsedMs > 0) ? (ctx->sentBytes * 1000.0f / elapsedMs) : 0;
    
    ESP_LOGW(TAG, "[Node %u] Upload completed! %zu bytes in %lld.%03lld sec (%.1f KB/s)", 
             ctx->nodeId, ctx->sentBytes, elapsedMs / 1000, elapsedMs % 1000, rate / 1024.0f);
    
    ctx->completed = true;
    free(chunk);
    fclose(f);
    free(args);
    vTaskDelete(NULL);
}

static void uploader_task(void *arg) {
    (void)arg;

    /* Wait for CANopen to be ready */
    vTaskDelay(wait_ticks(2000));

    g_master.numSlaves = CONFIG_MASTER_NUM_SLAVES;
    if (g_master.numSlaves > MAX_SLAVES) {
        g_master.numSlaves = MAX_SLAVES;
    }

    ESP_LOGW(TAG, "Starting multi-slave firmware upload: %d slaves (nodes %u-%u)", 
             g_master.numSlaves, 
             CONFIG_MASTER_TARGET_NODE_ID, 
             CONFIG_MASTER_TARGET_NODE_ID + g_master.numSlaves - 1);
    
    int64_t startTime = esp_timer_get_time();

    /* Open firmware file to compute CRC and get size */
    FILE *f = fopen(CONFIG_MASTER_FIRMWARE_PATH, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Cannot open firmware file: %s", CONFIG_MASTER_FIRMWARE_PATH);
        vTaskDelete(NULL);
        return;
    }

    /* Get file size */
    fseek(f, 0, SEEK_END);
    long fileSize = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    if (fileSize <= 0) {
        ESP_LOGE(TAG, "Invalid firmware file size");
        fclose(f);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Firmware file: %s (%ld bytes)", CONFIG_MASTER_FIRMWARE_PATH, fileSize);

    /* Compute CRC by streaming through file */
    uint16_t crc = 0xFFFFU;
    uint8_t *buf = malloc(CONFIG_MASTER_MAX_CHUNK_BYTES);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        fclose(f);
        vTaskDelete(NULL);
        return;
    }

    size_t bytesRead;
    while ((bytesRead = fread(buf, 1, CONFIG_MASTER_MAX_CHUNK_BYTES, f)) > 0) {
        for (size_t i = 0; i < bytesRead; i++) {
            crc ^= (uint16_t)buf[i] << 8;
            for (int bit = 0; bit < 8; bit++) {
                if (crc & 0x8000U) {
                    crc = (uint16_t)((crc << 1) ^ 0x1021U);
                } else {
                    crc <<= 1;
                }
            }
        }
    }
    free(buf);
    fclose(f);
    
    ESP_LOGI(TAG, "Firmware CRC: 0x%04X, version: %u", crc, CONFIG_MASTER_FIRMWARE_VERSION);

    /* Initialize slave contexts */
    for (int i = 0; i < g_master.numSlaves; i++) {
        g_master.slaveCtx[i].nodeId = CONFIG_MASTER_TARGET_NODE_ID + i;
        g_master.slaveCtx[i].totalBytes = (size_t)fileSize;
        g_master.slaveCtx[i].sentBytes = 0;
        g_master.slaveCtx[i].completed = false;
        g_master.slaveCtx[i].failed = false;
        g_master.slaveCtx[i].errorMsg = NULL;
    }

    /* Start progress monitor task */
    xTaskCreate(progress_monitor_task, "progress", 4096, NULL, 2, NULL);

    /* Spawn per-slave upload tasks */
    for (int i = 0; i < g_master.numSlaves; i++) {
        slave_upload_args_t *args = malloc(sizeof(slave_upload_args_t));
        if (args == NULL) {
            ESP_LOGE(TAG, "Failed to allocate args for slave %d", i);
            continue;
        }
        args->slaveIndex = i;
        args->firmwarePath = CONFIG_MASTER_FIRMWARE_PATH;
        args->fileSize = (size_t)fileSize;
        args->crc = crc;
        args->firmwareVersion = CONFIG_MASTER_FIRMWARE_VERSION;
        args->maxChunkBytes = CONFIG_MASTER_MAX_CHUNK_BYTES;

        char taskName[16];
        snprintf(taskName, sizeof(taskName), "upload_%u", g_master.slaveCtx[i].nodeId);
        xTaskCreate(slave_upload_task, taskName, 6144, args, 3, NULL);
        
        /* Stagger task starts slightly to reduce initial contention */
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Wait for all slaves to complete */
    bool allDone = false;
    while (!allDone) {
        vTaskDelay(pdMS_TO_TICKS(500));
        allDone = true;
        for (int i = 0; i < g_master.numSlaves; i++) {
            if (!g_master.slaveCtx[i].completed && !g_master.slaveCtx[i].failed) {
                allDone = false;
                break;
            }
        }
    }

    /* Summary report */
    int64_t endTime = esp_timer_get_time();
    int64_t elapsedMs = (endTime - startTime) / 1000;
    
    int successCount = 0;
    int failCount = 0;
    size_t totalBytes = 0;
    
    ESP_LOGW(TAG, "=== Multi-slave upload summary ===");
    for (int i = 0; i < g_master.numSlaves; i++) {
        slave_upload_ctx_t *ctx = &g_master.slaveCtx[i];
        if (ctx->completed) {
            successCount++;
            totalBytes += ctx->sentBytes;
            ESP_LOGW(TAG, "  Node %u: SUCCESS (%zu bytes)", ctx->nodeId, ctx->sentBytes);
        } else if (ctx->failed) {
            failCount++;
            ESP_LOGW(TAG, "  Node %u: FAILED (%s)", ctx->nodeId, ctx->errorMsg ? ctx->errorMsg : "unknown");
        }
    }
    
    float rate = (elapsedMs > 0) ? (totalBytes * 1000.0f / elapsedMs) : 0;
    ESP_LOGW(TAG, "Completed: %d success, %d failed, %zu bytes in %lld.%03lld sec (%.1f KB/s aggregate)",
             successCount, failCount, totalBytes, elapsedMs / 1000, elapsedMs % 1000, rate / 1024.0f);

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------------- */
/* CANopen Initialization                                                    */
/* ------------------------------------------------------------------------- */

static bool master_canopen_init(void) {
    if (g_master.started) {
        return true;
    }

    uint32_t heapBytes = 0;

    /* Allocate CANopen object (pass NULL when CO_MULTIPLE_OD is not defined) */
    g_master.co = CO_new(NULL, &heapBytes);
    if (g_master.co == NULL) {
        ESP_LOGE(TAG, "CO_new failed");
        return false;
    }
    ESP_LOGI(TAG, "CANopen allocated %u bytes", (unsigned)heapBytes);

    /* Initialize CANopen - CO_CANinit will install the TWAI driver */
    CO_ReturnError_t err;

#ifdef CONFIG_MASTER_CAN_BITRATE_KBPS
    err = CO_CANinit(g_master.co, NULL, CONFIG_MASTER_CAN_BITRATE_KBPS);
#else
    err = CO_CANinit(g_master.co, NULL, 500);  /* Default 500 kbps */
#endif
    if (err != CO_ERROR_NO) {
        ESP_LOGE(TAG, "CO_CANinit failed: %d", err);
        return false;
    }

    err = CO_CANopenInit(g_master.co,
                         NULL, NULL, OD,
                         NULL, NMT_CONTROL,
                         FIRST_HB_TIME,
                         SDO_SRV_TIMEOUT_TIME,
                         SDO_CLI_TIMEOUT_TIME,
                         SDO_CLI_BLOCK,
                         CONFIG_MASTER_NODE_ID,
                         NULL);
    if (err != CO_ERROR_NO) {
        ESP_LOGE(TAG, "CO_CANopenInit failed: %d", err);
        return false;
    }

    /* Get SDO client reference */
    g_master.sdoClient = g_master.co->SDOclient;
    if (g_master.sdoClient == NULL) {
        ESP_LOGE(TAG, "SDO client not available");
        return false;
    }

    /* Create mutex for SDO client access (shared across all slave upload tasks) */
    g_master.sdoMutex = xSemaphoreCreateMutex();
    if (g_master.sdoMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create SDO mutex");
        return false;
    }

    /* Start CANopen */
    CO_CANsetNormalMode(g_master.co->CANmodule);
    ESP_LOGI(TAG, "CANopen started, node ID %u", CONFIG_MASTER_NODE_ID);

    g_master.started = true;
    return true;
}

/* ------------------------------------------------------------------------- */
/* Main Entry Point                                                          */
/* ------------------------------------------------------------------------- */

void app_main(void) {
    ESP_LOGI(TAG, "Master Firmware Uploader starting...");

    init_nvs();

#if CONFIG_MASTER_USE_SPIFFS
    init_spiffs();
#endif

    if (!master_canopen_init()) {
        ESP_LOGE(TAG, "Failed to initialize CANopen");
        return;
    }

    /* Start CANopen tasks */
    xTaskCreate(canopen_process_task, "co_process", 4096, &g_master, 5, &g_master.processTask);
    xTaskCreate(canopen_rx_task, "co_rx", 4096, &g_master, 10, &g_master.rxTask);

#if CONFIG_MASTER_UPLOAD_ON_STARTUP
    xTaskCreate(uploader_task, "uploader", 8192, &g_master, 3, &g_master.uploaderTask);
#endif

    ESP_LOGI(TAG, "Master running");
}
