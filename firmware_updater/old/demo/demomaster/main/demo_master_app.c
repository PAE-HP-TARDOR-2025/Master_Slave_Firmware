#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "CANopen.h"
#include "OD.h"

#if CONFIG_DEMO_MASTER_USE_SPIFFS
#include "esp_spiffs.h"
#endif

#include "master_uploader_demo.h"

static const char* LOG_TAG = "demo_master";
static const char* CANOPEN_TAG = "canopen_master";

extern void CO_CANinterrupt(CO_CANmodule_t* CANmodule);

#define NMT_CONTROL                                                                                                    \
    (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_TIME        500U
#define SDO_SRV_TIMEOUT_TIME 1000U
#define SDO_CLI_TIMEOUT_TIME 1000U

typedef struct {
    CO_t* co;
    CO_SDOclient_t* sdoClient;
    TaskHandle_t processTask;
    TaskHandle_t rxTask;
    bool started;
} canopen_master_t;

static canopen_master_t g_canopen = {0};

static bool canopen_master_init(void);
static void canopen_process_task(void* arg);
static void canopen_rx_task(void* arg);
static void log_twai_status(const char* tag);

static void log_twai_status(const char* tag) {
    twai_status_info_t info = {0};
    if (twai_get_status_info(&info) == ESP_OK) {
        ESP_LOGI(tag,
                 "TWAI state=%d tx_err=%" PRIu32 " rx_err=%" PRIu32 " tx_q=%" PRIu32 " rx_q=%" PRIu32,
                 (int)info.state,
                 info.tx_error_counter,
                 info.rx_error_counter,
                 info.msgs_to_tx,
                 info.msgs_to_rx);
    } else {
        ESP_LOGE(tag, "Failed to query TWAI status");
    }
}

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

#if CONFIG_DEMO_MASTER_USE_SPIFFS
static void init_spiffs(void) {
    const esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_DEMO_MASTER_SPIFFS_BASE_PATH,
        .partition_label = CONFIG_DEMO_MASTER_SPIFFS_PARTITION_LABEL,
        .max_files = 4,
        .format_if_mount_failed = CONFIG_DEMO_MASTER_SPIFFS_FORMAT_IF_NEEDED};

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err == ESP_FAIL) {
        ESP_LOGE(LOG_TAG, "Failed to mount SPI flash file system");
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(LOG_TAG, "SPI flash partition not found");
    } else {
        ESP_ERROR_CHECK(err);
        size_t total = 0;
        size_t used = 0;
        ESP_ERROR_CHECK(esp_spiffs_info(conf.partition_label, &total, &used));
        ESP_LOGI(LOG_TAG, "SPIFFS: total=%u bytes used=%u bytes", (unsigned)total, (unsigned)used);
    }
}
#endif

static void canopen_process_task(void* arg) {
    canopen_master_t* ctx = (canopen_master_t*)arg;
    int64_t last = esp_timer_get_time();

    while (true) {
        if (ctx->co != NULL) {
            int64_t now = esp_timer_get_time();
            uint32_t diffUs = (uint32_t)(now - last);
            last = now;

            CO_NMT_reset_cmd_t reset = CO_process(ctx->co, false, diffUs, NULL);
            if (reset != CO_RESET_NOT) {
                ESP_LOGW(CANOPEN_TAG, "Requested CANopen reset (%d)", reset);
            }

#if (((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE) != 0) || (((CO_CONFIG_PDO) & (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE)) != 0)
            CO_LOCK_OD(ctx->co->CANmodule);
            bool_t syncWas = false;
#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE) != 0
            syncWas = CO_process_SYNC(ctx->co, diffUs, NULL);
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE) != 0
            CO_process_RPDO(ctx->co, syncWas, diffUs, NULL);
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE) != 0
            CO_process_TPDO(ctx->co, syncWas, diffUs, NULL);
#endif
            CO_UNLOCK_OD(ctx->co->CANmodule);
#endif
        }
        vTaskDelay(wait_ticks(1));
    }
}

static void canopen_rx_task(void* arg) {
    canopen_master_t* ctx = (canopen_master_t*)arg;
    while (true) {
        if (ctx->co != NULL && ctx->co->CANmodule != NULL && ctx->co->CANmodule->CANnormal) {
            CO_CANinterrupt(ctx->co->CANmodule);
        } else {
            vTaskDelay(wait_ticks(10));
        }
    }
}

static bool canopen_master_init(void) {
    if (g_canopen.started) {
        return true;
    }

    uint32_t heapBytes = 0U;
    g_canopen.co = CO_new(NULL, &heapBytes);
    if (g_canopen.co == NULL) {
        ESP_LOGE(CANOPEN_TAG, "Failed to allocate CANopen objects");
        return false;
    }
    ESP_LOGI(CANOPEN_TAG, "Reserved %lu bytes for CANopen", (unsigned long)heapBytes);

    void* CANptr = NULL;
    g_canopen.co->CANmodule->CANnormal = false;
    CO_CANsetConfigurationMode((void*)&CANptr);
    CO_CANmodule_disable(g_canopen.co->CANmodule);

    CO_ReturnError_t err = CO_CANinit(g_canopen.co, CANptr, (uint16_t)CONFIG_DEMO_MASTER_CAN_BITRATE_KBPS);
    if (err != CO_ERROR_NO) {
        ESP_LOGE(CANOPEN_TAG, "CO_CANinit failed (%d)", err);
        goto fail;
    }

    uint32_t errInfo = 0U;
    err = CO_CANopenInit(g_canopen.co, NULL, NULL, OD, NULL, NMT_CONTROL, FIRST_HB_TIME, SDO_SRV_TIMEOUT_TIME,
                         SDO_CLI_TIMEOUT_TIME, true, CONFIG_DEMO_MASTER_NODE_ID_SELF, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        ESP_LOGE(CANOPEN_TAG, "CO_CANopenInit failed (%d) info=0x%08lX", err, (unsigned long)errInfo);
        goto fail;
    }

#if ((CO_CONFIG_PDO) & (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE)) != 0
    err = CO_CANopenInitPDO(g_canopen.co, g_canopen.co->em, OD, CONFIG_DEMO_MASTER_NODE_ID_SELF, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        ESP_LOGE(CANOPEN_TAG, "CO_CANopenInitPDO failed (%d) info=0x%08lX", err, (unsigned long)errInfo);
        goto fail;
    }
#endif

    CO_CANsetNormalMode(g_canopen.co->CANmodule);
    log_twai_status(CANOPEN_TAG);

    g_canopen.sdoClient = g_canopen.co->SDOclient;
    if (g_canopen.sdoClient == NULL) {
        ESP_LOGE(CANOPEN_TAG, "SDO client unavailable");
        goto fail;
    }
    if (!fw_master_bind_sdo_client(g_canopen.sdoClient)) {
        ESP_LOGE(CANOPEN_TAG, "Failed to bind SDO client to uploader");
        goto fail;
    }

    if (xTaskCreate(canopen_process_task, "canopen_proc", 4096, &g_canopen, 5, &g_canopen.processTask) != pdPASS) {
        ESP_LOGE(CANOPEN_TAG, "Unable to create CANopen process task");
        goto fail;
    }
    if (xTaskCreate(canopen_rx_task, "canopen_rx", 4096, &g_canopen, 6, &g_canopen.rxTask) != pdPASS) {
        ESP_LOGE(CANOPEN_TAG, "Unable to create CANopen RX task");
        goto fail;
    }

    g_canopen.started = true;
    ESP_LOGI(CANOPEN_TAG, "CANopen master node %u running at %u kbps", CONFIG_DEMO_MASTER_NODE_ID_SELF,
             CONFIG_DEMO_MASTER_CAN_BITRATE_KBPS);
    return true;

fail:
    if (g_canopen.processTask != NULL) {
        vTaskDelete(g_canopen.processTask);
        g_canopen.processTask = NULL;
    }
    if (g_canopen.rxTask != NULL) {
        vTaskDelete(g_canopen.rxTask);
        g_canopen.rxTask = NULL;
    }
    if (g_canopen.co != NULL) {
        CO_CANsetConfigurationMode((void*)&CANptr);
        CO_delete(g_canopen.co);
        g_canopen.co = NULL;
    }
    return false;
}

void app_main(void) {
    init_nvs();
#if CONFIG_DEMO_MASTER_USE_SPIFFS
    init_spiffs();
#endif

    /* Give the developer a short window to attach the monitor after flashing. */
    ESP_LOGI(LOG_TAG, "Waiting 5 seconds before starting the upload demo...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(LOG_TAG, "Starting master firmware upload demo...");
    if (!canopen_master_init()) {
        ESP_LOGE(LOG_TAG, "CANopen master init failed; aborting demo");
        return;
    }

    fw_upload_plan_t plan = {
        .firmwarePath = CONFIG_DEMO_MASTER_FW_PATH,
        .type = FW_IMAGE_MAIN,
        .targetBank = CONFIG_DEMO_MASTER_TARGET_BANK,
        .targetNodeId = CONFIG_DEMO_MASTER_NODE_ID,
        .maxChunkBytes = CONFIG_DEMO_MASTER_CHUNK_BYTES,
        .expectedCrc = 0U};

    ESP_LOGI(LOG_TAG, "Starting master firmware upload demo using %s", plan.firmwarePath);

    if (!fw_run_upload_session(&plan)) {
        ESP_LOGE(LOG_TAG, "Firmware upload session failed. Check logs above for details.");
    } else {
        ESP_LOGI(LOG_TAG, "Firmware upload session completed. Reset the slave to boot the new image.");
    }

    while (true) {
        ESP_LOGI(LOG_TAG, "Master demo idle. Reboot to run another session.");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
