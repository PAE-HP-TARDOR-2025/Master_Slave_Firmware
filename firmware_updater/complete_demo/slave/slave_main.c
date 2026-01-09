/*
 * Template ESP-IDF CANopen slave main with firmware-update server integration.
 *
 * Features:
 *  - Initializes NVS, CANopen stack and firmware-download OD extensions.
 *  - Computes and exposes the running firmware CRC (object 0x1F5B if present).
 *  - Prints a periodic greeting that includes the running CRC — useful for
 *    verifying a successful OTA by observing changed output.
 *
 * Customization:
 *  - Define SLAVE_GREETING at compile time (e.g. via -DSLAVE_GREETING='"Hello"')
 *    to change the greeting printed to the serial monitor.
 *  - Adjust Kconfig options for node ID, bitrate and buffer sizes.
 *
 * Build with ESP-IDF:
 *   idf.py -C path/to/project build
 *   idf.py -p COMx flash monitor
 */

#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "CANopen.h"
#include "OD.h"

#include "fw_update_server.h"

/* External interrupt handler provided by CANopenNode ESP32 port. */
extern void CO_CANinterrupt(CO_CANmodule_t *CANmodule);

#ifndef SLAVE_GREETING
#define SLAVE_GREETING "Hello from slave"
#endif

/* NMT startup behaviour flags. */
#define NMT_CONTROL                                                            \
    (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG |                   \
     CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

#define FIRST_HB_TIME        500U
#define SDO_SRV_TIMEOUT_TIME 1000U
#define SDO_CLI_TIMEOUT_TIME 1000U

/* Optional marker string — some tooling can search for it inside the binary. */
static const char greeting_storage[] = "GREETING:" SLAVE_GREETING;

/* -------------------------------------------------------------------------- */
/*                             Runtime context                                 */
/* -------------------------------------------------------------------------- */

typedef struct {
    CO_t *co;
    TaskHandle_t processTask;
    TaskHandle_t rxTask;
    TaskHandle_t greetingTask;
    bool started;
} canopen_slave_t;

static const char *LOG_TAG = "slave_main";
static const char *CANOPEN_TAG = "canopen";
static canopen_slave_t g_canopen = {0};

/* -------------------------------------------------------------------------- */
/*                              Helpers                                        */
/* -------------------------------------------------------------------------- */

static void log_twai_status(const char *tag) {
    twai_status_info_t info = {0};
    if (twai_get_status_info(&info) == ESP_OK) {
        ESP_LOGI(tag,
                 "TWAI state=%u tx_err=%" PRIu32 " rx_err=%" PRIu32 " tx_q=%" PRIu32 " rx_q=%" PRIu32,
                 info.state,
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
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

/* -------------------------------------------------------------------------- */
/*                          FreeRTOS tasks                                     */
/* -------------------------------------------------------------------------- */

static void greeting_task(void *arg) {
    (void)arg;
    (void)greeting_storage; /* keep marker string in binary */
    while (true) {
        uint16_t crc = fw_server_get_running_crc();
        uint16_t ver = fw_server_get_running_version();
        printf("[SLAVE] %s (CRC: 0x%04X, ver: %u)\n", SLAVE_GREETING, (unsigned)crc, (unsigned)ver);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void canopen_process_task(void *arg) {
    canopen_slave_t *ctx = (canopen_slave_t *)arg;
    int64_t last = esp_timer_get_time();
    while (true) {
        if (ctx->co != NULL) {
            int64_t now = esp_timer_get_time();
            uint32_t diffUs = (uint32_t)(now - last);
            last = now;
            CO_process(ctx->co, false, diffUs, NULL);
        }
        vTaskDelay(wait_ticks(1));
    }
}

static void canopen_rx_task(void *arg) {
    canopen_slave_t *ctx = (canopen_slave_t *)arg;
    while (true) {
        if (ctx->co != NULL && ctx->co->CANmodule != NULL &&
            ctx->co->CANmodule->CANnormal) {
            CO_CANinterrupt(ctx->co->CANmodule);
        } else {
            vTaskDelay(wait_ticks(10));
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                        CANopen init / cleanup                               */
/* -------------------------------------------------------------------------- */

static bool canopen_slave_init(void) {
    if (g_canopen.started) {
        return true;
    }

    uint32_t heapBytes = 0U;
    g_canopen.co = CO_new(NULL, &heapBytes);
    if (g_canopen.co == NULL) {
        ESP_LOGE(CANOPEN_TAG, "Failed to allocate CANopen objects");
        return false;
    }
    ESP_LOGI(CANOPEN_TAG, "Reserved %lu bytes for CANopen",
             (unsigned long)heapBytes);

    void *CANptr = NULL;
    g_canopen.co->CANmodule->CANnormal = false;
    CO_CANsetConfigurationMode((void *)&CANptr);
    CO_CANmodule_disable(g_canopen.co->CANmodule);

    CO_ReturnError_t err =
        CO_CANinit(g_canopen.co, CANptr,
                   (uint16_t)CONFIG_DEMO_SLAVE_CAN_BITRATE_KBPS);
    if (err != CO_ERROR_NO) {
        ESP_LOGE(CANOPEN_TAG, "CO_CANinit failed (%d)", err);
        goto fail;
    }

    uint32_t errInfo = 0U;
    err = CO_CANopenInit(g_canopen.co, NULL, NULL, OD, NULL, NMT_CONTROL,
                         FIRST_HB_TIME, SDO_SRV_TIMEOUT_TIME,
                         SDO_CLI_TIMEOUT_TIME, true,
                         CONFIG_DEMO_SLAVE_NODE_ID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        ESP_LOGE(CANOPEN_TAG,
                 "CO_CANopenInit failed (%d) info=0x%08lX", err,
                 (unsigned long)errInfo);
        goto fail;
    }

#if ((CO_CONFIG_PDO) & (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE)) != 0
    err = CO_CANopenInitPDO(g_canopen.co, g_canopen.co->em, OD,
                            CONFIG_DEMO_SLAVE_NODE_ID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        ESP_LOGE(CANOPEN_TAG,
                 "CO_CANopenInitPDO failed (%d) info=0x%08lX", err,
                 (unsigned long)errInfo);
        goto fail;
    }
#endif

    if (!fw_server_init(g_canopen.co)) {
        ESP_LOGE(CANOPEN_TAG, "Failed to bind firmware update server");
        goto fail;
    }

    CO_CANsetNormalMode(g_canopen.co->CANmodule);
    log_twai_status(CANOPEN_TAG);

    if (xTaskCreate(canopen_process_task, "co_proc", 4096, &g_canopen, 5,
                    &g_canopen.processTask) != pdPASS) {
        ESP_LOGE(CANOPEN_TAG, "Unable to create CANopen process task");
        goto fail;
    }
    if (xTaskCreate(canopen_rx_task, "co_rx", 4096, &g_canopen, 6,
                    &g_canopen.rxTask) != pdPASS) {
        ESP_LOGE(CANOPEN_TAG, "Unable to create CANopen RX task");
        goto fail;
    }
    if (xTaskCreate(greeting_task, "greet", 2048, NULL, 3,
                    &g_canopen.greetingTask) != pdPASS) {
        ESP_LOGE(LOG_TAG, "Unable to create greeting task");
        goto fail;
    }

    g_canopen.started = true;
    ESP_LOGI(CANOPEN_TAG, "CANopen slave node %u ready at %u kbps",
             CONFIG_DEMO_SLAVE_NODE_ID, CONFIG_DEMO_SLAVE_CAN_BITRATE_KBPS);
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
    if (g_canopen.greetingTask != NULL) {
        vTaskDelete(g_canopen.greetingTask);
        g_canopen.greetingTask = NULL;
    }
    if (g_canopen.co != NULL) {
        CO_CANsetConfigurationMode((void *)&CANptr);
        CO_delete(g_canopen.co);
        g_canopen.co = NULL;
    }
    return false;
}

/* -------------------------------------------------------------------------- */
/*                              app_main                                       */
/* -------------------------------------------------------------------------- */

void app_main(void) {
    init_nvs();
    printf("[SLAVE] Boot image built on %s %s\n", __DATE__, __TIME__);
    printf("[SLAVE] Greeting: %s\n", SLAVE_GREETING);

    if (!canopen_slave_init()) {
        ESP_LOGE(LOG_TAG, "CANopen slave init failed; halting");
        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    /* Main task idles; CANopen + greeting tasks do the work. */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
