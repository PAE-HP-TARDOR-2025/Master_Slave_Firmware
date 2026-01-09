# CANopen Firmware Update Library

This folder contains the reusable library components for CANopen-based firmware updates between ESP32 devices.

## Structure

```
library/
├── README.md
├── master/              # Master (uploader) library
│   ├── OD.c             # CANopen Object Dictionary (with SDO client)
│   ├── OD.h
│   ├── fw_master_update.c   # Firmware upload logic with CRC/version check
│   └── fw_master_update.h
└── slave/               # Slave (receiver) library
    ├── OD.c             # CANopen Object Dictionary (with firmware objects 0x1F5x)
    ├── OD.h
    ├── fw_update_server.c   # OTA reception and CRC/version handling
    ├── fw_update_server.h
    ├── Kconfig.projbuild    # ESP-IDF menuconfig options
    └── build_slave_bins.py  # Build utility for slave firmware variants
```

## Usage

### Master Side (Firmware Uploader)

1. Copy `library/master/` files to your ESP-IDF project's `main/` folder
2. Add to your `main/CMakeLists.txt`:
   ```cmake
   idf_component_register(
       SRCS "main.c" "OD.c" "fw_master_update.c"
       INCLUDE_DIRS "."
       REQUIRES spiffs   # If loading firmware from SPIFFS
   )
   ```
3. In your main, use:
   ```c
   #include "fw_master_update.h"
   
   // Configure the upload plan
   fw_upload_plan_t plan = {
       .firmwarePath = "/spiffs/firmware.bin",
       .targetNodeId = 10,
       .maxChunkBytes = 896,
       .type = FW_IMAGE_MAIN,
       .targetBank = 0,
       .firmwareVersion = 1,
       .expectedCrc = 0  // Auto-compute if 0, usually its better to just have it precomputed as a pseudo version because of the slave memory partitions and to not have 1/65536 chance of collision. Just put 1 or any other number if you don't want to deal with it.
   };
   
   // Run upload (skips if slave already has matching CRC+version)
   bool success = fw_master_run_upload_if_needed(&plan);
   ```

4. **Implement the weak functions** for your platform:
   - `fw_master_send_metadata()` - SDO download to 0x1F57
   - `fw_master_send_start_command()` - SDO write to 0x1F51
   - `fw_master_send_chunk()` - SDO block download to 0x1F50
   - `fw_master_send_finalize_request()` - SDO write to 0x1F5A
   - `fw_master_query_slave_crc()` - SDO upload from 0x1F5B:01
   - `fw_master_query_slave_version()` - SDO upload from 0x1F5C:01

### Slave Side (Firmware Receiver)

1. Copy `library/slave/` files to your ESP-IDF project's `main/` folder
2. Copy `Kconfig.projbuild` to enable menuconfig options
3. Add to your `main/CMakeLists.txt`:
   ```cmake
   idf_component_register(
       SRCS "main.c" "OD.c" "fw_update_server.c"
       INCLUDE_DIRS "."
       REQUIRES nvs_flash esp_partition app_update esp_timer
   )
   ```
4. In your main, after CANopen init:
   ```c
   #include "fw_update_server.h"
   
   // Initialize firmware server (registers OD handlers)
   if (!fw_server_init(CO)) {
       ESP_LOGE(TAG, "Failed to init firmware server");
   }
   
   // Optionally read running firmware info
   uint16_t crc = fw_server_get_running_crc();
   uint16_t ver = fw_server_get_running_version();
   ```

5. Run `idf.py menuconfig` and configure under "Firmware Update Server Configuration":
   - **Node-ID**: CANopen node ID (1-127)
   - **CAN bitrate**: 125-1000 kbps
   - **Firmware version**: Increment for each release
   - **Max chunk size**: OTA chunk size (64-1024 bytes)
   - **Max image size**: Maximum firmware size

## Object Dictionary

### Slave OD Objects (0x1F5x)
| Index  | Name                     | Access | Description                           |
|--------|--------------------------|--------|---------------------------------------|
| 0x1F50 | programDownload          | WO     | Firmware data stream                  |
| 0x1F51 | programControl           | RW     | Start command (0x01 = start)          |
| 0x1F57 | programIdentification    | RW     | 10-byte metadata [size,crc,type,bank,ver] |
| 0x1F5A | programStatus            | RW     | Finalize CRC (triggers verification)  |
| 0x1F5B | runningFirmwareCRC       | RO     | Current firmware CRC (from NVS)       |
| 0x1F5C | runningFirmwareVersion   | RO     | Current firmware version (from NVS)   |

### Master OD Objects
| Index  | Name                  | Description                        |
|--------|-----------------------|------------------------------------|
| 0x1280 | SDOClientParameter    | SDO client for slave communication |

## Metadata Format (10 bytes)
```
[0-3]  uint32_t imageBytes   - Firmware size in bytes
[4-5]  uint16_t crc          - CRC-16-CCITT
[6]    uint8_t  imageType    - 0=main, 1=bootloader, 2=config
[7]    uint8_t  bank         - Target partition/bank
[8-9]  uint16_t version      - Firmware version (1-65535)
```

## Dual-Check Logic

The master performs intelligent firmware checking:
1. Query slave's running CRC via SDO upload from 0x1F5B:01
2. Query slave's running version via SDO upload from 0x1F5C:01
3. Upload **only if CRC or version differs**
4. Skip upload if **both** CRC and version match

This prevents:
- Unnecessary flash wear on repeated boots
- False positives from CRC collisions (~1/65536 probability)

## Building Slave Variants

Use `build_slave_bins.py` to create multiple firmware variants:

```bash
cd library/slave
python build_slave_bins.py --greeting "hello:Hello World:1" --greeting "bye:Goodbye:2"
```

Format: `NAME:TEXT:VERSION`

Output binaries go to `artifacts/` folder.

## Dependencies

- **CANopenNode v4.x**: Place in `canopennode/` at repository root or as component
- **ESP-IDF v5.0+**: For slave (uses esp_ota_ops, NVS, esp_timer)
- Standard C library for master (portable)

## ESP-IDF Project Setup

### CANopenNode Component

Add CANopenNode to your project. In your project's root `CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)

# Add path to CANopenNode
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/../canopennode")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(your_project)
```

### Partition Table (Slave OTA)

The slave needs an OTA-capable partition table. Create `partitions.csv`:
```csv
# Name,   Type, SubType, Offset,  Size,    Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x100000,
ota_0,    app,  ota_0,   0x110000,0x100000,
ota_1,    app,  ota_1,   0x210000,0x100000,
```

In `sdkconfig` or menuconfig:
```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

### Required sdkconfig Options

For the slave, ensure these are enabled:
```
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_APP_ROLLBACK_ENABLE=y
```

## Complete Example: Master main.c

```c
/**
 * Example master main.c - Firmware Uploader with LED blink demo
 * This uploads firmware to slave node 10, then blinks an LED forever.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* CANopen includes */
#include "CANopen.h"
#include "OD.h"

/* Firmware updater library */
#include "fw_master_update.h"

static const char *TAG = "master_main";

#define LED_GPIO        GPIO_NUM_2
#define SLAVE_NODE_ID   10

/* Global CANopen instance */
CO_t *CO = NULL;

/*---------------------------------------------------------------------------*/
/* Implement the weak SDO functions for your platform                        */
/*---------------------------------------------------------------------------*/

bool fw_master_query_slave_crc(const fw_upload_plan_t* plan, uint16_t* slaveCrc) {
    /* TODO: Implement SDO upload from 0x1F5B:01 using your CANopen stack */
    /* Example with CO_SDOclient:
     * CO_SDO_abortCode_t abort = CO_SDOclientUpload(..., 0x1F5B, 1, ...);
     * if (abort == CO_SDO_AB_NONE) { *slaveCrc = received_value; return true; }
     */
    ESP_LOGW(TAG, "fw_master_query_slave_crc: implement SDO upload 0x1F5B:01");
    *slaveCrc = 0;
    return true;  /* Return false if SDO fails */
}

bool fw_master_query_slave_version(const fw_upload_plan_t* plan, uint16_t* slaveVersion) {
    /* TODO: Implement SDO upload from 0x1F5C:01 */
    ESP_LOGW(TAG, "fw_master_query_slave_version: implement SDO upload 0x1F5C:01");
    *slaveVersion = 0;
    return true;
}

bool fw_master_send_metadata(const fw_upload_plan_t* plan, const fw_payload_t* payload, uint16_t crc) {
    /* TODO: Implement SDO download to 0x1F57 with 10-byte metadata */
    ESP_LOGW(TAG, "fw_master_send_metadata: implement SDO download 0x1F57");
    return true;
}

bool fw_master_send_start_command(const fw_upload_plan_t* plan) {
    /* TODO: Implement SDO write 0x01 to 0x1F51:01 */
    ESP_LOGW(TAG, "fw_master_send_start_command: implement SDO write 0x1F51");
    return true;
}

bool fw_master_send_chunk(const fw_upload_plan_t* plan, const uint8_t* chunk, size_t len, size_t offset) {
    /* TODO: Implement SDO block download to 0x1F50:01 */
    ESP_LOGW(TAG, "fw_master_send_chunk: implement SDO block download 0x1F50");
    return true;
}

bool fw_master_send_finalize_request(const fw_upload_plan_t* plan, uint16_t crc) {
    /* TODO: Implement SDO write CRC to 0x1F5A:01 */
    ESP_LOGW(TAG, "fw_master_send_finalize_request: implement SDO write 0x1F5A");
    return true;
}

/*---------------------------------------------------------------------------*/
/* CANopen initialization (simplified)                                       */
/*---------------------------------------------------------------------------*/

static bool init_canopen(void) {
    /* TODO: Initialize your CAN driver and CANopen stack here */
    /* CO = CO_new(...); CO_CANinit(...); CO_CANopenInit(...); */
    ESP_LOGI(TAG, "CANopen initialized (placeholder)");
    return true;
}

/*---------------------------------------------------------------------------*/
/* Firmware update task                                                      */
/*---------------------------------------------------------------------------*/

static void firmware_update_task(void *arg) {
    ESP_LOGI(TAG, "Starting firmware update check...");

    fw_upload_plan_t plan = {
        .firmwarePath    = "/spiffs/slave_firmware.bin",
        .targetNodeId    = SLAVE_NODE_ID,
        .maxChunkBytes   = 256,
        .type            = FW_IMAGE_MAIN,
        .targetBank      = 0,
        .firmwareVersion = 1,
        .expectedCrc     = 0  /* Auto-compute, or set known CRC */
    };

    bool success = fw_master_run_upload_if_needed(&plan);
    
    if (success) {
        ESP_LOGI(TAG, "Firmware update complete or already up-to-date");
    } else {
        ESP_LOGE(TAG, "Firmware update failed!");
    }

    vTaskDelete(NULL);
}

/*---------------------------------------------------------------------------*/
/* LED blink task (your actual application)                                  */
/*---------------------------------------------------------------------------*/

static void led_blink_task(void *arg) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*---------------------------------------------------------------------------*/
/* Main entry point                                                          */
/*---------------------------------------------------------------------------*/

void app_main(void) {
    ESP_LOGI(TAG, "=== Master Firmware Uploader Starting ===");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize CANopen */
    if (!init_canopen()) {
        ESP_LOGE(TAG, "CANopen init failed");
        return;
    }

    /* Start firmware update task (runs once, then deletes itself) */
    xTaskCreate(firmware_update_task, "fw_update", 8192, NULL, 5, NULL);

    /* Start your main application tasks */
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "All tasks started. Application running...");
}
```

## Complete Example: Slave main.c

```c
/**
 * Example slave main.c - OTA Receiver with sensor reading demo
 * This initializes the firmware server, then reads a sensor periodically.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* CANopen includes */
#include "CANopen.h"
#include "OD.h"

/* Firmware update server library */
#include "fw_update_server.h"

static const char *TAG = "slave_main";

#define LED_GPIO        GPIO_NUM_2
#define SENSOR_GPIO     GPIO_NUM_34  /* Example ADC pin */

/* Global CANopen instance */
CO_t *CO = NULL;

/*---------------------------------------------------------------------------*/
/* CANopen initialization (simplified)                                       */
/*---------------------------------------------------------------------------*/

static bool init_canopen(void) {
    /* TODO: Initialize your CAN driver and CANopen stack here */
    /* CO = CO_new(...); CO_CANinit(...); CO_CANopenInit(...); */
    ESP_LOGI(TAG, "CANopen initialized (placeholder)");
    return true;
}

/*---------------------------------------------------------------------------*/
/* CANopen process task                                                      */
/*---------------------------------------------------------------------------*/

static void canopen_task(void *arg) {
    while (1) {
        /* TODO: Call CO_process() and handle NMT/SDO/PDO */
        /* CO_process(CO, ...); */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*---------------------------------------------------------------------------*/
/* Sensor reading task (your actual application)                             */
/*---------------------------------------------------------------------------*/

static void sensor_task(void *arg) {
    int reading_count = 0;
    
    while (1) {
        /* Simulate sensor reading */
        int sensor_value = reading_count * 10;  /* Replace with real ADC read */
        
        ESP_LOGI(TAG, "Sensor reading #%d: %d", reading_count, sensor_value);
        
        /* Optional: Update OD with sensor value for PDO transmission */
        /* OD_set_u16(OD_ENTRY_xxx, sensor_value); */
        
        reading_count++;
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Read every second */
    }
}

/*---------------------------------------------------------------------------*/
/* Status LED task                                                           */
/*---------------------------------------------------------------------------*/

static void status_led_task(void *arg) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        /* Heartbeat LED - shows slave is running */
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

/*---------------------------------------------------------------------------*/
/* Main entry point                                                          */
/*---------------------------------------------------------------------------*/

void app_main(void) {
    ESP_LOGI(TAG, "=== Slave OTA Receiver Starting ===");

    /* Initialize NVS (required for firmware CRC/version storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize CANopen */
    if (!init_canopen()) {
        ESP_LOGE(TAG, "CANopen init failed");
        return;
    }

    /*=======================================================================*/
    /* FIRMWARE UPDATE SERVER INIT - Must be after CANopen init              */
    /*=======================================================================*/
    if (!fw_server_init(CO)) {
        ESP_LOGE(TAG, "Firmware server init failed!");
        return;
    }

    /* Log current firmware info */
    uint16_t running_crc = fw_server_get_running_crc();
    uint16_t running_ver = fw_server_get_running_version();
    ESP_LOGI(TAG, "Running firmware: CRC=0x%04X, Version=%u", running_crc, running_ver);
    /*=======================================================================*/

    /* Start CANopen processing task */
    xTaskCreate(canopen_task, "canopen", 4096, NULL, 10, NULL);

    /* Start your main application tasks */
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);
    xTaskCreate(status_led_task, "status_led", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "All tasks started. Slave running, ready for OTA...");
}
```

## Quick Start Checklist

### Master
- [ ] Copy `library/master/*` to your project's `main/` folder
- [ ] Add CANopenNode as component
- [ ] Implement the 6 weak SDO functions for your CAN driver
- [ ] Place slave firmware binary in SPIFFS at `/spiffs/slave_firmware.bin`
- [ ] Call `fw_master_run_upload_if_needed()` at startup

### Slave
- [ ] Copy `library/slave/*` to your project's `main/` folder  
- [ ] Add CANopenNode as component
- [ ] Set up OTA partition table (`partitions.csv`)
- [ ] Configure version in `idf.py menuconfig`
- [ ] Call `fw_server_init(CO)` after CANopen init
- [ ] NVS must be initialized before `fw_server_init()`

