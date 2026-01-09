# Firmware Updater Reference

This repository bundles a complete CANopen firmware update path that now runs entirely on two ESP-IDF applications:

- `demo/demoslave` – CANopen slave that validates metadata, streams chunks into the inactive OTA partition, sets the next boot partition, and automatically reboots once the image is verified.
- `demo/demomaster` – CANopen master that mounts a SPIFFS partition, serves firmware binaries from `/spiffs/*.bin`, and drives the CiA‑302 download sequence over TWAI using CANopenNode.

You can reuse the same state machines on other hardware (desktop, STM32, Raspberry Pi, etc.), but the demos make it easy to observe the whole loop on a pair of ESP32 boards.

## Key Implementation Notes (Lessons Learned)

These critical details were discovered during development and **must** be followed:

1. **CRC-16 CCITT Algorithm** – Both master and slave MUST use identical CRC:
   ```c
   uint16_t crc = 0xFFFF;  // Initial value
   for each byte:
       crc ^= byte << 8;
       for 8 bits:
           if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
           else crc <<= 1;
   // No final XOR, transmitted little-endian
   ```

2. **NVS CRC Storage** – Slave stores verified CRC in NVS after successful OTA. On boot, it reads CRC from NVS (not flash) for reliable reporting. This solves the "0xFF detection" problem.

3. **SDO Buffer Size is 32 bytes** – CANopenNode's default SDO buffer is only 32 bytes. You cannot write 256 bytes at once. Fill the buffer progressively as space becomes available during the transfer loop.

4. **Metadata Format** – 10 bytes little-endian to `0x1F57:01`:
   ```
   [0-3] size (uint32_t)
   [4-5] CRC-16 (uint16_t) 
   [6]   image type
   [7]   target bank
   [8-9] version (uint16_t)
   ```

5. **Start Command** – 3 bytes `{0x01, 0x00, 0x00}` to `0x1F51:01`

6. **Finalize** – 2-byte CRC little-endian to `0x1F5A:01` (not 0x1F52!)

7. **Query Slave CRC** – SDO upload from `0x1F5B:01` returns 2-byte CRC. Use this to skip upload if firmware already matches.

8. **Query Slave Version** – SDO upload from `0x1F5C:01` returns 2-byte version. Combined with CRC, ensures reliable change detection.

9. **Dual Check Logic** – Upload is skipped only if **both** CRC and version match. This avoids false positives from CRC collisions (~1/65536 probability).

10. **SDO Timeout** – Use 1000-3000ms for reliability over CAN bus.

11. **Use Segmented Transfer** – Block transfer may not be supported by all slaves. Segmented transfer (7 bytes per segment) is more compatible.

## Repository layout

```
firmware_updater/
├── main_firmware_update.c           ← generic CANopen slave reference implementation
├── master_firmware_uploader.c       ← desktop/host master reference
├── library/                         ← ⭐ REUSABLE LIBRARY (start here for new projects)
│   ├── README.md                    ← Complete usage guide with example main.c files
│   ├── master/                      ← Master firmware uploader library
│   │   ├── OD.c, OD.h               ← CANopen Object Dictionary (SDO client)
│   │   ├── fw_master_update.c       ← Upload logic with CRC/version check
│   │   └── fw_master_update.h       ← API: fw_master_run_upload_if_needed()
│   └── slave/                       ← Slave OTA receiver library
│       ├── OD.c, OD.h               ← CANopen Object Dictionary (0x1F5x objects)
│       ├── fw_update_server.c       ← OTA handler with NVS CRC/version storage
│       ├── fw_update_server.h       ← API: fw_server_init(), fw_server_get_running_crc()
│       ├── Kconfig.projbuild        ← ESP-IDF menuconfig options
│       └── build_slave_bins.py      ← Build utility for firmware variants
├── demo/
│   ├── build_slave_bins.py       ← helper that builds multiple slave greetings
│   ├── artifacts/                ← `.bin` output staged for uploads
│   ├── demoslave/                ← ESP-IDF slave project (OTA, auto reboot)
│   └── demomaster/               ← ESP-IDF master project (SPIFFS + CANopen SDO)
├── librarywithfunctions/            ← Full working demos with all dependencies
│   ├── master/                   ← ESP32 master implementation (CANopenNode + TWAI)
│   │   └── demomaster/           ← ESP-IDF project with SPIFFS storage
│   ├── slave/                    ← ESP32 slave implementation (OTA + CANopen)
│   │   ├── demoslave/            ← ESP-IDF project with OTA support
│   │   ├── artifacts/            ← hello.bin / bye.bin built from demoslave
│   │   ├── build_slave_bins.py   ← builds greeting variants
│   │   ├── slave_main.c          ← full CANopen slave template with CRC reporting
│   │   ├── fw_update_server.[ch] ← ESP-IDF OTA handler with NVS CRC storage
│   │   └── fw_slave_update.[ch]  ← platform-neutral state machine
│   ├── raspberry_master/         ← Raspberry Pi SocketCAN master (production-ready)
│   │   ├── main_rpi_master.c     ← Main application entry point
│   │   ├── sdo_client.[ch]       ← Standalone SDO client implementation
│   │   ├── fw_master_update.[ch] ← CRC computation and upload logic
│   │   ├── rpi_can.[ch]          ← SocketCAN wrapper
│   │   └── Makefile              ← Build configuration
│   └── partitions/               ← CSV partition tables for master/slave
└── README.md (this file)
```

## Getting Started

### Option 1: Use the Library (Recommended for New Projects)

The `library/` folder contains clean, reusable code that you can copy into any ESP-IDF project:

1. Copy `library/master/` or `library/slave/` files to your project's `main/` folder
2. Follow the examples in `library/README.md`
3. Implement the weak SDO functions for master (or use the slave as-is)

See [`library/README.md`](library/README.md) for complete example `main.c` files and setup instructions.

### Option 2: Run the Full Demos

The `librarywithfunctions/` folder contains complete working projects you can flash directly.

Each project has its own detailed README:
- `librarywithfunctions/master/demomaster/README.md` – ESP32 master documentation
- `librarywithfunctions/slave/demoslave/README.md` – ESP32 slave documentation  
- `librarywithfunctions/raspberry_master/README.md` – Raspberry Pi master documentation
- `librarywithfunctions/README.md` – Overall library documentation

## Quick start (two ESP32 boards)

1. **Generate slave firmware variants**
   ```pwsh
   # From the library folder (preferred)
   cd librarywithfunctions/slave
   
   # Build with greeting and version
   python build_slave_bins.py --greeting "hello:Hello from slave:1" --greeting "bye:Goodbye from slave:2"
   # Creates artifacts/hello.bin (version 1) and artifacts/bye.bin (version 2)
   ```
   
   **Format:** `--greeting "name:text:version"` where:
   - `name` – Output filename (e.g., `bye` → `bye.bin`)
   - `text` – Greeting message displayed by slave
   - `version` – Firmware version (1-65535), used by master to detect changes
   
   The script:
   - Creates deterministic build folders (`demoslave/build-hello`, `build-bye`, …)
   - Copies the finished binaries into `artifacts/hello.bin` and `artifacts/bye.bin`

2. **Flash the baseline image to the slave**
   ```pwsh
   idf.py -C demo/demoslave -B build-hello -p <SLAVE_PORT> flash monitor
   ```
   Keep the monitor open to watch `[SLAVE]` and `[fw_server]` logs.

3. **Stage the update for the master**
   ```pwsh
   cd librarywithfunctions/master/demomaster
   
   # Copy the "bye" firmware (version 2) to master storage
   copy ..\..\slave\artifacts\bye.bin storage\bye.bin
   
   # Build and flash SPIFFS partition
   idf.py storage
   idf.py storage-flash -p <MASTER_PORT>
   ```
   The SPIFFS partition named `storage` now contains `/spiffs/bye.bin`.

4. **Configure and flash the master**
   ```pwsh
   idf.py menuconfig   # Set TWAI pins, bitrate, firmware path, target node ID, firmware version
   idf.py build
   idf.py -p <MASTER_PORT> flash monitor
   ```
   The master automatically starts the uploader task and runs forever.

5. **Observe the OTA transfer**
   - The master prints `[FW-MASTER]` metadata, start, chunk, and finalize logs.
   - Master queries slave CRC (`0x1F5B:01`) and version (`0x1F5C:01`) before upload.
   - If both match, upload is skipped. If either differs, transfer proceeds.
   - The slave prints `[fw_server]` chunk confirmations, CRC verification, and finally `Firmware image validated …`.
   - One second later the slave reboots itself and loads the `ota_0` partition with new firmware.

## How the pieces work together

| Stage | Master (`demo/demomaster`) | Slave (`demo/demoslave`) |
| ----- | ------------------------- | ------------------------ |
| Query CRC (0x1F5B) | Reads slave's running CRC via SDO upload | Returns CRC from NVS |
| Query Version (0x1F5C) | Reads slave's running version via SDO upload | Returns version from NVS |
| Decision | If BOTH match: skip upload. If EITHER differs: proceed | - |
| Metadata (0x1F57) | Pushes size/CRC/bank/type/version (10 bytes) | Validates limits, prepares internal state |
| Start (0x1F51) | Issues CiA‑302 start command | Calls `esp_ota_begin()` on the inactive OTA partition |
| Data (0x1F50) | Streams file in <= chunk size blocks (default 256 B) | Pipes data straight into `esp_ota_write()` while computing CRC |
| Status (0x1F5A) | Sends final CRC | Verifies CRC, calls `esp_ota_end()`, selects new partition, saves CRC+version to NVS, schedules auto reboot |

Key ESP-IDF features in use:

- Two OTA partitions on a 4 MB flash map (`partitions_two_ota.csv`).
- `esp_ota_get_next_update_partition()`/`esp_ota_write()`/`esp_ota_set_boot_partition()`.
- Auto reboot through an ESP timer that fires ~500 ms after validation so logs reach the console before reset.
- TWAI (CAN) driver + CANopenNode stack to speak SDO.
- SPIFFS image baked from `demo/demomaster/storage/` to distribute firmware files.

## Reusing the components

- **Library folder (`librarywithfunctions/`)** – a self-contained snapshot of the core OTA helpers without the rest of the demo scaffolding. See its own `README.md` for integration details.
- **Slave template (`librarywithfunctions/slave/slave_main.c`)** – full ESP-IDF CANopen slave main with firmware update server integration and running CRC/version reporting. Use this as a starting point for new slave firmware.
- **Slave reference (`main_firmware_update.c`)** – drop this file into any CANopenNode project to get the same metadata state machine and CRC validation. Replace the ESP-specific storage hooks with your platform's flash drivers.
- **ESP32 Master (`librarywithfunctions/master/demomaster/`)** – complete ESP-IDF project using CANopenNode SDO client, SPIFFS storage, and TWAI driver. Production-ready.
- **Raspberry Pi Master (`librarywithfunctions/raspberry_master/`)** – production-ready SocketCAN implementation with standalone SDO client:
  ```bash
  # Build
  cd librarywithfunctions/raspberry_master
  make
  
  # Setup CAN interface
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
  
  # Run with version
  ./rpi_uploader firmware.bin -n 10 -v 2
  ```
  Features:
  - Full SDO expedited and segmented transfers
  - CRC-16 computation matching ESP32 slave
  - Version-aware dual check (CRC + version)
  - Command-line interface: `-n <node>`, `-v <version>`, `-f` (force), `-i <interface>`
- **Build helper (`build_slave_bins.py`)** – reproducibly generates multiple slave binaries by greeting name and version. Essential for CI/CD pipelines:
  ```bash
  python build_slave_bins.py --greeting "name:text:version"
  # Example: --greeting "prod:Production Firmware:42"
  ```
- **Running CRC API (`fw_server_get_running_crc()`)** – allows the slave to expose its running firmware CRC via CANopen object 0x1F5B, enabling the master to check firmware. CRC is stored in NVS for reliability.
- **Running Version API (`fw_server_get_running_version()`)** – allows the slave to expose its firmware version via CANopen object 0x1F5C. Combined with CRC, provides reliable dual-check to skip unnecessary uploads.
- **Dual Check Logic** – Master uploads firmware only if CRC **or** version differs, eliminating false positives from CRC collisions (~1/65536 probability).

## Integrating into Production Projects

The demo projects are intentionally minimal (just a greeting task). Here's how to integrate firmware update capability into larger, real-world applications.

### Slave Integration Checklist

Your production slave likely has sensors, actuators, control loops, communication tasks, etc. To add OTA support:

1. **Include the firmware update files**
   ```c
   // In your main application
   #include "fw_update_server.h"
   ```

2. **Initialize firmware update server after CANopenNode**
   ```c
   void app_main(void) {
       // 1. Your existing initialization (NVS, peripherals, etc.)
       nvs_flash_init();
       my_sensor_init();
       my_motor_controller_init();
       
       // 2. Initialize CANopenNode
       CO_t* co = co_init(...);
       
       // 3. Initialize firmware update server (adds OD objects)
       fw_server_init(co);
       
       // 4. Start your application tasks
       xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);
       xTaskCreate(control_task, "control", 4096, NULL, 6, NULL);
       
       // 5. CANopen processing loop (or in a dedicated task)
       while (1) {
           CO_process(co, 1000);
           vTaskDelay(1);
       }
   }
   ```

3. **Expose CRC and version for master queries**
   ```c
   // The firmware server automatically registers:
   // - 0x1F5B:01 → fw_server_get_running_crc()
   // - 0x1F5C:01 → fw_server_get_running_version()
   // No additional code needed!
   ```

4. **Handle reboot gracefully**
   - After successful OTA, the slave schedules a reboot in 500ms
   - Ensure your application can handle sudden restarts
   - Consider adding a "shutdown hook" to save state before reboot:
   ```c
   // In fw_update_server.c, before scheduling reboot:
   app_save_critical_state();  // Your function
   ```

5. **Version management in your build**
   - Set `CONFIG_DEMO_SLAVE_FW_VERSION` in `Kconfig` or `sdkconfig.defaults`
   - Or use build script: `python build_slave_bins.py --greeting "prod:Production:42"`
   - **Important:** Increment version for every release!

### Master Integration Checklist

Your production master might be a gateway, HMI, or central controller. To add firmware upload capability:

1. **ESP32 Master: Add uploader as a task**
   ```c
   void app_main(void) {
       // Your existing initialization
       init_display();
       init_network();
       
       // Initialize CANopenNode
       CO_t* co = co_init(...);
       
       // Start firmware uploader task (runs in background)
       xTaskCreate(firmware_uploader_task, "fw_upload", 8192, co, 3, NULL);
       
       // Your main application loop
       while (1) {
           update_display();
           handle_user_input();
           CO_process(co, 1000);
           vTaskDelay(10);
       }
   }
   
   void firmware_uploader_task(void* arg) {
       vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for system to stabilize
       
       fw_upload_plan_t plan = {
           .firmwarePath = "/spiffs/slave_fw.bin",
           .targetNodeId = 10,
           .firmwareVersion = 2,
           // ... other settings
       };
       
       // Check and upload if needed (non-blocking to other tasks)
       fw_master_run_upload_if_needed(&plan);
       
       vTaskDelete(NULL);  // Or loop to check periodically
   }
   ```

2. **Raspberry Pi Master: Integrate into your application**
   ```c
   // Option A: Call as subprocess
   system("./rpi_uploader /opt/firmware/slave_v2.bin -n 10 -v 2");
   
   // Option B: Link the library directly
   #include "fw_master_update.h"
   
   void update_slaves(void) {
       fw_upload_plan_t plan = {
           .firmwarePath = "/opt/firmware/slave_v2.bin",
           .targetNodeId = 10,
           .firmwareVersion = 2,
           .maxChunkBytes = 256,
           .targetBank = 1,
           .type = FW_IMAGE_MAIN
       };
       
       if (!fw_master_run_upload_if_needed(&plan)) {
           log_error("Firmware update failed for node 10");
       }
   }
   ```

3. **Multi-node updates**
   ```c
   // Update multiple slaves sequentially
   uint8_t slave_nodes[] = {10, 11, 12, 13};
   for (int i = 0; i < sizeof(slave_nodes); i++) {
       plan.targetNodeId = slave_nodes[i];
       fw_master_run_upload_if_needed(&plan);
   }
   ```

4. **Firmware storage strategies**
   - **SPIFFS** (ESP32): Embed firmware in flash partition
   - **SD Card** (ESP32): `plan.firmwarePath = "/sdcard/firmware.bin"`
   - **Network download** (RPi): Download from server, then upload to slaves
   - **USB drive** (RPi): Mount and read from `/mnt/usb/firmware.bin`

### Production Build Workflow

```
┌─────────────────────────────────────────────────────────────────────┐
│ 1. DEVELOP: Make changes to slave firmware                         │
├─────────────────────────────────────────────────────────────────────┤
│ 2. VERSION: Increment CONFIG_DEMO_SLAVE_FW_VERSION                  │
│    sdkconfig: CONFIG_DEMO_SLAVE_FW_VERSION=43                       │
├─────────────────────────────────────────────────────────────────────┤
│ 3. BUILD: Generate binary with version embedded                     │
│    python build_slave_bins.py --greeting "prod:MyApp:43"            │
│    → artifacts/prod.bin                                             │
├─────────────────────────────────────────────────────────────────────┤
│ 4. DEPLOY: Copy binary to master(s)                                 │
│    ESP32: Copy to storage/, rebuild SPIFFS, flash storage partition │
│    RPi:   scp prod.bin pi@gateway:/opt/firmware/                    │
├─────────────────────────────────────────────────────────────────────┤
│ 5. TRIGGER: Master uploads to slaves                                │
│    ESP32: Automatic on boot, or trigger via command                 │
│    RPi:   ./rpi_uploader /opt/firmware/prod.bin -n 10 -v 43         │
├─────────────────────────────────────────────────────────────────────┤
│ 6. VERIFY: Check slave logs for successful update                   │
│    [fw_server] Firmware image validated, CRC=0xABCD, version=43     │
│    [SLAVE] Production firmware v43 running                          │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Files for Integration

| File | Purpose | Copy To Your Project |
|------|---------|---------------------|
| `slave/fw_update_server.[ch]` | ESP-IDF OTA handler with NVS | Slave project |
| `slave/fw_slave_update.[ch]` | Platform-neutral state machine | Any slave |
| `slave/OD.[ch]` | Object Dictionary with 0x1F5x objects | Slave project |
| `master/fw_master_update.[ch]` | Upload logic and CRC computation | Master project |
| `raspberry_master/main_rpi_master.c` | Complete RPi implementation | Linux master |
| `raspberry_master/sdo_client.[ch]` | Standalone SDO client | Linux master |
| `build_slave_bins.py` | Build automation | CI/CD pipeline |

### Version Numbering Best Practices

- Use **semantic versioning** mapped to uint16_t: `major*1000 + minor*10 + patch`
  - v1.2.3 → version 1023
  - v2.0.0 → version 2000
- Store version in `Kconfig` for compile-time embedding
- **Never reuse version numbers** – always increment!
- Consider storing version in a header file for easy CI extraction:
  ```c
  // version.h (generated by CI)
  #define FW_VERSION_MAJOR 1
  #define FW_VERSION_MINOR 2
  #define FW_VERSION_PATCH 3
  #define FW_VERSION_NUMBER ((FW_VERSION_MAJOR * 1000) + (FW_VERSION_MINOR * 10) + FW_VERSION_PATCH)
  ```

## Troubleshooting cheatsheet

- `Chunk rejected: expected offset …` – master and slave lost sync. Verify SDO clients aren't retransmitting stale segments.
- `Image size exceeds OTA partition` – adjust flash size in `demo/demoslave/sdkconfig` or reduce application footprint.
- `esp_ota_set_boot_partition` errors – ensure both `ota_0` and `ota_1` partitions exist and that the binary fits inside them.
- Master stuck waiting for file – confirm `/spiffs/<name>.bin` exists and that you reflashed the `storage` partition after copying the new file.
- No reboot after finalize – the slave now schedules its own restart; if you disable auto reboot via Kconfig, manually reset the board after the `[fw_server] Firmware image validated` log.
- **Upload happens every boot** – check that slave saves CRC+version to NVS after successful update (verify `fw_save_to_nvs()` is called).
- **Version always reports 1** – ensure `CONFIG_DEMO_SLAVE_FW_VERSION` is set correctly and passed to `build_slave_bins.py`.
- **CRC matches but version differs** – this is correct behavior; upload will proceed because dual-check requires BOTH to match.

## Next steps

1. Review `demo/demoslave/README.md` for configurable node IDs, TWAI pins, and OTA tuning knobs.
2. Review `demo/demomaster/README.md` for storage layout, wiring diagrams, and the SPIFFS workflow.
3. Port the reference code into your production projects, replacing the dummy greeting logic with real application payloads.

With these pieces running, you now have a proven CANopen OTA flow that starts from a freshly built `.bin`, stages it on the master, and ends with the slave rebooting into the new firmware automatically.
