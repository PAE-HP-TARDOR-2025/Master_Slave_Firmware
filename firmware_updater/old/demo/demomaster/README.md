| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- |

# Demo CANopen master (ESP32)

This ESP-IDF project embeds the host-side `master_firmware_uploader` so the entire firmware update pipeline runs between two ESP32 boards. The master mounts a SPIFFS partition, reads `/spiffs/<name>.bin`, and pushes it to the slave over CANopen Service Data Objects (SDO) using the vendored CANopenNode stack and the on-chip TWAI controller.

## Highlights

- Same transfer state machine as the desktop uploader (metadata → start → chunked data → finalize).
- SPIFFS storage baked from `storage/` and flashed as the `storage` partition.
- Background FreeRTOS task keeps pulling firmware jobs as soon as the master boots—no button presses required.
- Verbose logging (`[FW-MASTER]`) mirrors every SDO write so you can debug the exchange side-by-side with the slave console.

## Directory overview

```
demo/demomaster/
├── CMakeLists.txt
├── storage/                  ← files copied into the SPIFFS image
├── main/
│   ├── demo_master_app.c     ← mounts SPIFFS, spawns uploader, handles TWAI
│   ├── master_uploader_demo.c/h
│   ├── Kconfig.projbuild     ← firmware path, node IDs, TWAI pins, timeouts
│   └── CMakeLists.txt
├── canopennode/              ← vendored CANopenNode component
└── README.md (this file)
```

## Prepare the firmware file

1. Build the slave binaries (see `demo/build_slave_bins.py`). The generated files land in `demo/artifacts/`.
2. Copy the binary you want to stream (for example `bye.bin`) into the storage folder:
   ```pwsh
   cd demo/demomaster
   copy ..\artifacts\bye.bin storage\bye.bin
   ```
3. Rebuild and flash the SPIFFS image so `/spiffs/bye.bin` exists on the master:
   ```pwsh
   idf.py storage
   idf.py storage-flash -p <MASTER_PORT>
   ```

You can keep multiple binaries under `storage/`; just update the path in menuconfig to choose which file the master opens at boot.

## Configure CANopen + TWAI

Run `idf.py menuconfig` → **Demo master uploader** to adjust:

- **Firmware image path** – default `/spiffs/bye.bin`.
- **Target node ID** – slave node (default 10).
- **Master node ID** – this device (default 100).
- **TWAI bit rate** – 125/250/500/1000 kbps (default 500).
- **TWAI TX / RX GPIO** – GPIO5 / GPIO4 by default; change to match your board.
- **Chunk size** – must not exceed the slave’s `CONFIG_DEMO_SLAVE_MAX_CHUNK_BYTES` (default 256 B).

### Wiring cheat sheet

| Signal | Default GPIO | Notes |
| ------ | ------------ | ----- |
| TWAI TXD | GPIO5 | Connect to transceiver TXD (e.g., SN65HVD230 pin 1). |
| TWAI RXD | GPIO4 | Connect to transceiver RXD (pin 4). |
| 3V3 | 3.3 V | Powers the transceiver. |
| GND | GND | Common ground across nodes. |
| STB/EN | Pull low | Keeps the transceiver active. |
| CANH/L | — | Wire into the CAN bus with 120 Ω termination at both ends. |

Update the GPIO settings if your ESP32 board exposes different pins for TWAI.

## Build, flash, run

```pwsh
cd demo/demomaster
idf.py set-target esp32
idf.py build
idf.py -p <MASTER_PORT> flash monitor
```

After reset the console shows logs similar to:

```
[FW-MASTER] Opening /spiffs/bye.bin (114464 bytes, crc 0x1725)
[FW-MASTER] Metadata write OK
[FW-MASTER] Start command acknowledged
[FW-MASTER] Sent 256-byte chunk @0 (total 256/114464)
…
[FW-MASTER] Firmware upload session completed
```

Leave the master running; it will reattempt the transfer automatically if the slave restarts before completing the finalize step.

## Full workflow recap

1. **Slave baseline** – flash `demo/demoslave` with the “hello” image so you can observe a greeting change.
2. **Master storage** – copy the “bye” image into `storage/`, rebuild, and flash the SPIFFS partition.
3. **Master firmware** – configure CAN parameters, build, and flash this project.
4. **Streaming** – connect both boards to the same CAN bus; the master starts writing metadata/chunks immediately.
5. **Verification** – watch the slave log report `Firmware image validated`. The slave auto reboots; no manual reset is needed anymore.

## Troubleshooting

- **`Failed to open firmware file`** – confirm the SPIFFS image contains the file path printed in the error and that you reflashed `storage` after copying.
- **`CO_SDOclient` aborts** – make sure the slave is powered and reachable on the same bit rate/node ID.
- **`Chunk rejected` on the slave** – the master will retry the current block; if it fails repeatedly, restart the master to restart the session from metadata.
- **SPIFFS too small** – edit `partitions.csv` or shrink your firmware binary; the default `storage` partition is sized for single ~1 MB image files.

Once the master prints “Firmware upload session completed” and the slave boots into the new greeting automatically, your CANopen OTA path is fully operational.
