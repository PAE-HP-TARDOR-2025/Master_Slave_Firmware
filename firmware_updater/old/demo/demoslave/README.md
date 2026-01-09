# OTA-ready CANopen slave (ESP32)

This ESP-IDF project turns an ESP32 board into a CANopen slave that accepts firmware downloads over CiA‑302 objects and writes them straight into an OTA partition. After the CRC is validated the slave automatically schedules an `esp_restart()` and boots the freshly written image.

The same application supplies the demo firmware that we hand to the master uploader (greetings such as "Hello from slave" or "Bye from slave").

## Features

- CANopenNode stack configured as node ID **10** by default.
- Firmware download objects 0x1F50, 0x1F51, 0x1F57, 0x1F5A, and 0x1F5B wired into `fw_update_server.c`.
- Metadata validation (size limit, CRC16/CCITT, bank/type hints).
- Direct streaming into the inactive OTA partition via `esp_ota_*` APIs.
- Auto reboot 500 ms after a successful finalize so logs flush before reset.
- Configurable heartbeat prints through the `SLAVE_GREETING` string.
- **Running CRC API** – `fw_server_get_running_crc()` exposes the CRC of the currently running firmware via CANopen object `0x1F5B:01`, allowing the master to skip OTA if firmware already matches.

Read the root `README.md` for the high-level workflow; this file focuses on the slave build itself.

## Project layout

```
demo/demoslave/
├── CMakeLists.txt
├── sdkconfig            ← checked-in base config (4 MB flash, dual OTA)
├── main/
│   ├── dummy_slave_main.c  ← app_main that runs CANopen + greeting prints
│   ├── fw_update_server.c  ← OTA state machine exposed via CANopen
│   ├── fw_update_server.h
│   ├── Kconfig.projbuild   ← greeting, node-id, TWAI pins, chunk size
│   └── CMakeLists.txt
└── README.md (this file)
```

## Build the demo binaries

The repository root contains `demo/build_slave_bins.py`, which compiles as many greeting variants as you need without re-running menuconfig for every build:

```pwsh
cd demo
python build_slave_bins.py --greeting hello:"Hello from slave" --greeting bye:"Bye from slave"
```

What you get:

- `demo/demoslave/build-hello/` and `demo/demoslave/build-bye/` with full `idf.py` outputs.
- `demo/artifacts/hello.bin` and `demo/artifacts/bye.bin` ready to copy into the master storage partition.

**Alternative:** Use the library version in `librarywithfunctions/slave/`:

```pwsh
cd librarywithfunctions/slave
python build_slave_bins.py
# Produces: artifacts/hello.bin ("Hello from slave") and artifacts/bye.bin ("Bye from slave")
```

To only build one variant with the default greeting:

```pwsh
python build_slave_bins.py --greeting hello
```

## Flashing and monitoring

Pick the build directory that matches the greeting you want on the device:

```pwsh
idf.py -C demo/demoslave -B build-hello -p COM5 flash monitor
```

Key logs to watch:

- `[SLAVE] Hello from slave` – heartbeat proving the application task is alive.
- `[fw_server] Metadata accepted / Chunk @... / Firmware image validated` – OTA progress.
- Automatic reset approximately half a second after validation with the new greeting printed immediately after boot.

## Configuration knobs (menuconfig)

Open `idf.py -C demo/demoslave menuconfig` and look under **Dummy slave demo**:

- **Default slave greeting** – fallback string used by `dummy_slave_main.c`.
- **Slave node identifier** – CANopen node ID (default 10).
- **TWAI TX/RX GPIO** – pins that connect to your CAN transceiver (default TX=5, RX=4).
- **Maximum accepted chunk size** – caps SDO block size (default 256 bytes).
- **Maximum firmware image size** – rejects metadata that would overflow the OTA slot (default 512 KiB).

Global ESP-IDF settings to keep in mind:

- Flash size must stay at **4 MB** with the included `partitions_two_ota.csv` layout so each OTA partition has 1 MB available.
- The project depends on `app_update` and CANopenNode components, so `idf.py set-target esp32` (or another supported ESP32-class part) before building.

## CAN / TWAI wiring

This project expects an external CAN transceiver such as SN65HVD230.

| Signal | Default GPIO | Notes |
| ------ | ------------ | ----- |
| TWAI TXD | GPIO5 | Connects to transceiver TXD pin |
| TWAI RXD | GPIO4 | Connects to transceiver RXD pin |
| 3V3 | 3.3 V rail | Powers the transceiver |
| GND | GND | Common ground between all nodes |
| STB / EN | Pull low | Keeps transceiver out of standby |
| CANH / CANL | Bus wires | Terminate with 120 Ω at each end |

Edit the GPIO values in menuconfig if your development board routes CAN to different pins.

## OTA lifecycle

1. **Metadata** (`0x1F57:01`) – the slave stores the expected size, CRC, type, and bank once the master writes the packed metadata structure.
2. **Start** (`0x1F51:01`) – triggers `esp_ota_begin()` on the inactive OTA partition reported by `esp_ota_get_next_update_partition()`.
3. **Data** (`0x1F50:01`) – every SDO download block writes directly into flash while running a CRC16 update.
4. **Finalize** (`0x1F5A:01`) – compares CRC, calls `esp_ota_end()`, selects the new partition, logs success, and starts a one-shot timer that issues `esp_restart()` after 500 ms.
5. **Running CRC** (`0x1F5B:01`) – the master can read this object to get the CRC of the slave's currently running firmware, enabling skip-if-matching logic.

If any step fails, the slave logs the reason and you can retry from the metadata stage without power-cycling.

## Troubleshooting tips

- **`Chunk rejected: expected offset …`** – ensure the master did not skip blocks. Clear the session by re-sending metadata.
- **`Image size exceeds OTA partition`** – rebuild the app with fewer features or increase the flash size + partition table in `sdkconfig`.
- **No reboot after finalize** – auto reboot can be disabled at build time through `CONFIG_DEMO_SLAVE_AUTO_REBOOT_AFTER_OTA`. If you turned it off, manually reset the board to boot the newly programmed partition.
- **CAN errors** – check TWAI wiring and confirm both nodes share the same bit rate (default 500 kbps).

Once these steps work, the slave will automatically switch partitions and start printing the new greeting after every OTA transfer from the master.
