# ESP32 CANopen Master Firmware Uploader

A complete ESP-IDF CANopen master that uploads firmware to slave devices over CAN bus using CiA 302 firmware update protocol.

## ✅ Verified Working

- Queries slave CRC via `0x1F5B:01` before upload
- Queries slave **firmware version** via `0x1F5C:01` before upload
- Skips upload when **both CRC and version match** (saves time and flash wear)
- Streams firmware chunks via SDO (256 bytes each)
- Full progress reporting during transfer

## Features

- **CANopen SDO Client** – Full implementation with segmented and block transfer support
- **Dual Check (CRC + Version)** – Queries slave CRC and version before upload, skips only if **both** match
- **SPIFFS Storage** – Firmware binaries stored in flash partition
- **Streaming Transfer** – Memory-efficient, doesn't load entire binary to RAM
- **Configurable** – All settings adjustable via Kconfig menuconfig

**Why dual check?** CRC alone has ~1/65536 collision probability. By also checking version, the master ensures firmware changes are detected even in rare CRC collisions.

## Quick Start

### 1. Prepare Firmware Binary
```powershell
# Build slave with desired greeting and version
cd ../../../slave
python build_slave_bins.py --greeting "bye:Goodbye from slave:2"
# Creates artifacts/bye.bin with version 2

# Copy to master storage
cp artifacts/bye.bin ../../master/demomaster/storage/bye.bin
```

**Format:** `--greeting "name:text:version"` where:
- `name` – Output filename (e.g., `bye` → `bye.bin`)
- `text` – Greeting message
- `version` – Firmware version (1-65535)

### 2. Build and Flash
```powershell
cd complete_demo/master/demomaster
idf.py build flash -p COM6 monitor
```

## Configuration

Run `idf.py menuconfig` → **Master Uploader Configuration**:

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_MASTER_NODE_ID` | 1 | CANopen node ID for master |
| `CONFIG_MASTER_TARGET_NODE_ID` | 10 | Target slave node ID |
| `CONFIG_MASTER_TWAI_TX_GPIO` | 5 | CAN transceiver TX pin |
| `CONFIG_MASTER_TWAI_RX_GPIO` | 4 | CAN transceiver RX pin |
| `CONFIG_MASTER_FIRMWARE_PATH` | `/spiffs/bye.bin` | Path to firmware in SPIFFS |
| `CONFIG_MASTER_MAX_CHUNK_BYTES` | 256 | Chunk size for transfer |
| `CONFIG_MASTER_SKIP_IF_CRC_MATCH` | y | Skip upload if CRC **and version** match |
| `CONFIG_MASTER_UPLOAD_ON_STARTUP` | y | Auto-upload when master boots |
| `CONFIG_MASTER_FIRMWARE_VERSION` | 1 | Firmware version embedded in metadata |

## How It Works

### Boot Sequence
```
1. Initialize NVS flash
2. Mount SPIFFS partition at /spiffs
3. Initialize CANopen stack (node ID 1)
4. Start uploader task after 2 second delay
```

### Upload Process
```
┌─────────────────────────────────────────────────────────────┐
│ 1. Open /spiffs/bye.bin and get file size                  │
├─────────────────────────────────────────────────────────────┤
│ 2. Compute CRC-16 of entire file                           │
│    CRC = 0xFFFF initial, polynomial 0x1021                  │
├─────────────────────────────────────────────────────────────┤
│ 3. Query slave CRC via SDO upload (0x1F5B:01)              │
│    TX: [40 5B 1F 01 00 00 00 00]                           │
│    RX: [4B 5B 1F 01 XX XX 00 00] (XX XX = CRC little-end)  │
├─────────────────────────────────────────────────────────────┤
│ 4. Query slave version via SDO upload (0x1F5C:01)          │
│    TX: [40 5C 1F 01 00 00 00 00]                           │
│    RX: [4B 5C 1F 01 VV VV 00 00] (VV VV = version LE)      │
├─────────────────────────────────────────────────────────────┤
│ 5. Compare CRC AND version                                  │
│    If BOTH match: "Skipping upload" → Done                  │
│    If EITHER differs: Proceed to step 6                     │
├─────────────────────────────────────────────────────────────┤
│ 6. Send metadata to 0x1F57:01 (10 bytes)                   │
│    [size(4)|crc(2)|type(1)|bank(1)|version(2)] little-end  │
├─────────────────────────────────────────────────────────────┤
│ 7. Send start command to 0x1F51:01 (3 bytes)               │
│    [0x01, 0x00, 0x00]                                       │
├─────────────────────────────────────────────────────────────┤
│ 8. Stream firmware chunks to 0x1F50:01                     │
│    256 bytes per chunk, progress logged every 10%           │
├─────────────────────────────────────────────────────────────┤
│ 9. Send finalize to 0x1F5A:01 (2 bytes)                    │
│    [CRC low byte, CRC high byte]                            │
├─────────────────────────────────────────────────────────────┤
│10. Slave validates CRC, sets boot partition, reboots       │
└─────────────────────────────────────────────────────────────┘
```

## SDO Implementation Details

### Buffer Limitation
CANopenNode's SDO buffer is only **32 bytes** by default. The master writes data progressively:

```c
size_t offset = 0;
do {
    // Fill buffer as space becomes available
    size_t written = CO_SDOclientDownloadBufWrite(sdoClient, data + offset, len - offset);
    offset += written;
    
    // Process transfer
    ret = CO_SDOclientDownload(sdoClient, timeDiff, false, offset < len, &abort, &transferred, NULL);
    if (ret == CO_SDO_RT_waitingResponse) vTaskDelay(1);
} while (ret > CO_SDO_RT_ok_communicationEnd);
```

### SDO Client Setup
For each transfer, configure the SDO client for the target node:
```c
CO_SDOclient_setup(sdoClient,
    0x600 + nodeId,  // COB-ID client→server (TX)
    0x580 + nodeId,  // COB-ID server→client (RX)
    nodeId);
```

### Timeouts
- Default: 3000ms (`SDO_CLI_TIMEOUT_TIME`)
- Block transfer enabled: `SDO_CLI_BLOCK = true`

## CRC-16 Calculation

```c
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
// CRC is computed over entire firmware binary
```

## SPIFFS Storage

The `storage/` folder is packaged as a SPIFFS partition during build:

```
storage/
└── bye.bin    ← Place firmware binary here
```

### Partition Layout (`partitions.csv`)
```csv
# Name,   Type, SubType, Offset,   Size
nvs,      data, nvs,     0x9000,   0x4000
otadata,  data, ota,     0xd000,   0x2000
phy_init, data, phy,     0xf000,   0x1000
storage,  data, spiffs,  0x10000,  0x80000   # 512 KB for firmware binaries
factory,  app,  factory, 0x90000,  0x170000  # ~1.4 MB for master app
```

## Example Console Output

### CRC and Version Match (No Upload Needed)
```
I (2447) master_main: Starting firmware upload task
I (2447) master_main: Upload plan: file=/spiffs/bye.bin node=10
I (2467) master_main: Firmware file size: 253120 bytes
I (3017) master_main: Computed CRC: 0x1A00
I (3017) master_main: Querying slave CRC from node 10 (0x1F5B:01)
I (3037) master_main: Slave running firmware CRC: 0x1A00
I (3047) master_main: Querying slave version from node 10 (0x1F5C:01)
I (3067) master_main: Slave running firmware version: 2
I (3077) master_main: Both CRC and version match, skipping upload
```

### CRC or Version Mismatch (Upload Proceeds)
```
I (2447) master_main: Starting firmware upload task
I (2447) master_main: Upload plan: file=/spiffs/bye.bin node=10 version=2
I (2467) master_main: Firmware file size: 253120 bytes
I (3017) master_main: Computed CRC: 0x1A00
I (3017) master_main: Querying slave CRC from node 10 (0x1F5B:01)
I (3037) master_main: Slave running firmware CRC: 0x1A00
I (3047) master_main: Querying slave version from node 10 (0x1F5C:01)
I (3067) master_main: Slave running firmware version: 1
I (3077) master_main: Version mismatch (slave=1, local=2), proceeding with upload
I (3087) master_main: Sending metadata: size=253120 crc=0x1A00 type=0 bank=1 version=2
I (3107) master_main: Sending start command to node 10
I (3127) master_main: Upload progress: 25312/253120 bytes (10%)
I (4127) master_main: Upload progress: 50624/253120 bytes (20%)
... (continues)
I (15127) master_main: Sent 253120 bytes total
I (15137) master_main: Sending finalize with CRC 0x1A00
I (15157) master_main: Firmware upload completed successfully!
```

## File Structure

```
demomaster/
├── CMakeLists.txt           # Build configuration, SPIFFS packaging
├── partitions.csv           # Flash partition layout
├── sdkconfig                # Build configuration
├── sdkconfig.defaults       # Default Kconfig values
├── storage/                 # SPIFFS content
│   └── bye.bin              # Firmware binary to upload
├── main/
│   ├── CMakeLists.txt       # Component build file
│   ├── Kconfig.projbuild    # Menuconfig options
│   └── master_main.c        # Main application (SDO client, uploader task)
└── README.md (this file)
```

## Dependencies

- ESP-IDF v5.0.1 or later
- CANopenNode library
- CAN transceiver connected to TX/RX GPIO pins

## Hardware Requirements

- ESP32 development board
- CAN transceiver (e.g., SN65HVD230, TJA1050)
- CAN bus connected to slave device
- 120Ω termination resistors at bus ends

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "Cannot open firmware file" | Wrong path or missing file | Check `storage/bye.bin` exists |
| "SDO upload failed" | Slave not responding | Check wiring, node ID, bitrate |
| "SPIFFS full error" | Binary too large | Increase partition size in `partitions.csv` |
| Transfer very slow | Using segmented SDO | Enable block transfer in CO_driver_target.h |
| CRC mismatch every boot | NVS not populated on slave | Complete one successful OTA |

## License

This project follows CANopenNode licensing terms.
