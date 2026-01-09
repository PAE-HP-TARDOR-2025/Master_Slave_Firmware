# CANopen Firmware Update Library

A complete ESP-IDF CANopen firmware update system enabling over-the-air (OTA) updates between a master controller and slave devices over CAN bus.

## ✅ Verified Working

This implementation has been tested and confirmed working:
- **Master queries slave CRC** via SDO upload on `0x1F5B:01`
- **Master queries slave version** via SDO upload on `0x1F5C:01`
- **Dual check works** – upload is skipped only when **both** CRC and version match
- **Full OTA transfer works** – streaming firmware via SDO segmented transfer
- **NVS persistence** – slave stores CRC and version in NVS for reliable reporting after reboot

## Architecture Overview

```
┌─────────────────┐         CAN Bus (500 kbps)        ┌─────────────────┐
│   ESP32 Master  │◄──────────────────────────────────►│   ESP32 Slave   │
│   (Node ID: 1)  │                                   │   (Node ID: 10) │
│                 │                                   │                 │
│  ┌───────────┐  │   1. Query CRC (0x1F5B:01)       │  ┌───────────┐  │
│  │   SPIFFS  │  │   2. Query Version (0x1F5C:01)   │  │    NVS    │  │
│  │  bye.bin  │  │   ◄─────────────────────────────►│  │CRC + ver  │  │
│  └───────────┘  │   3. If either differs, send:    │  └───────────┘  │
│                 │      - Metadata (0x1F57:01)       │                 │
│  SDO Client     │      - Start (0x1F51:01)          │  SDO Server     │
│                 │      - Data chunks (0x1F50:01)    │  OTA Handler    │
│                 │      - Finalize (0x1F5A:01)       │                 │
└─────────────────┘                                   └─────────────────┘
```

## Object Dictionary (CiA 302 Firmware Objects)

| Index | Sub | Name | Access | Size | Description |
|-------|-----|------|--------|------|-------------|
| `0x1F50` | 01 | Firmware Download | WO | Variable | Firmware data FIFO (write chunks here) |
| `0x1F51` | 01 | Control | WO | 3 bytes | Start token: `{0x01, 0x00, 0x00}` |
| `0x1F57` | 01 | Metadata | WO | 8 bytes | `[size(4) | crc(2) | type(1) | bank(1)]` LE |
| `0x1F5A` | 01 | Status/Finalize | RW | 2 bytes | Write: final CRC (LE), Read: status |
| `0x1F5B` | 01 | Running CRC | RO | 2 bytes | Currently running firmware CRC |
| `0x1F5C` | 01 | Running Version | RO | 2 bytes | Currently running firmware version |

### Message Formats (All Little-Endian)

**Metadata (0x1F57:01) - 10 bytes:**
```
Byte 0-3: Image size (uint32_t)
Byte 4-5: CRC-16 (uint16_t)  
Byte 6:   Image type (0 = main app)
Byte 7:   Target bank (1 = OTA slot)
Byte 8-9: Firmware version (uint16_t)
```

**Start Command (0x1F51:01) - 3 bytes:**
```
Byte 0: 0x01 (start token)
Byte 1: 0x00 (reserved)
Byte 2: 0x00 (reserved)
```

**Finalize (0x1F5A:01) - 2 bytes:**
```
Byte 0-1: Final CRC-16 (uint16_t, little-endian)
```

**Running CRC Response (0x1F5B:01) - 2 bytes:**
```
Byte 0-1: Running firmware CRC (uint16_t, little-endian)
```

## CRC-16 Calculation

The CRC uses **CRC-16-CCITT** polynomial `0x1021` with initial value `0xFFFF`:

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
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
```

**Important:** CRC is computed over the **entire firmware binary file**, not the flash partition.

## SDO Transfer Details

### Buffer Size Limitation
CANopenNode's default SDO buffer is only **32 bytes**. For large transfers:
- Write data progressively using `CO_SDOclientDownloadBufWrite()`
- Don't try to send entire firmware in one call
- Enable **SDO Block Transfer** for 5-10x faster speeds (requires buffer increase)

### SDO Block Transfer Configuration
In `CO_driver_target.h`:
```c
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_SDO_SRV_BLOCK)
#define CO_CONFIG_SDO_CLI (CO_CONFIG_SDO_CLI_SEGMENTED | CO_CONFIG_SDO_CLI_BLOCK | CO_CONFIG_SDO_CLI_LOCAL)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 889  /* 127 segments × 7 bytes */
#define CO_CONFIG_SDO_CLI_BUFFER_SIZE 889
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE | CO_CONFIG_FIFO_CRC16_CCITT)
```

### SDO Timeout
Use **3000ms** timeout for block transfers, **1000ms** for segmented.

## Firmware Update Process

### Master Side (Uploader)
```
1. Open firmware file from SPIFFS
2. Compute CRC-16 of entire file
3. Query slave's running CRC via SDO upload (0x1F5B:01)
4. Query slave's running version via SDO upload (0x1F5C:01)
5. If BOTH CRC and version match → Skip upload (firmware already current)
6. If EITHER differs → Proceed with upload:
   a. Send metadata (size, CRC, type, bank, version) → 0x1F57:01
   b. Send start command → 0x1F51:01  
   c. Stream firmware chunks → 0x1F50:01 (256 bytes each)
   d. Send finalize with CRC → 0x1F5A:01
7. Slave validates, sets boot partition, reboots
```

**Why dual check (CRC + version)?** CRC-16 has ~1/65536 collision probability. By also checking version, the master ensures firmware changes are detected even in rare CRC collisions.

### Slave Side (Receiver)
```
1. On boot: Read CRC and version from NVS (or fallback if not found)
2. Report CRC when master queries 0x1F5B:01
3. Report version when master queries 0x1F5C:01
4. When receiving firmware:
   a. Parse metadata from 0x1F57:01 (includes version)
   b. On start command (0x1F51:01): Erase OTA partition
   c. Receive chunks on 0x1F50:01, compute running CRC
   d. On finalize (0x1F5A:01): Verify CRC, set boot partition
   e. Save verified CRC and version to NVS
   f. Schedule reboot (500ms delay)
```

## NVS CRC and Version Storage

The slave stores the verified firmware CRC and version in NVS after successful OTA:

```c
#define FW_NVS_NAMESPACE "fw_update"
#define FW_NVS_KEY_CRC   "fw_crc"
#define FW_NVS_KEY_VER   "fw_version"

// After successful finalize:
nvs_set_u16(handle, FW_NVS_KEY_CRC, verified_crc);
nvs_set_u16(handle, FW_NVS_KEY_VER, firmware_version);
nvs_commit(handle);

// On boot, read from NVS first:
if (nvs_get_u16(handle, FW_NVS_KEY_CRC, &crc) == ESP_OK) {
    return crc;  // Use stored CRC
}
if (nvs_get_u16(handle, FW_NVS_KEY_VER, &version) == ESP_OK) {
    return version;  // Use stored version
}
// Fall back to compile-time defaults if not found
```

**Why NVS?** Computing CRC from flash is unreliable because:
- Flash reading stops at first `0xFF` run (could be valid data)
- Partition size doesn't equal actual image size
- Image size not easily accessible at runtime
- Version cannot be determined from binary content

## Folder Structure

```
librarywithfunctions/
├── master/
│   ├── OD.[ch]                  # Controller Object Dictionary
│   ├── fw_master_update.[ch]    # CRC-16, binary loading, CiA 302 helpers
│   └── demomaster/              # Complete ESP-IDF master project
│       ├── CMakeLists.txt
│       ├── partitions.csv       # 512KB SPIFFS + factory app
│       ├── sdkconfig.defaults
│       ├── storage/             # SPIFFS: place bye.bin here
│       │   └── bye.bin          # Firmware to upload
│       └── main/
│           ├── master_main.c    # Full SDO client implementation
│           ├── Kconfig.projbuild
│           └── CMakeLists.txt
├── slave/
│   ├── OD.[ch]                  # OTA-capable slave Object Dictionary
│   ├── fw_slave_update.[ch]     # Platform-neutral state machine
│   ├── fw_update_server.[ch]    # ESP-IDF OTA handler + NVS CRC storage
│   ├── slave_main.c             # Full CANopen slave template
│   ├── build_slave_bins.py      # Builds hello.bin/bye.bin variants
│   ├── artifacts/               # Generated binaries
│   └── demoslave/               # Complete ESP-IDF slave project
│       ├── CMakeLists.txt
│       ├── sdkconfig
│       ├── canopennode/         # CANopenNode with CO_driver_target.h
│       └── main/
│           ├── Kconfig.projbuild
│           └── CMakeLists.txt
├── raspberry_master/            # Linux/Raspberry Pi master implementation
│   └── raspberry_master_firmware_uploader.c
├── partitions/
│   ├── master_storage_spiffs.csv
│   └── slave_two_ota.csv
└── README.md (this file)
```

## Quick Start

### 1. Build Slave Firmware Variants
```powershell
cd librarywithfunctions/slave

# Build with greeting and version
python build_slave_bins.py --greeting "bye:Goodbye from slave:2"
# Creates artifacts/bye.bin with version 2

# Build multiple variants
python build_slave_bins.py --greeting "hello:Hello from slave:1" --greeting "bye:Goodbye from slave:2"
```

**Format:** `--greeting "name:text:version"` where:
- `name` – Output filename (e.g., `bye` → `bye.bin`)
- `text` – Greeting message displayed by slave
- `version` – Firmware version (1-65535), used by master to detect changes

### 2. Prepare Master SPIFFS
```powershell
# Copy firmware to master storage
cp artifacts/bye.bin ../master/demomaster/storage/bye.bin
```

### 3. Build and Flash Slave (COM5)
```powershell
cd librarywithfunctions/slave/demoslave
idf.py build flash -p COM5 monitor
```

### 4. Build and Flash Master (COM6)
```powershell
cd librarywithfunctions/master/demomaster
idf.py build flash -p COM6 monitor
```

### 5. Observe
- Master queries slave CRC
- If different: firmware transfer proceeds
- If same: "Slave already has matching firmware, skipping upload"

## Configuration (Kconfig)

### Master (`main/Kconfig.projbuild`)
| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_MASTER_NODE_ID` | 1 | CANopen node ID |
| `CONFIG_MASTER_TARGET_NODE_ID` | 10 | Target slave node ID |
| `CONFIG_MASTER_TWAI_TX_GPIO` | 5 | CAN TX pin |
| `CONFIG_MASTER_TWAI_RX_GPIO` | 4 | CAN RX pin |
| `CONFIG_MASTER_FIRMWARE_PATH` | `/spiffs/bye.bin` | Firmware file path |
| `CONFIG_MASTER_SKIP_IF_CRC_MATCH` | y | Skip upload if CRC **and version** match |
| `CONFIG_MASTER_UPLOAD_ON_STARTUP` | y | Auto-upload on boot |
| `CONFIG_MASTER_FIRMWARE_VERSION` | 1 | Firmware version embedded in metadata |

### Slave (`main/Kconfig.projbuild`)
| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_DEMO_SLAVE_NODE_ID` | 10 | CANopen node ID |
| `CONFIG_DEMO_SLAVE_CAN_BITRATE_KBPS` | 500 | CAN bitrate |
| `CONFIG_DEMO_SLAVE_MAX_CHUNK_BYTES` | 256 | Max OTA chunk size |
| `CONFIG_DEMO_SLAVE_MAX_IMAGE_BYTES` | 524288 | Max firmware size |
| `CONFIG_DEMO_SLAVE_FW_VERSION` | 1 | Firmware version (1-65535) |

## Hardware Setup

```
ESP32 Master (COM6)          ESP32 Slave (COM5)
     GPIO 5 (TX) ─────────────── CAN_H ───────────────── GPIO 5 (TX)
     GPIO 4 (RX) ─────────────── CAN_L ───────────────── GPIO 4 (RX)
         GND ─────────────────── GND ─────────────────── GND

                    ┌─────────────────┐
                    │  CAN Transceiver │  (e.g., SN65HVD230)
                    │  on both sides   │
                    └─────────────────┘
                    
     120Ω termination resistor at each end of bus
```

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| CRC always mismatches | NVS not populated | Complete one successful OTA first |
| SDO timeout | Wrong node ID or wiring | Check CAN connections, node IDs |
| SPIFFS full error | Firmware too large | Increase partition size in `partitions.csv` |
| Slave doesn't respond | Wrong bitrate | Both must use 500 kbps |
| Transfer very slow | Using segmented SDO | Enable block transfer (see above) |

## Key Implementation Notes

### SDO Download Pattern (Master)
```c
// Setup for target node
CO_SDOclient_setup(sdoClient, 0x600 + nodeId, 0x580 + nodeId, nodeId);

// Initialize download
CO_SDOclientDownloadInitiate(sdoClient, index, subIndex, dataLen, timeout, block);

// Write data progressively (buffer is small!)
size_t offset = 0;
do {
    size_t written = CO_SDOclientDownloadBufWrite(sdoClient, data + offset, len - offset);
    offset += written;
    
    ret = CO_SDOclientDownload(sdoClient, timeDiff, false, offset < len, &abort, &transferred, NULL);
    if (ret == CO_SDO_RT_waitingResponse) vTaskDelay(1);
} while (ret > CO_SDO_RT_ok_communicationEnd);
```

### SDO Upload Pattern (Master)
```c
// Setup and initiate
CO_SDOclient_setup(sdoClient, 0x600 + nodeId, 0x580 + nodeId, nodeId);
CO_SDOclientUploadInitiate(sdoClient, index, subIndex, timeout, block);

// Process until complete
do {
    ret = CO_SDOclientUpload(sdoClient, timeDiff, false, &abort, &sizeInd, &transferred, NULL);
    if (ret == CO_SDO_RT_waitingResponse) vTaskDelay(1);
} while (ret > CO_SDO_RT_ok_communicationEnd);

// Read result
size_t dataSize = CO_SDOclientUploadBufRead(sdoClient, buffer, maxLen);
```

## Raspberry Pi / Linux Master

See `raspberry_master/raspberry_master_firmware_uploader.c` for a complete SocketCAN implementation that can run on Linux systems including Raspberry Pi.

## License

This firmware update library follows the CANopenNode licensing terms.
