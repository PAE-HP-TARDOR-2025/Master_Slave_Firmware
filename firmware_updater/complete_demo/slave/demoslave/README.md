# ESP32 CANopen Slave with OTA Firmware Update

A complete ESP-IDF CANopen slave that receives firmware updates over CAN bus using CiA 302 protocol.

## ✅ Verified Working

- Responds to CRC queries on `0x1F5B:01`
- Responds to **firmware version** queries on `0x1F5C:01`
- Receives firmware chunks via SDO on `0x1F50:01`
- Stores verified CRC **and version** in NVS for reliable boot-time reporting
- Automatic reboot after successful OTA validation
- Prints greeting, CRC, **and version** every 5 seconds for visual verification

## Features

- **Full CANopen Slave** – SDO server with firmware update objects
- **OTA Support** – Dual partition layout for safe firmware updates
- **NVS CRC Storage** – Reliable CRC reporting after reboot
- **Configurable Greeting** – Build variants with different messages
- **Auto-Reboot** – 500ms delay after successful firmware validation

## Quick Start

### Build and Flash
```powershell
cd librarywithfunctions/slave/demoslave
idf.py build flash -p COM5 monitor
```

### Build Greeting Variants
```powershell
cd librarywithfunctions/slave

# Build with greeting and version (recommended)
python build_slave_bins.py --greeting "bye:Goodbye from slave:2"
# Creates artifacts/bye.bin with version 2

# Build multiple variants
python build_slave_bins.py --greeting "hello:Hello from slave:1" --greeting "bye:Goodbye from slave:2"
# Creates artifacts/hello.bin (v1) and artifacts/bye.bin (v2)
```

**Format:** `--greeting "name:text:version"` where:
- `name` – Output filename (e.g., `bye` → `bye.bin`)
- `text` – Greeting message displayed by slave
- `version` – Firmware version (1-65535), used by master to detect changes

## Configuration

Run `idf.py menuconfig` → **Demo Slave Configuration**:

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_DEMO_SLAVE_NODE_ID` | 10 | CANopen node ID |
| `CONFIG_DEMO_SLAVE_CAN_BITRATE_KBPS` | 500 | CAN bus bitrate |
| `CONFIG_DEMO_SLAVE_MAX_CHUNK_BYTES` | 256 | Max bytes per firmware chunk |
| `CONFIG_DEMO_SLAVE_MAX_IMAGE_BYTES` | 524288 | Max firmware image size |
| `CONFIG_DEMO_SLAVE_FW_VERSION` | 1 | Firmware version (1-65535) |

## Object Dictionary (Firmware Objects)

| Index | Sub | Name | Access | Size | Description |
|-------|-----|------|--------|------|-------------|
| `0x1F50` | 01 | Firmware Download | WO | Variable | Firmware data FIFO |
| `0x1F51` | 01 | Control | WO | 3 bytes | Start: `{0x01, 0x00, 0x00}` |
| `0x1F57` | 01 | Metadata | WO | 8 bytes | Size, CRC, type, bank |
| `0x1F5A` | 01 | Status/Finalize | RW | 2 bytes | Final CRC / status |
| `0x1F5B` | 01 | Running CRC | RO | 2 bytes | Current firmware CRC |
| `0x1F5C` | 01 | Running Version | RO | 2 bytes | Current firmware version |

## How It Works

### Boot Sequence
```
1. Initialize NVS flash
2. Try to load CRC from NVS key "fw_update/fw_crc"
3. Try to load version from NVS key "fw_update/fw_version"
4. If CRC not found: Compute from flash (fallback, less reliable)
5. If version not found: Use compile-time CONFIG_DEMO_SLAVE_FW_VERSION
6. Initialize CANopen stack (node ID 10)
7. Start greeting task (prints message + CRC + version every 5 seconds)
8. Respond to master CRC queries on 0x1F5B:01
9. Respond to master version queries on 0x1F5C:01
```

### Firmware Receive Process
```
┌─────────────────────────────────────────────────────────────┐
│ 1. Master writes metadata to 0x1F57:01 (10 bytes)          │
│    [size(4)|crc(2)|type(1)|bank(1)|version(2)] little-end  │
│    → Slave parses and stores expected size/CRC/version      │
├─────────────────────────────────────────────────────────────┤
│ 2. Master writes start command to 0x1F51:01 (3 bytes)      │
│    [0x01, 0x00, 0x00]                                       │
│    → Slave erases target OTA partition                      │
├─────────────────────────────────────────────────────────────┤
│ 3. Master streams data chunks to 0x1F50:01                 │
│    → Slave writes to OTA partition, computes running CRC   │
│    → Logs: "Chunk @offset accepted (X bytes, total Y/Z)"   │
├─────────────────────────────────────────────────────────────┤
│ 4. Master writes finalize to 0x1F5A:01 (2 bytes)           │
│    [CRC low, CRC high]                                      │
│    → Slave verifies running CRC matches                     │
│    → If match: Set boot partition, save CRC + version to NVS│
│    → Schedule reboot in 500ms                               │
├─────────────────────────────────────────────────────────────┤
│ 5. Slave reboots into new firmware                         │
│    → Reads verified CRC from NVS                            │
│    → Reports correct CRC on 0x1F5B:01 queries              │
└─────────────────────────────────────────────────────────────┘
```

## NVS CRC Storage

The slave stores verified CRC in NVS for reliable retrieval after reboot:

```c
#define FW_NVS_NAMESPACE "fw_update"
#define FW_NVS_KEY_CRC   "fw_crc"

// After successful finalize:
static void fw_save_crc_to_nvs(uint16_t crc) {
    nvs_handle_t handle;
    nvs_open(FW_NVS_NAMESPACE, NVS_READWRITE, &handle);
    nvs_set_u16(handle, FW_NVS_KEY_CRC, crc);
    nvs_commit(handle);
    nvs_close(handle);
}

// On boot:
static uint16_t fw_compute_running_firmware_crc(void) {
    uint16_t nvsCrc;
    if (fw_load_crc_from_nvs(&nvsCrc)) {
        return nvsCrc;  // Reliable!
    }
    // Fallback: compute from flash (less reliable)
    return compute_crc_from_partition();
}
```

**Why NVS?** Computing CRC from flash is problematic:
- Can't determine exact image size (only partition size)
- Reading stops at first `0xFF` run which may be valid data
- NVS stores the exact verified CRC after successful update

## CRC-16 Calculation

Both master and slave use identical CRC-16-CCITT:

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;  // Initial value
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);  // Polynomial
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

## Example Console Output

### Normal Boot (CRC and Version from NVS)
```
I (320) slave_main: CANopen slave starting
I (330) fw_server: Running firmware CRC from NVS: 0x1A00, version: 2
I (340) slave_main: CANopen slave node 10 ready at 500 kbps
I (5340) [SLAVE] Goodbye from slave (CRC: 0x1A00, version: 2)
I (10340) [SLAVE] Goodbye from slave (CRC: 0x1A00, version: 2)
```

### Master CRC Query
```
I (3017) CO_DRIVER: RX: id=0x60A dlc=8 data=[40 5B 1F 01 00 00 00 00]
I (3017) CO_DRIVER: TX: id=0x58A dlc=8 data=[4B 5B 1F 01 00 1A 00 00]
```

### Receiving Firmware Update
```
I (3057) fw_server: Metadata received: size=253120 crc=0x1A00 type=0 bank=1
I (3077) fw_server: Start command received, erasing partition
I (3097) fw_server: Chunk @0 accepted (256 bytes, total 256/253120)
I (3117) fw_server: Chunk @256 accepted (256 bytes, total 512/253120)
...
I (15097) fw_server: Finalize received, verifying CRC
I (15097) fw_server: CRC match! Setting boot partition
I (15107) fw_server: Saved firmware CRC 0x1A00 to NVS
I (15107) fw_server: Scheduling reboot in 500ms
```

## Partition Layout

Uses dual OTA partitions for safe updates:

```csv
# Name,     Type,  SubType,  Offset,   Size
nvs,        data,  nvs,      0x9000,   0x4000
otadata,    data,  ota,      0xd000,   0x2000
phy_init,   data,  phy,      0xf000,   0x1000
ota_0,      app,   ota_0,    0x20000,  0x120000  # ~1.1 MB
ota_1,      app,   ota_1,    0x140000, 0x120000  # ~1.1 MB
```

## File Structure

```
demoslave/
├── CMakeLists.txt           # Build configuration
├── sdkconfig                # Build settings
├── sdkconfig.defaults       # Default Kconfig values
├── canopennode/             # CANopenNode library
│   └── CO_driver_target.h   # SDO buffer/block transfer config
├── main/
│   ├── CMakeLists.txt       # Sources from parent folder
│   └── Kconfig.projbuild    # Menuconfig options
└── README.md (this file)

# Sources pulled from parent directory:
../slave_main.c              # Main application
../fw_update_server.[ch]     # OTA handler + NVS CRC storage
../fw_slave_update.[ch]      # Platform-neutral state machine
../OD.[ch]                   # Object Dictionary
```

## SDO Block Transfer (Optional Speed Boost)

For faster transfers, enable block transfer in `canopennode/CO_driver_target.h`:

```c
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_SDO_SRV_BLOCK)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 889  // 127 segments × 7 bytes
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE | CO_CONFIG_FIFO_CRC16_CCITT)
```

This provides 5-10x speed improvement over segmented transfer.

## Building Greeting Variants

Use the parent folder's build script:

```powershell
cd librarywithfunctions/slave
python build_slave_bins.py                           # Builds default hello/bye
python build_slave_bins.py --greeting "test:Test"    # Custom greeting
```

Output binaries are placed in `artifacts/`:
- `hello.bin` – "Hello from slave"
- `bye.bin` – "Goodbye from slave"

## Hardware Requirements

- ESP32 development board
- CAN transceiver (e.g., SN65HVD230, TJA1050)
- CAN bus connected to master device
- 120Ω termination resistors at bus ends

GPIO Configuration (default):
- **TX**: GPIO 5
- **RX**: GPIO 4

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| CRC always 0x0000 | NVS empty, flash computation failed | Complete one successful OTA |
| "Metadata rejected" | Size too large | Increase `CONFIG_DEMO_SLAVE_MAX_IMAGE_BYTES` |
| "CRC mismatch" on finalize | Data corruption | Check CAN wiring, try slower bitrate |
| Slave doesn't respond | Wrong node ID | Verify `CONFIG_DEMO_SLAVE_NODE_ID` matches master target |
| Very slow transfer | Using segmented SDO | Enable block transfer (see above) |

## Dependencies

- ESP-IDF v5.0.1 or later
- CANopenNode library (included in canopennode/)
- NVS flash component (for CRC storage)

## License

This project follows CANopenNode licensing terms.
