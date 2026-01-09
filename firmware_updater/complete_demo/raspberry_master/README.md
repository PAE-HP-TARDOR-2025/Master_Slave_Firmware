# Raspberry Pi CANopen Master Firmware Uploader

This directory contains a production-ready firmware uploader for Raspberry Pi that communicates with ESP32 CANopen slaves via SocketCAN.

## Overview

The Raspberry Pi master uploads firmware to ESP32 slaves using the CANopen SDO protocol. It includes intelligent CRC **and version** checking to skip unnecessary uploads when the slave already has the desired firmware.

## Features

- **SocketCAN Integration**: Uses Linux SocketCAN for CAN communication
- **Dual Check (CRC + Version)**: Skips upload only if **both** CRC and version match
- **SDO Protocol**: Implements CiA 301 SDO expedited and segmented transfers
- **Command Line Interface**: Easy to use from shell scripts and automation
- **Progress Reporting**: Real-time upload progress feedback
- **Force Upload Option**: Override CRC/version check when needed

## Files

| File | Description |
|------|-------------|
| `main_rpi_master.c` | Main application entry point and SDO wrapper implementations |
| `sdo_client.c/h` | Standalone SDO client using raw SocketCAN frames |
| `fw_master_update.c/h` | Core update logic (CRC computation, session management) |
| `rpi_can.c/h` | SocketCAN wrapper (open, send, receive, close) |
| `Makefile` | Build configuration for GCC |

## Building

### Prerequisites

```bash
# On Raspberry Pi OS / Debian / Ubuntu:
sudo apt-get update
sudo apt-get install build-essential can-utils
```

### Compile

```bash
cd raspberry_master
make
```

This produces `rpi_uploader` executable.

## Hardware Setup

### CAN Interface

The Raspberry Pi requires a CAN HAT or USB-CAN adapter. Common options:

- **MCP2515 CAN HAT**: Uses SPI, requires kernel overlay
- **Waveshare CAN HAT**: Similar to MCP2515
- **USB-CAN Adapter**: Shows up as `can0` or `slcan0`

### MCP2515 Setup (if using CAN HAT)

Add to `/boot/config.txt`:
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
dtoverlay=spi-bcm2835-overlay
```

Reboot after changes.

### Wiring

```
Raspberry Pi CAN HAT       ESP32 Slave
       CAN_H  ────────────  CAN_H (via transceiver)
       CAN_L  ────────────  CAN_L (via transceiver)
        GND   ────────────   GND
```

**Important**: Both ends need CAN transceivers (e.g., MCP2551, TJA1050, SN65HVD230).

## Usage

### 1. Configure CAN Interface

```bash
# Set bitrate (must match ESP32 slave - default 500 kbps)
sudo ip link set can0 type can bitrate 500000

# Bring interface up
sudo ip link set can0 up

# Verify
ip link show can0
```

### 2. Run Uploader

```bash
# Basic usage
./rpi_uploader firmware.bin

# Specify target node ID
./rpi_uploader firmware.bin -n 10

# Force upload (skip CRC check)
./rpi_uploader firmware.bin -f

# Use different CAN interface
./rpi_uploader firmware.bin -i can1

# All options
./rpi_uploader -h
```

### Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-n <id>` | Target slave node ID (1-127) | 10 |
| `-i <if>` | CAN interface name | can0 |
| `-b <kbps>` | CAN bitrate in kbps | 500 |
| `-v <ver>` | Firmware version (1-65535) | 1 |
| `-f` | Force upload (ignore CRC/version match) | off |
| `-h` | Show help | - |

### Example Session

```
$ ./rpi_uploader bye.bin -n 10 -v 2
[RPI-MASTER] ========================================
[RPI-MASTER] CANopen Firmware Uploader
[RPI-MASTER] ========================================
[RPI-MASTER] CAN Interface: can0 @ 500 kbps
[RPI-MASTER] Firmware file: bye.bin
[RPI-MASTER] Target node:   10 (0x0A)
[RPI-MASTER] Firmware version: 2
[RPI-MASTER] Force upload:  no
[RPI-MASTER] ========================================
[RPI-MASTER] Opening CAN interface can0
[RPI-CAN] Opened can0 (ifindex 3)
[FW-MASTER] Loaded 253120 bytes from bye.bin
[FW-MASTER] Local firmware CRC: 0x1A00
[RPI-MASTER] Querying slave CRC from node 10 (0x1F5B:01)
[RPI-MASTER] Slave running firmware CRC: 0x1A00
[RPI-MASTER] Querying slave version from node 10 (0x1F5C:01)
[RPI-MASTER] Slave running firmware version: 2
[FW-MASTER] Both CRC and version match - skipping upload
[RPI-MASTER] ========================================
[RPI-MASTER] Upload completed successfully!
```

## CANopen Object Dictionary

The uploader uses these CiA 302 firmware update objects:

| Index | Sub | Name | Direction | Description |
|-------|-----|------|-----------|-------------|
| 0x1F50 | 01 | Program Data | Download | Firmware binary data (segmented) |
| 0x1F51 | 01 | Program Control | Download | Start command: `[0x01, 0x00, 0x00]` |
| 0x1F57 | 01 | Flash Status | Download | Metadata: `[size:4][crc:2][type:1][bank:1][version:2]` (10 bytes) |
| 0x1F5A | 01 | Finalize | Download | Final CRC: `[crc_lo, crc_hi]` |
| 0x1F5B | 01 | Running CRC | Upload | Slave's current firmware CRC (2 bytes) |
| 0x1F5C | 01 | Running Version | Upload | Slave's current firmware version (2 bytes) |

## CRC-16 Algorithm

Both master and slave must use identical CRC computation:

```c
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;  // Initial value
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // Polynomial
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;  // No final XOR
}
```

**Key Properties**:
- Polynomial: 0x1021 (X.25/CCITT)
- Initial value: 0xFFFF
- Process MSB first
- No final inversion
- Little-endian when transmitted

## SDO Protocol Details

### COB-IDs

- **TX (Request)**: 0x600 + nodeId (Master → Slave)
- **RX (Response)**: 0x580 + nodeId (Slave → Master)

For node ID 10:
- TX COB-ID: 0x60A
- RX COB-ID: 0x58A

### Expedited Transfer (≤4 bytes)

Used for start command, finalize, CRC query.

```
Request:  [ccs | idx_lo | idx_hi | sub | d0 | d1 | d2 | d3]
Response: [scs | idx_lo | idx_hi | sub | d0 | d1 | d2 | d3]
```

### Segmented Transfer (>4 bytes)

Used for firmware data chunks.

```
Initiate Request:  [0x21 | idx_lo | idx_hi | sub | size (4 bytes)]
Initiate Response: [0x60 | idx_lo | idx_hi | sub | 0 | 0 | 0 | 0]

Segment Request:   [toggle<<4 | n<<1 | c | data (7 bytes)]
Segment Response:  [toggle<<4 | 0x20 | ...]
```

- `toggle`: Alternates 0,1,0,1... for each segment
- `n`: Number of empty bytes (7 - data_len)
- `c`: Last segment flag (1 = final)

## Troubleshooting

### CAN Interface Not Found

```
[RPI-ERROR ] Failed to open CAN interface.
```

**Solution**:
```bash
# Check interface exists
ip link show

# Bring up interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### SDO Timeout

```
[SDO-ERR] Timeout waiting for response
```

**Possible causes**:
- Slave not connected or powered
- CAN bitrate mismatch (must be 500 kbps)
- Incorrect node ID
- CAN bus termination issue (need 120Ω at each end)

### CRC Always Mismatches

If upload happens every time even with same firmware:
- Verify slave has NVS CRC storage (check slave version)
- After successful upload, slave saves CRC to NVS
- On first boot with new slave firmware, CRC is computed from flash (may differ)

### Permission Denied

```
[RPI-CAN-ERR] socket(PF_CAN) failed: Operation not permitted
```

**Solution**:
```bash
# Run as root or add capability
sudo ./rpi_uploader firmware.bin

# Or add CAP_NET_RAW capability
sudo setcap cap_net_raw+ep ./rpi_uploader
```

## Integration with Scripts

### Systemd Service

Create `/etc/systemd/system/firmware-updater.service`:

```ini
[Unit]
Description=CANopen Firmware Updater
After=network.target

[Service]
Type=oneshot
ExecStartPre=/sbin/ip link set can0 type can bitrate 500000
ExecStartPre=/sbin/ip link set can0 up
ExecStart=/usr/local/bin/rpi_uploader /opt/firmware/slave.bin -n 10
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### Cron Job

```bash
# Update firmware at 3am daily if new version available
0 3 * * * /usr/local/bin/rpi_uploader /opt/firmware/slave.bin -n 10 >> /var/log/fw-update.log 2>&1
```

## Comparison with ESP32 Master

| Feature | ESP32 Master | Raspberry Pi Master |
|---------|--------------|---------------------|
| CAN Driver | TWAI (ESP-IDF) | SocketCAN (Linux) |
| SDO Client | CANopenNode | Custom implementation |
| Storage | SPIFFS | Filesystem |
| CRC Algorithm | Same (0x1021, 0xFFFF) | Same |
| Version Check | Dual (CRC + Version) | Dual (CRC + Version) |
| Object Addresses | Same (0x1F50-0x1F5C) | Same |

Both implementations are fully compatible with the same slave.

## Author

Production implementation based on lessons learned from ESP32 development.

## License

See repository root for license information.
