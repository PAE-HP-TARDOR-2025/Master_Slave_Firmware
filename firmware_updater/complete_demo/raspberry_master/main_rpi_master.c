/**
 * @file main_rpi_master.c
 * @brief Raspberry Pi CANopen Master Firmware Uploader
 *
 * This application runs on a Raspberry Pi and uploads firmware to ESP32
 * CANopen slaves via SocketCAN. It implements CRC and version checking
 * to skip unnecessary uploads.
 *
 * Build:
 *   gcc -o rpi_uploader main_rpi_master.c sdo_client.c fw_master_update.c \
 *       rpi_can.c -lpthread
 *
 * Usage:
 *   sudo ip link set can0 type can bitrate 500000
 *   sudo ip link set can0 up
 *   ./rpi_uploader firmware.bin [node_id]
 *
 * Key Implementation Notes (Lessons Learned):
 * - CAN bitrate must match slave (default 500 kbps)
 * - Node ID 10 (0x0A) is the default slave
 * - Master uses Node ID 1
 * - CRC-16 CCITT with polynomial 0x1021, init 0xFFFF
 * - SDO timeout set to 3 seconds for reliability
 * - Metadata format: [size:4][crc:2][type:1][bank:1][version:2] = 10 bytes
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "rpi_can.h"
#include "sdo_client.h"
#include "fw_master_update.h"

#define LOG(fmt, ...) printf("[RPI-MASTER] " fmt, ##__VA_ARGS__)
#define ERR(fmt, ...) fprintf(stderr, "[RPI-ERROR ] " fmt, ##__VA_ARGS__)

/* Default configuration */
#define DEFAULT_CAN_INTERFACE  "can0"
#define DEFAULT_BITRATE_KBPS   500
#define DEFAULT_SLAVE_NODE_ID  10
#define DEFAULT_MASTER_NODE_ID 1
#define DEFAULT_CHUNK_SIZE     256

static int g_can_sock = -1;
static volatile bool g_running = true;

static void signal_handler(int sig) {
    (void)sig;
    g_running = false;
    LOG("Interrupt received, shutting down...\n");
}

/**
 * Print usage information
 */
static void print_usage(const char* progname) {
    printf("CANopen Firmware Uploader for Raspberry Pi\n\n");
    printf("Usage: %s <firmware.bin> [options]\n\n", progname);
    printf("Options:\n");
    printf("  -n <node_id>    Target slave node ID (default: %d)\n", DEFAULT_SLAVE_NODE_ID);
    printf("  -i <interface>  CAN interface name (default: %s)\n", DEFAULT_CAN_INTERFACE);
    printf("  -b <bitrate>    CAN bitrate in kbps (default: %d)\n", DEFAULT_BITRATE_KBPS);
    printf("  -v <version>    Firmware version number (default: 1)\n");
    printf("  -f              Force upload even if CRC and version match\n");
    printf("  -h              Show this help\n");
    printf("\nExample:\n");
    printf("  sudo ip link set can0 type can bitrate 500000\n");
    printf("  sudo ip link set can0 up\n");
    printf("  %s bye.bin -n 10 -v 2\n", progname);
}

/* ========================================================================= */
/* FW Master Update Interface Implementation                                 */
/* ========================================================================= */

/**
 * Send metadata to slave (Object 0x1F57:01)
 *
 * Metadata format (10 bytes):
 *   Bytes 0-3: Image size (uint32_t, little-endian)
 *   Bytes 4-5: CRC-16 (uint16_t, little-endian)
 *   Byte 6:    Image type (0x00 = main firmware)
 *   Byte 7:    Target bank (0x01)
 *   Bytes 8-9: Version (uint16_t, little-endian)
 */
bool fw_master_send_metadata(const fw_upload_plan_t* plan, const fw_payload_t* payload, uint16_t crc) {
    uint8_t meta[10];
    uint32_t size = (uint32_t)payload->size;
    
    /* Pack metadata in little-endian format */
    meta[0] = (uint8_t)(size & 0xFF);
    meta[1] = (uint8_t)((size >> 8) & 0xFF);
    meta[2] = (uint8_t)((size >> 16) & 0xFF);
    meta[3] = (uint8_t)((size >> 24) & 0xFF);
    meta[4] = (uint8_t)(crc & 0xFF);
    meta[5] = (uint8_t)((crc >> 8) & 0xFF);
    meta[6] = (uint8_t)plan->type;
    meta[7] = plan->targetBank;
    meta[8] = (uint8_t)(plan->firmwareVersion & 0xFF);
    meta[9] = (uint8_t)((plan->firmwareVersion >> 8) & 0xFF);
    
    LOG("Sending metadata: size=%u, crc=0x%04X, type=%u, bank=%u, version=%u\n",
        size, crc, plan->type, plan->targetBank, plan->firmwareVersion);
    
    return sdo_download(plan->targetNodeId, 0x1F57, 1, meta, sizeof(meta));
}

/**
 * Send start command to slave (Object 0x1F51:01)
 *
 * Start command format (3 bytes):
 *   Byte 0: Start token (0x01 = initiate OTA)
 *   Bytes 1-2: Reserved (0x00, 0x00)
 */
bool fw_master_send_start_command(const fw_upload_plan_t* plan) {
    uint8_t cmd[3] = { 0x01, 0x00, 0x00 };
    
    LOG("Sending start command to node %u\n", plan->targetNodeId);
    
    return sdo_download(plan->targetNodeId, 0x1F51, 1, cmd, sizeof(cmd));
}

/**
 * Send firmware chunk to slave (Object 0x1F50:01)
 *
 * Uses SDO segmented transfer for large chunks.
 */
bool fw_master_send_chunk(const fw_upload_plan_t* plan, const uint8_t* chunk, size_t len, size_t offset) {
    (void)offset;  /* Slave tracks position internally */
    
    return sdo_download(plan->targetNodeId, 0x1F50, 1, chunk, len);
}

/**
 * Send finalize request to slave (Object 0x1F5A:01)
 *
 * Finalize format (2 bytes):
 *   Bytes 0-1: Final CRC-16 (uint16_t, little-endian)
 */
bool fw_master_send_finalize_request(const fw_upload_plan_t* plan, uint16_t crc) {
    uint8_t finalize[2];
    finalize[0] = (uint8_t)(crc & 0xFF);
    finalize[1] = (uint8_t)((crc >> 8) & 0xFF);
    
    LOG("Sending finalize request with CRC 0x%04X\n", crc);
    
    return sdo_download(plan->targetNodeId, 0x1F5A, 1, finalize, sizeof(finalize));
}

/**
 * Query slave's running firmware CRC (Object 0x1F5B:01)
 *
 * Returns the CRC-16 of the currently running firmware on the slave.
 */
bool fw_master_query_slave_crc(const fw_upload_plan_t* plan, uint16_t* slaveCrc) {
    uint8_t buf[2] = {0};
    size_t actualLen = 0;
    
    LOG("Querying slave CRC from node %u (0x1F5B:01)\n", plan->targetNodeId);
    
    if (!sdo_upload(plan->targetNodeId, 0x1F5B, 1, buf, sizeof(buf), &actualLen)) {
        ERR("Failed to read slave CRC (abort code: 0x%08X)\n", sdo_get_last_abort_code());
        return false;
    }
    
    if (actualLen < 2) {
        ERR("Short read from 0x1F5B:01, expected 2 bytes, got %zu\n", actualLen);
        return false;
    }
    
    *slaveCrc = (uint16_t)(buf[0] | (buf[1] << 8));
    LOG("Slave running firmware CRC: 0x%04X\n", *slaveCrc);
    
    return true;
}

/**
 * Query slave's running firmware version (Object 0x1F5C:01)
 *
 * Returns the version number of the currently running firmware on the slave.
 */
bool fw_master_query_slave_version(const fw_upload_plan_t* plan, uint16_t* slaveVersion) {
    uint8_t buf[2] = {0};
    size_t actualLen = 0;
    
    LOG("Querying slave version from node %u (0x1F5C:01)\n", plan->targetNodeId);
    
    if (!sdo_upload(plan->targetNodeId, 0x1F5C, 1, buf, sizeof(buf), &actualLen)) {
        ERR("Failed to read slave version (abort code: 0x%08X)\n", sdo_get_last_abort_code());
        return false;
    }
    
    if (actualLen < 2) {
        ERR("Short read from 0x1F5C:01, expected 2 bytes, got %zu\n", actualLen);
        return false;
    }
    
    *slaveVersion = (uint16_t)(buf[0] | (buf[1] << 8));
    LOG("Slave running firmware version: %u\n", *slaveVersion);
    
    return true;
}

/* ========================================================================= */
/* Main Entry Point                                                          */
/* ========================================================================= */

int main(int argc, char** argv) {
    const char* canInterface = DEFAULT_CAN_INTERFACE;
    const char* firmwarePath = NULL;
    uint8_t slaveNodeId = DEFAULT_SLAVE_NODE_ID;
    int bitrateKbps = DEFAULT_BITRATE_KBPS;
    uint16_t firmwareVersion = 1;  /* Default version */
    bool forceUpload = false;
    
    /* Parse command line arguments */
    int opt;
    while ((opt = getopt(argc, argv, "n:i:b:v:fh")) != -1) {
        switch (opt) {
            case 'n':
                slaveNodeId = (uint8_t)atoi(optarg);
                break;
            case 'i':
                canInterface = optarg;
                break;
            case 'b':
                bitrateKbps = atoi(optarg);
                break;
            case 'v':
                firmwareVersion = (uint16_t)atoi(optarg);
                break;
            case 'f':
                forceUpload = true;
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }
    
    /* Get firmware path from remaining arguments */
    if (optind < argc) {
        firmwarePath = argv[optind];
    } else {
        ERR("No firmware file specified\n\n");
        print_usage(argv[0]);
        return 1;
    }
    
    /* Setup signal handler for graceful shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    LOG("========================================\n");
    LOG("CANopen Firmware Uploader\n");
    LOG("========================================\n");
    LOG("CAN Interface: %s @ %d kbps\n", canInterface, bitrateKbps);
    LOG("Firmware file: %s\n", firmwarePath);
    LOG("Target node:   %u (0x%02X)\n", slaveNodeId, slaveNodeId);
    LOG("FW Version:    %u\n", firmwareVersion);
    LOG("Force upload:  %s\n", forceUpload ? "yes" : "no");
    LOG("========================================\n");
    
    /* Initialize CAN interface */
    LOG("Opening CAN interface %s\n", canInterface);
    g_can_sock = rpi_can_init(canInterface, bitrateKbps);
    if (g_can_sock < 0) {
        ERR("Failed to open CAN interface. Make sure it's configured:\n");
        ERR("  sudo ip link set %s type can bitrate %d000\n", canInterface, bitrateKbps);
        ERR("  sudo ip link set %s up\n", canInterface);
        return 1;
    }
    
    /* Initialize SDO client */
    sdo_client_init(g_can_sock);
    
    /* Setup upload plan */
    fw_upload_plan_t plan = {
        .firmwarePath = firmwarePath,
        .type = FW_IMAGE_MAIN,
        .targetBank = 1,
        .targetNodeId = slaveNodeId,
        .maxChunkBytes = DEFAULT_CHUNK_SIZE,
        .expectedCrc = 0,  /* Will be computed from file */
        .firmwareVersion = firmwareVersion
    };
    
    /* Run the upload */
    bool success;
    if (forceUpload) {
        LOG("Force upload enabled, skipping CRC check\n");
        success = fw_master_run_upload_session(&plan);
    } else {
        success = fw_master_run_upload_if_needed(&plan);
    }
    
    /* Cleanup */
    rpi_can_close(g_can_sock);
    
    LOG("========================================\n");
    if (success) {
        LOG("Upload completed successfully!\n");
        LOG("Slave will reboot with new firmware.\n");
        return 0;
    } else {
        ERR("Upload failed!\n");
        return 1;
    }
}
