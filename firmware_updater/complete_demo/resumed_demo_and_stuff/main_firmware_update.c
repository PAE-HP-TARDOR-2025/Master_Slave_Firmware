/*
 * Demonstration CANopen firmware update mainline with verbose diagnostics.
 *
 * This dummy program shows how a CiA 302 style firmware update session could
 * be orchestrated on top of CANopenNode.  It is meant for bring-up on ESP-IDF
 * (or any other platform) where serial logging is available, so the code is
 * intentionally chatty and loaded with runtime validation.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "CANopen.h"
#include "OD.h"
#include "CO_storageBlank.h"

#define log_printf(fmt, ...) printf("[FW-DEMO] " fmt, ##__VA_ARGS__)
#define log_error(fmt, ...)  printf("[FW-ERR ] " fmt, ##__VA_ARGS__)
#define log_warn(fmt, ...)   printf("[FW-WARN] " fmt, ##__VA_ARGS__)

#define RETURN_IF_FALSE(cond, msg, ...)                                                                                \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            log_error(msg "\n", ##__VA_ARGS__);                                                                       \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

/* Default CANopen configuration mirrors the blank example */
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME        500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK        false
#define OD_STATUS_BITS       NULL

/* Object Dictionary helpers for the pseudo firmware update objects */
#define FW_CTRL_INDEX   0x1F51U /* CiA 302-3 firmware download control */
#define FW_META_INDEX   0x1F57U /* Firmware metadata (size, CRC, etc.) */
#define FW_BANK_INDEX   0x1F5AU /* Vendor-specific bank status */

#define FW_MAX_IMAGE_SIZE_BYTES (1024U * 512U)
#define FW_CHUNK_SIZE_BYTES     64U

typedef enum {
    FW_STAGE_IDLE = 0,
    FW_STAGE_METADATA_READY,
    FW_STAGE_ERASING_FLASH,
    FW_STAGE_RECEIVING_BLOCKS,
    FW_STAGE_VERIFYING,
    FW_STAGE_READY_TO_BOOT
} fw_stage_t;

typedef struct {
    fw_stage_t stage;
    uint32_t expectedSize;
    uint32_t receivedBytes;
    uint16_t expectedCrc;
    uint16_t runningCrc;
    uint8_t currentBank;
    bool metadataReceived;
    bool flashPrepared;
    bool crcMatched;
} fw_update_context_t;

static CO_t* CO = NULL;
static fw_update_context_t fwCtx;
static uint8_t LED_red, LED_green;

/* Reset the firmware state machine before a new download attempt. */
static void
fw_reset_context(fw_update_context_t* ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->stage = FW_STAGE_IDLE;
    ctx->currentBank = 0U;
}

/* Run one step of the CRC-16 calculation for inbound data bytes. */
static uint16_t
fw_crc16_step(uint16_t seed, uint8_t data) {
    seed ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++) {
        if (seed & 0x8000U) {
            seed = (uint16_t)((seed << 1) ^ 0x1021U);
        } else {
            seed <<= 1;
        }
    }
    return seed;
}

/* Simulate flash erase and mark the state machine as ready for chunk reception. */
static bool
fw_prepare_storage(fw_update_context_t* ctx) {
    log_printf("Preparing flash bank %u for new image...\n", ctx->currentBank);
    RETURN_IF_FALSE(ctx->stage == FW_STAGE_METADATA_READY, "Cannot erase flash before metadata step");

    ctx->flashPrepared = true;
    ctx->stage = FW_STAGE_ERASING_FLASH;
    log_printf("Flash bank %u erased successfully (simulated).\n", ctx->currentBank);
    ctx->stage = FW_STAGE_RECEIVING_BLOCKS;
    return true;
}

/* Validate and store incoming metadata record issued by the master. */
static bool
fw_store_metadata(fw_update_context_t* ctx, uint32_t size, uint16_t crc, uint8_t bank) {
    log_printf("Received metadata: size=%lu crc=0x%04X bank=%u\n", (unsigned long)size, crc, bank);

    RETURN_IF_FALSE(size > 0U, "Metadata rejected: size is zero");
    RETURN_IF_FALSE(size <= FW_MAX_IMAGE_SIZE_BYTES, "Metadata rejected: size %lu exceeds limit", (unsigned long)size);
    RETURN_IF_FALSE(crc != 0x0000U, "Metadata rejected: CRC cannot be zero");

    ctx->expectedSize = size;
    ctx->expectedCrc = crc;
    ctx->currentBank = bank;
    ctx->stage = FW_STAGE_METADATA_READY;
    ctx->metadataReceived = true;
    ctx->runningCrc = 0xFFFFU;
    log_printf("Metadata accepted; expecting %lu bytes.\n", (unsigned long)ctx->expectedSize);
    return true;
}

/* Accept one data chunk from the master while maintaining running CRC and offsets. */
static bool
fw_receive_chunk(fw_update_context_t* ctx, const uint8_t* data, uint32_t len, uint32_t offset) {
    RETURN_IF_FALSE(ctx->stage == FW_STAGE_RECEIVING_BLOCKS, "Chunk rejected: wrong stage %d", ctx->stage);
    RETURN_IF_FALSE(ctx->flashPrepared, "Chunk rejected: flash not prepared");
    RETURN_IF_FALSE(data != NULL, "Chunk rejected: NULL pointer");
    RETURN_IF_FALSE(len > 0U, "Chunk rejected: length zero");
    RETURN_IF_FALSE(offset == ctx->receivedBytes, "Chunk rejected: expected offset %lu got %lu",
                    (unsigned long)ctx->receivedBytes, (unsigned long)offset);

    ctx->receivedBytes += len;

    for (uint32_t i = 0; i < len; i++) {
        ctx->runningCrc = fw_crc16_step(ctx->runningCrc, data[i]);
    }

    log_printf("Chunk @%lu (%u bytes) accepted; total=%lu/%lu\n", (unsigned long)offset, (unsigned)len,
               (unsigned long)ctx->receivedBytes, (unsigned long)ctx->expectedSize);

    RETURN_IF_FALSE(ctx->receivedBytes <= ctx->expectedSize, "Overflow: received %lu > expected %lu",
                    (unsigned long)ctx->receivedBytes, (unsigned long)ctx->expectedSize);
    return true;
}

/* Verify total size and CRC, then mark the image as ready to boot. */
static bool
fw_finalize(fw_update_context_t* ctx) {
    RETURN_IF_FALSE(ctx->receivedBytes == ctx->expectedSize, "Finalize refused: size mismatch");
    ctx->stage = FW_STAGE_VERIFYING;

    ctx->crcMatched = (ctx->runningCrc == ctx->expectedCrc);
    if (!ctx->crcMatched) {
        log_error("CRC mismatch: computed 0x%04X expected 0x%04X\n", ctx->runningCrc, ctx->expectedCrc);
        return false;
    }

    ctx->stage = FW_STAGE_READY_TO_BOOT;
    log_printf("CRC validated (0x%04X). Image ready in bank %u\n", ctx->runningCrc, ctx->currentBank);
    return true;
}

/* Print every key field so the operator can inspect current progress. */
static void
fw_dump_context(const fw_update_context_t* ctx) {
    log_printf("--- Firmware context snapshot ---\n");
    log_printf(" stage          : %d\n", ctx->stage);
    log_printf(" metadata ready : %s\n", ctx->metadataReceived ? "yes" : "no");
    log_printf(" flash prepared : %s\n", ctx->flashPrepared ? "yes" : "no");
    log_printf(" expected size  : %lu bytes\n", (unsigned long)ctx->expectedSize);
    log_printf(" received bytes : %lu bytes\n", (unsigned long)ctx->receivedBytes);
    log_printf(" expected crc   : 0x%04X\n", ctx->expectedCrc);
    log_printf(" running crc    : 0x%04X\n", ctx->runningCrc);
    log_printf(" crc matched    : %s\n", ctx->crcMatched ? "yes" : "no");
    log_printf("----------------------------------\n");
}

/* Drive a scripted end-to-end update to exercise all guardrails without hardware. */
static void
fw_demo_session(CO_t* co) {
    (void)co;
    fw_reset_context(&fwCtx);

    /* First, demonstrate that invalid metadata is rejected */
    if (!fw_store_metadata(&fwCtx, 0U, 0x1234U, 0U)) {
        log_warn("As expected, metadata validation prevented the update. Retrying with sane values...\n");
    }

    const uint32_t imageSize = 512U;
    const uint16_t expectedCrc = 0x9C21U;
    if (!fw_store_metadata(&fwCtx, imageSize, expectedCrc, 1U)) {
        log_error("Unable to register valid metadata; aborting demo.\n");
        return;
    }
    if (!fw_prepare_storage(&fwCtx)) {
        log_error("Failed to prepare flash; aborting demo.\n");
        return;
    }

    uint8_t pattern[FW_CHUNK_SIZE_BYTES];
    for (uint32_t chunk = 0; chunk < imageSize; chunk += FW_CHUNK_SIZE_BYTES) {
        uint32_t remaining = imageSize - chunk;
        uint32_t len = remaining < FW_CHUNK_SIZE_BYTES ? remaining : FW_CHUNK_SIZE_BYTES;
        for (uint32_t i = 0; i < len; i++) {
            pattern[i] = (uint8_t)((chunk + i) & 0xFFU);
        }
        if (!fw_receive_chunk(&fwCtx, pattern, len, chunk)) {
            log_error("Chunk processing failed at offset %lu\n", (unsigned long)chunk);
            return;
        }
    }

    if (fw_finalize(&fwCtx)) {
        log_printf("Firmware image accepted; scheduling CANopen controlled reboot.\n");
    } else {
        log_error("Firmware demo failed during final verification.\n");
    }

    fw_dump_context(&fwCtx);
}

/* Entry point that wires the Controller Area Network Open stack and launches the demo session. */
int
main(void) {
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    void* CANptr = NULL;
    uint8_t pendingNodeId = 10;
    uint8_t activeNodeId = 10;
    uint16_t pendingBitRate = 125;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;
    CO_storage_entry_t storageEntries[] = {{.addr = &OD_PERSIST_COMM,
                                            .len = sizeof(OD_PERSIST_COMM),
                                            .subIndexOD = 2,
                                            .attr = CO_storage_cmd | CO_storage_restore,
                                            .addrNV = NULL}};
    const uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0U;
#endif

    CO_config_t* config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config);
    co_config.CNT_LEDS = 1;
    co_config.CNT_LSS_SLV = 1;
    config_ptr = &co_config;
#endif

    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_error("Memory allocation for CANopen failed\n");
        return -1;
    }
    log_printf("Reserved %lu bytes for CANopen objects\n", (unsigned long)heapMemoryUsed);

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);
    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_error("Storage init error %lu\n", (unsigned long)storageInitError);
        return -1;
    }
#endif

    while (reset != CO_RESET_APP) {
        log_printf("--- CANopen communication reset requested ---\n");
        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode((void*)&CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        err = CO_CANinit(CO, CANptr, pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_error("CO_CANinit failed (%d)\n", err);
            break;
        }

        CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                    .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                    .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                    .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};
        err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_error("CO_LSSinit failed (%d)\n", err);
            break;
        }

        activeNodeId = pendingNodeId;
        uint32_t errInfo = 0U;
        err = CO_CANopenInit(CO, NULL, NULL, OD, OD_STATUS_BITS, NMT_CONTROL, FIRST_HB_TIME, SDO_SRV_TIMEOUT_TIME,
                             SDO_CLI_TIMEOUT_TIME, SDO_CLI_BLOCK, activeNodeId, &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            log_error("CO_CANopenInit failed (%d) info=0x%lX\n", err, (unsigned long)errInfo);
            break;
        }

        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            log_error("CO_CANopenInitPDO failed (%d) info=0x%lX\n", err, (unsigned long)errInfo);
            break;
        }

        if (!CO->nodeIdUnconfigured) {
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if (storageInitError != 0U) {
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
            }
#endif
        } else {
            log_warn("Node ID still unconfigured; firmware demo will use pending defaults.\n");
        }

        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        log_printf("CANopen stack is running; launching firmware update drill.\n");

        fw_demo_session(CO);

        while (reset == CO_RESET_NOT) {
            const uint32_t timeDifference_us = 500U;
            reset = CO_process(CO, false, timeDifference_us, NULL);
            LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
            LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

            /* Periodic heartbeat over serial so the operator sees progress */
            static uint32_t hbPrintCountdown = 0U;
            if (++hbPrintCountdown >= 1000U) {
                hbPrintCountdown = 0U;
                log_printf("HB tick | LEDs R:%u G:%u | NMT=%u | errReg=0x%02X\n", LED_red, LED_green,
                           CO->NMT->operatingState, CO->em->errorRegister);
            }
        }
    }

    CO_CANsetConfigurationMode((void*)&CANptr);
    CO_delete(CO);
    log_printf("Firmware update demo finished.\n");
    return 0;
}

/* Timer handler that keeps synchronization and process data objects running. */
void
tmrTask_thread(void) {
    for (;;) {
        CO_LOCK_OD(CO->CANmodule);
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;
            const uint32_t timeDifference_us = 1000U;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif
        }
        CO_UNLOCK_OD(CO->CANmodule);
    }
}

/* Placeholder for the real Controller Area Network interrupt handler. */
void
CO_CAN1InterruptHandler(void) {
    /* Interrupt body intentionally blank for demo */
}
