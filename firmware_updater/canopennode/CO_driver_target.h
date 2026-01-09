/* components/canopennode/CO_driver_target.h */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* Enable SDO block transfer for faster firmware updates */
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_SDO_SRV_BLOCK | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 889  /* 127 segments * 7 bytes for block transfer */

/* Enable SDO client block transfer for master */
#define CO_CONFIG_SDO_CLI (CO_CONFIG_SDO_CLI_ENABLE | CO_CONFIG_SDO_CLI_SEGMENTED | CO_CONFIG_SDO_CLI_BLOCK | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#define CO_CONFIG_SDO_CLI_BUFFER_SIZE 889  /* 127 segments * 7 bytes for block transfer */

/* Enable FIFO for block transfer */
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE | CO_CONFIG_FIFO_ALT_READ | CO_CONFIG_FIFO_CRC16_CCITT)

#ifdef __cplusplus
extern "C" {
#endif

// 1. Arquitectura
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

// 2. Tipos
typedef uint8_t         bool_t;
typedef float           float32_t;
typedef double          float64_t;

// 3. ESTRUCTURA DEL MENSAJE (¡AQUÍ ES DONDE DEBE ESTAR!)
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

// 4. Macros de acceso (Ahora sí funcionarán)
#define CO_CANrxMsg_readIdent(msg) (((CO_CANrxMsg_t *)msg)->ident)
#define CO_CANrxMsg_readDLC(msg)   (((CO_CANrxMsg_t *)msg)->DLC)
#define CO_CANrxMsg_readData(msg)  (((CO_CANrxMsg_t *)msg)->data)

// 5. Estructuras estándar del driver
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

typedef struct {
    void* CANptr;
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
} CO_CANmodule_t;

typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    void* addrNV;
} CO_storage_entry_t;

// 6. Bloqueos
#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

#define CO_PROGMEM
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) { CO_MemoryBarrier(); rxNew = (void*)1L; }
#define CO_FLAG_CLEAR(rxNew) { CO_MemoryBarrier(); rxNew = NULL; }

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* CO_DRIVER_TARGET_H */