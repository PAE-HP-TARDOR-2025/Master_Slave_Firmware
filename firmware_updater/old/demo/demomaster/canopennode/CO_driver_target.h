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