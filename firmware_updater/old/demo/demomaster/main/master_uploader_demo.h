#ifndef MASTER_UPLOADER_DEMO_H
#define MASTER_UPLOADER_DEMO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "CO_SDOclient.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FW_IMAGE_MAIN = 0,
    FW_IMAGE_BOOTLOADER = 1,
    FW_IMAGE_CONFIG = 2
} fw_image_type_t;

typedef struct {
    const char *firmwarePath;
    fw_image_type_t type;
    uint8_t targetBank;
    uint8_t targetNodeId;
    uint32_t maxChunkBytes;
    uint16_t expectedCrc;
} fw_upload_plan_t;

bool fw_master_bind_sdo_client(CO_SDOclient_t *client);
bool fw_run_upload_session(const fw_upload_plan_t *plan);

#ifdef __cplusplus
}
#endif

#endif /* MASTER_UPLOADER_DEMO_H */
