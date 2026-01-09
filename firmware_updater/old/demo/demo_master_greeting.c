/*
 * Desktop helper that inspects two firmware binaries and explains how the user-facing
 * greeting will change after the update. It does not talk to hardware; instead it lets
 * you verify that the "hello" firmware and the "not hello" firmware actually contain
 * different greetings before you stream the new image with master_firmware_uploader.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GREETING_TAG "GREETING:"

static const uint8_t*
find_pattern(const uint8_t* haystack, size_t haystackLen, const uint8_t* needle, size_t needleLen) {
    if (needleLen == 0 || haystackLen < needleLen) {
        return NULL;
    }
    for (size_t i = 0; i <= haystackLen - needleLen; i++) {
        if (memcmp(&haystack[i], needle, needleLen) == 0) {
            return &haystack[i];
        }
    }
    return NULL;
}

static bool
extract_greeting(const char* path, char* greeting, size_t greetingLen) {
    FILE* f = fopen(path, "rb");
    if (f == NULL) {
        fprintf(stderr, "[MASTER] Unable to open %s\n", path);
        return false;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        fprintf(stderr, "[MASTER] Seek failed for %s\n", path);
        return false;
    }
    long fileSize = ftell(f);
    if (fileSize <= 0) {
        fclose(f);
        fprintf(stderr, "[MASTER] File %s is empty\n", path);
        return false;
    }
    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        fprintf(stderr, "[MASTER] Rewind failed for %s\n", path);
        return false;
    }

    uint8_t* buffer = (uint8_t*)malloc((size_t)fileSize);
    if (buffer == NULL) {
        fclose(f);
        fprintf(stderr, "[MASTER] Out of memory reading %s\n", path);
        return false;
    }
    size_t readBytes = fread(buffer, 1, (size_t)fileSize, f);
    fclose(f);
    if (readBytes != (size_t)fileSize) {
        free(buffer);
        fprintf(stderr, "[MASTER] Short read on %s\n", path);
        return false;
    }

    const uint8_t* tag = find_pattern(buffer, readBytes, (const uint8_t*)GREETING_TAG, strlen(GREETING_TAG));
    if (tag == NULL) {
        free(buffer);
        fprintf(stderr, "[MASTER] Could not find %s marker in %s\n", GREETING_TAG, path);
        return false;
    }

    const char* start = (const char*)(tag + strlen(GREETING_TAG));
    size_t maxCopy = greetingLen - 1;
    size_t count = 0;
    while (count < maxCopy && (size_t)(start - (const char*)buffer) + count < readBytes) {
        char c = start[count];
        greeting[count] = c;
        if (c == '\0') {
            break;
        }
        count++;
    }
    greeting[greetingLen - 1] = '\0';
    free(buffer);

    if (count == maxCopy && greeting[maxCopy] != '\0') {
        fprintf(stderr, "[MASTER] Greeting in %s is too long\n", path);
        return false;
    }
    return true;
}

int
main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: demo_master_greeting <hello.bin> <not_hello.bin>\n");
        fprintf(stderr, "Build both images from dummy_slave_main.c using different SLAVE_GREETING values.\n");
        return -1;
    }

    char oldGreeting[128];
    char newGreeting[128];
    if (!extract_greeting(argv[1], oldGreeting, sizeof(oldGreeting))) {
        return -1;
    }
    if (!extract_greeting(argv[2], newGreeting, sizeof(newGreeting))) {
        return -1;
    }

    printf("[MASTER] Current firmware greeting : %s\n", oldGreeting);
    printf("[MASTER] Target firmware greeting  : %s\n", newGreeting);
    printf("[MASTER] Action: upload %s to stop the slave from saying \"%s\"\n", argv[2], oldGreeting);
    printf("[MASTER] Hint: master_firmware_uploader %s <nodeId> <bank>\n", argv[2]);
    return 0;
}
