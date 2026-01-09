/*
 * SocketCAN implementation for Raspberry Pi.
 * Requires linux/can.h and a configured CAN interface (e.g., via ip link).
 */

#include "rpi_can.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define RPI_CAN_LOG(fmt, ...) printf("[RPI-CAN] " fmt, ##__VA_ARGS__)
#define RPI_CAN_ERR(fmt, ...) fprintf(stderr, "[RPI-CAN-ERR] " fmt, ##__VA_ARGS__)

int
rpi_can_init(const char* ifname, int bitrate_kbps) {
    (void)bitrate_kbps; /* Bitrate is configured externally via `ip link set can0 type can bitrate ...` */

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        RPI_CAN_ERR("socket(PF_CAN) failed: %s\n", strerror(errno));
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        RPI_CAN_ERR("ioctl SIOCGIFINDEX for %s failed: %s\n", ifname, strerror(errno));
        close(sock);
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RPI_CAN_ERR("bind to %s failed: %s\n", ifname, strerror(errno));
        close(sock);
        return -1;
    }

    RPI_CAN_LOG("Opened %s (ifindex %d)\n", ifname, ifr.ifr_ifindex);
    return sock;
}

bool
rpi_can_send(int sock, uint32_t id, const uint8_t* data, uint8_t len) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = id;
    frame.can_dlc = len > 8 ? 8 : len;
    if (data && len > 0) {
        memcpy(frame.data, data, frame.can_dlc);
    }

    ssize_t nbytes = write(sock, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RPI_CAN_ERR("write failed: %s\n", strerror(errno));
        return false;
    }
    return true;
}

int
rpi_can_recv(int sock, uint32_t* id, uint8_t* data, size_t max_len) {
    struct can_frame frame;
    ssize_t nbytes = read(sock, &frame, sizeof(frame));
    if (nbytes < 0) {
        RPI_CAN_ERR("read failed: %s\n", strerror(errno));
        return -1;
    }
    if (nbytes < (ssize_t)sizeof(frame)) {
        return 0;
    }
    if (id) {
        *id = frame.can_id & CAN_EFF_MASK;
    }
    size_t copy_len = frame.can_dlc;
    if (copy_len > max_len) {
        copy_len = max_len;
    }
    if (data && copy_len > 0) {
        memcpy(data, frame.data, copy_len);
    }
    return (int)frame.can_dlc;
}

void
rpi_can_close(int sock) {
    if (sock >= 0) {
        close(sock);
        RPI_CAN_LOG("Socket closed\n");
    }
}
