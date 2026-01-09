# Makefile for Raspberry Pi CANopen Master Firmware Uploader
#
# Build:
#   make
#
# Clean:
#   make clean
#
# Install (optional):
#   sudo make install

CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c11
LDFLAGS = -lpthread

# Source files
SRCS = main_rpi_master.c \
       sdo_client.c \
       fw_master_update.c \
       rpi_can.c

# Object files
OBJS = $(SRCS:.c=.o)

# Target binary
TARGET = rpi_uploader

# Default target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

# Dependencies
main_rpi_master.o: main_rpi_master.c rpi_can.h sdo_client.h fw_master_update.h
sdo_client.o: sdo_client.c sdo_client.h rpi_can.h
fw_master_update.o: fw_master_update.c fw_master_update.h
rpi_can.o: rpi_can.c rpi_can.h

.PHONY: all clean install
