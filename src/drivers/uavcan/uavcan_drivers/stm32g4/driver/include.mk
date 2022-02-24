#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

LIBUAVCAN_STM32G4_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

LIBUAVCAN_STM32G4_SRC := $(shell find $(LIBUAVCAN_STM32G4_DIR)src -type f -name '*.cpp')

LIBUAVCAN_STM32G4_INC := $(LIBUAVCAN_STM32G4_DIR)include/
