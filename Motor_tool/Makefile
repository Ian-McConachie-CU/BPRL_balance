##############################################################################
# Motor_tool — standalone CAN motor test/setup firmware — ChibiOS Makefile
#
# Usage:
#   make                            (CubeOrangePlus — same board as BPRL_balance)
#   make flash PORT=/dev/ttyACM0   (Cube bootloader)
#   make flash-stlink               (ST-Link / OpenOCD)
#
# Shares the parent BPRL_balance project's ChibiOS tree, board files, and
# cfg/ (halconf.h/mcuconf.h/chconf.h) — same MCU, same clock tree, no need
# to duplicate them. Only the application sources below are unique to this
# tool.
#

##############################################################################
# Board selection — CubeOrangePlus (STM32H743ZI) only, same as BPRL_balance
#

BOARD ?= CubeOrangePlus

PARENT   := ..
BOARDDIR := $(PARENT)/boards/$(BOARD)

BOARD_UDEFS = -DSTM32H743xx

##############################################################################
# Flash / upload targets  (defined after 'all' so bare 'make' builds only)
#

PORT            ?= /dev/ttyACM0
UPLOAD_SCRIPT   := $(PARENT)/tools/flash_upload.py
OPENOCD_CFG     := -f interface/stlink.cfg -f target/stm32h7x.cfg

##############################################################################
# Build global options
#

ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
endif

ifeq ($(USE_COPT),)
  USE_COPT =
endif

ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

##############################################################################
# Architecture / project specific options
#

ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x800
endif

ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x800
endif

# STM32H7 has a double-precision FPU — always use hard float
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-d16
endif

##############################################################################
# Project, target, sources and paths
#

PROJECT = MOTOR_TOOL

MCU = cortex-m7

CHIBIOS  := $(PARENT)/third_party/ChibiOS
CONFDIR  := $(PARENT)/cfg
BUILDDIR := build
DEPDIR   := .dep

# ChibiOS includes — no FatFS (no SD card use in this tool)
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32H7xx/platform.mk
include $(BOARDDIR)/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk

# Application starts at 0x08020000 (after 128 KB CubeOrangePlus bootloader).
# Same linker script as BPRL_balance — same board, same bootloader.
LDSCRIPT = $(BOARDDIR)/STM32H743xI_app.ld

# C sources
CSRC = $(ALLCSRC) \
       $(BOARDDIR)/board.c \
       $(CHIBIOS)/os/various/syscalls.c

# C++ sources
CPPSRC = $(ALLCPPSRC) \
         main.cpp \
         src/CAN.cpp \
         src/RmdMotor.cpp \
         src/GimMotor.cpp \
         src/OdriveMotor.cpp \
         src/Imx5.cpp \
         src/CmdShell.cpp \
         src/usb_serial.cpp

ASMSRC  = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# Include paths: PARENT so "src/CAN.hpp"-style includes resolve the same way
# they do in BPRL_balance; . so this tool's own src/ headers resolve too.
# CONFDIR provides chconf.h/halconf.h/mcuconf.h; BOARDDIR provides board.h.
INCDIR = $(CONFDIR) $(BOARDDIR) . $(ALLINC)

CWARN   = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User defines
#

UDEFS   = $(BOARD_UDEFS) -DCHPRINTF_USE_FLOAT=1 $(UDEFS_EXTRA)
UADEFS  =
UINCDIR =
ULIBDIR =
ULIBS   = -lm

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

##############################################################################
# Upload targets (after ChibiOS rules so 'all' is the default target)
#

flash: all
	python3 $(UPLOAD_SCRIPT) --port $(PORT) build/$(PROJECT).bin

flash-stlink: all
	openocd $(OPENOCD_CFG) \
	    -c "program build/$(PROJECT).hex verify reset exit"

.PHONY: flash flash-stlink
