##############################################################################
# BPRL_Balance — 5-bar linkage wheeled biped — ChibiOS Makefile
#
# Usage:
#   make                            (CubeOrangePlus — only supported board)
#   make flash PORT=/dev/ttyACM0   (Cube bootloader)
#   make flash-stlink               (ST-Link / OpenOCD)
#
# Debug USB CDC:
#   make UDEFS_EXTRA=-DBPRL_DEBUG
#

##############################################################################
# Board selection — CubeOrangePlus (STM32H743ZI) only
#

BOARD ?= CubeOrangePlus

BOARDDIR := boards/$(BOARD)

BOARD_UDEFS = -DSTM32H743xx

##############################################################################
# Flash / upload targets  (defined after 'all' so bare 'make' builds only)
#

PORT            ?= /dev/ttyACM0
UPLOAD_SCRIPT   := tools/flash_upload.py
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

PROJECT = BPRL_BALANCE

MCU = cortex-m7

CHIBIOS  := third_party/ChibiOS
CONFDIR  := cfg
BUILDDIR := build
DEPDIR   := .dep

# ChibiOS includes
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32H7xx/platform.mk
include $(BOARDDIR)/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk

# Application starts at 0x08020000 (after 128 KB CubeOrangePlus bootloader).
LDSCRIPT = $(BOARDDIR)/STM32H743xI_app.ld

# C sources
CSRC = $(ALLCSRC) \
       $(BOARDDIR)/board.c \
       $(CHIBIOS)/os/various/syscalls.c

# C++ sources
CPPSRC = $(ALLCPPSRC) \
         main.cpp \
         src/threads.cpp \
         src/math/math.cpp \
         src/state_estimator/EKF.cpp \
         src/state_estimator/StateManager.cpp \
         src/controllers/PID.cpp \
         src/controllers/RobotStateMachine.cpp \
         src/controllers/BalanceController.cpp \
         src/coms/IMUs/ICM42688.cpp \
         src/coms/IMUs/ICM45686.cpp \
         src/coms/SPI.cpp \
         src/coms/CAN.cpp \
         src/coms/CANMotor.cpp \
         src/coms/CANPower.cpp \
         src/coms/CalFlash.cpp \
         src/coms/Radio.cpp \
         src/coms/SBUS.cpp \
         src/usb_serial.cpp \
         src/logging/Logger.cpp

ASMSRC  = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# Include paths: cfg/ provides chconf.h, halconf.h, mcuconf.h
# BOARDDIR provides board.h
INCDIR = $(CONFDIR) $(BOARDDIR) $(ALLINC)

CWARN   = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User defines
#

# Board-specific MCU variant + optional debug flag
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
