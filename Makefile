TARGET=CY8CPROTO-062-4343W
APPNAME=ouroboros
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=1

COMPONENTS=design
DISABLE_COMPONENTS=BSP_DESIGN_MODUS
CY_BASELIB_PATH=libs/psoc6make

CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

CY_TOOLS_PATHS+=
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

PHONY: all
all:
	make program TARGET=CY8CPROTO-062-4343W TOOLCHAIN=GCC_ARM

include $(CY_TOOLS_DIR)/make/start.mk
