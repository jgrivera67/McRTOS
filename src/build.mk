#
# McRTOS application top-level build makefile
#
# Copyright (C) 2013 German Rivera
#
# NOTE: This Makefile must be run only from the obj directory
#
ifndef APPLICATION
    $(error APPLICATION not defined)
endif

ifndef PLATFORM
    $(error PLATFORM not defined)
endif

ifndef BUILD_FLAVOR
    $(error BUILD_FLAVOR not defined)
endif

BASE_DIR := $(dir $(CURDIR))
SOURCE_DIR := $(BASE_DIR)src
INCLUDE_DIR := $(BASE_DIR)inc
PROJECT_DIR := $(BASE_DIR)prj

#
# Check that this makefile is not being run from the source directory
#
$(if $(filter $(notdir $(SOURCE_DIR)), $(notdir $(CURDIR))), \
     $(error This makefile cannot be run from directory $(CURDIR)))

#
# Tools
#
TOOLCHAIN   ?= arm-none-eabi
CC          = $(TOOLCHAIN)-gcc
CPP         = $(TOOLCHAIN)-cpp
AS          = $(TOOLCHAIN)-gcc -x assembler-with-cpp
LD          = $(TOOLCHAIN)-ld
OBJCOPY     = $(TOOLCHAIN)-objcopy
OBJDUMP     = $(TOOLCHAIN)-objdump
AR          = $(TOOLCHAIN)-ar
RANLIB      = $(TOOLCHAIN)-ranlib

ifeq "$(PLATFORM)" "LPC2478-STK"
    SYSTEM_ON_CHIP = LPC2478_SOC
    CPU_ARCHITECTURE = armv4
    MCU  = arm7tdmi
    CODETYPE = arm
endif

ifeq "$(PLATFORM)" "LM4F120-LaunchPad"
    SYSTEM_ON_CHIP = LM4F120_SOC
    CPU_ARCHITECTURE = arm_cortex_m
    MCU  = cortex-m4
    CODETYPE = thumb
    # To enable code generation for hard FP:
    #EXTRA_MCFLAGS = -mfloat-abi=hard -mfpu=fpv4-sp-d16
endif

ifeq "$(PLATFORM)" "FRDM-KL25Z"
    SYSTEM_ON_CHIP = KL25Z_SOC
    CPU_ARCHITECTURE = arm_cortex_m
    MCU = cortex-m0plus
    CODETYPE = thumb
endif

ifeq "$(PLATFORM)" "FRDM-K20D5"
    SYSTEM_ON_CHIP = K20D5_SOC
    CPU_ARCHITECTURE = arm_cortex_m
    MCU  = cortex-m4
    CODETYPE = thumb
endif

ifeq "$(PLATFORM)" "FRDM-K64F"
    SYSTEM_ON_CHIP = K64F_SOC
    CPU_ARCHITECTURE = arm_cortex_m
    MCU  = cortex-m4
    CODETYPE = thumb
endif

ifndef CPU_ARCHITECTURE
    $(error unsupported platform $(PLATFORM))
endif

LDSCRIPT = $(PROJECT_DIR)/$(SYSTEM_ON_CHIP)-flash.ld


# $(call source-to-object, source-file-list)
source-to-object = $(subst .c,.o,$(filter %.c,$1)) \
		   $(subst .s,.o,$(filter %.s,$1))

# $(subdirectory)
subdirectory = $(patsubst $(SOURCE_DIR)/%/module.mk,%,	\
		 $(word					\
		   $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))

# $(call make-library, library-name, source-file-list)
define make-library
  libraries += $1
  sources   += $2

  $1: $(call source-to-object,$2)
	$(AR) $(ARFLAGS) $$@ $$^
endef

modules      := \
		McRTOS \
		BoardSupport \
		Applications/$(APPLICATION)

#
# NOTE: 'programs' is populated by included
# Applications/*/module.mk makefiles
#
programs     :=

#
# NOTE: 'libraries' is populated by make-library invocations and if
# necessary by included Applications/$(PLATFORM)/*/module.mk makefiles
#
libraries    :=

#
# NOTE: 'sources' is populated by make-library invocations
# and by included Applications/$(PLATFORM)/*/module.mk makefiles
#
sources	     :=

objects      = 	$(call source-to-object,$(sources))
dependencies = 	$(subst .o,.d,$(objects))

hex_files = 	$(subst .elf,.hex,$(programs))
bin_files = 	$(subst .elf,.bin,$(programs))
lst_files = 	$(subst .elf,.lst,$(programs))

include_dirs := $(INCLUDE_DIR)/McRTOS \
	        $(INCLUDE_DIR)/BoardSupport \
	        $(INCLUDE_DIR)/BoardSupport/$(PLATFORM) \

ifeq "$(CPU_ARCHITECTURE)" "arm_cortex_m"
    include_dirs += $(INCLUDE_DIR)/BoardSupport/CMSIS
endif

CPPFLAGS     += $(addprefix -I ,$(include_dirs)) \
		-D$(SYSTEM_ON_CHIP)

ifeq "$(BUILD_FLAVOR)" "debug"
    CPPFLAGS += -DDEBUG \
		-D_RELIABILITY_CHECKS_ \
		-D_BRANCH_MICRO_TRACING_ #\
		#-D_CPU_CYCLES_MEASURE_
    OPT = -O0
endif

ifeq "$(BUILD_FLAVOR)" "reliability"
    CPPFLAGS += -D_RELIABILITY_CHECKS_ #\
		#-D_CPU_CYCLES_MEASURE_
    OPT = -O0
endif

ifeq "$(BUILD_FLAVOR)" "performance"
    #OPT can be -O1, -O2, -Os or -O3
    OPT = -O2
endif

# optimisation level here -O0, -O1, -O2, -Os, or -03
MCFLAGS = 	-mcpu=$(MCU) -m$(CODETYPE) $(EXTRA_MCFLAGS)
ASFLAGS = 	$(MCFLAGS) -g -gdwarf-2 -Wa,-amhls=$(<:.s=.lst) \
		$(CPPFLAGS)
CFLAGS  = 	$(MCFLAGS) $(OPT) -gdwarf-2 -fomit-frame-pointer \
		-Wmissing-prototypes \
		-Wpointer-arith \
		-Winline \
		-fverbose-asm -Wa,-ahlms=$(<:.c=.lst) \
		-Werror \
		-Wstack-usage=112 \
		-Wundef \
		$(CPPFLAGS) \
		${COMMON_CFLAGS} ${CMD_LINE_CFLAGS}

LDFLAGS = 	$(MCFLAGS) -nostartfiles -T$(LDSCRIPT) \
		-Wl,-Map=$(PLATFORM).map,--cref,--no-warn-mismatch

vpath %.h $(include_dirs)
vpath %.c $(SOURCE_DIR)
vpath %.s $(SOURCE_DIR)

MKDIR := mkdir -p
MV    := mv -f
RM    := rm -f
SED   := sed
TEST  := test

create-output-directories :=				\
	$(shell for f in $(modules);			\
		do					\
		  $(TEST) -d $$f || $(MKDIR) $$f;	\
		done)

all:

include $(patsubst %,$(SOURCE_DIR)/%/module.mk,$(modules))

.PHONY: all
all: $(programs) $(hex_files) $(bin_files) $(lst_files)

.PHONY: libraries
libraries: $(libraries)

.PHONY: clean
clean:
	$(RM) -r *

ifneq "$(MAKECMDGOALS)" "clean"
  include $(dependencies)
endif

%o : %c
	$(CC) -c $(CFLAGS) $< -o $@
	$(MV) $(<:.c=.lst) $(@:.o=.lst)

%o : %s
	$(AS) -c $(ASFLAGS) $< -o $@
	$(MV) $(<:.s=.lst) $(@:.o=.lst)

GEN_HEADER_DEPENDENCIES = \
	($(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -M $< | \
	 $(SED) 's,\($(notdir $*)\.o\) *:,$(dir $@)\1 $@: ,' > $@.tmp); \
	$(MV) $@.tmp $@

%.d: %.c
	$(GEN_HEADER_DEPENDENCIES)

%.d: %.s
	$(GEN_HEADER_DEPENDENCIES)

#%elf: $(OBJS)
#	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%lst: %elf
	$(OBJDUMP) -dSst $< > $@

%bin: %elf
	$(OBJCOPY) -O binary -S $< $@

%srec: %elf
	$(OBJCOPY) -O srec -S $< $@

%hex: %elf
	$(OBJCOPY) -O ihex -S $< $@

.PHONY: list_predefined_macros
list_predefined_macros:
	touch ~/tmp/foo.h; ${CPP} -dM ~/tmp/foo.h; rm ~/tmp/foo.h

