#
# McRTOS application top-level makefile
#
# Copyright (C) 2013 German Rivera
#
# Author: German Rivera - September, 2013
#
# NOTE: additional CFLAGS can be specfied in the command line like this:
# make EXTRA_CFLAGS=...
#

#
# Application subdirectory
# This variable must be set via an environment variable
#
APPLICATION ?=

#
# PLATFORM values: LPC2478-STK, LaunchPad-LM4F120, FRDM-KL25Z, FRDM-K20D50,
# FRDM-K64F
# This variable must be set via an environment variable
#
PLATFORM ?=

#
# BUILD_FLAVOR values: debug, reliability, performance
# This variable must be set via an environment variable
#
BUILD_FLAVOR ?=

SOURCE_DIR := src
OBJECT_DIR := $(PLATFORM)-obj-$(BUILD_FLAVOR)
UNIT_TESTS_DIR := unit_tests
SCRIPTS_DIR := scripts
DOC_DIR := doc

RUN_BUILD_MAKEFILE := $(MAKE) --directory=$(OBJECT_DIR) \
			      --file=../$(SOURCE_DIR)/build.mk \
			      $(BUILD_TARGET) \
			      APPLICATION='$(APPLICATION)' \
			      PLATFORM='$(PLATFORM)' \
			      BUILD_FLAVOR='$(BUILD_FLAVOR)' \
			      EXTRA_CFLAGS='$(EXTRA_CFLAGS)' \

RUN_TESTS_MAKEFILE := $(MAKE) --directory=$(UNIT_TESTS_DIR)

RUN_DOC_MAKEFILE := $(MAKE) --directory=$(DOC_DIR)

COMMON_CFLAGS = -Wall -Wstrict-prototypes -fms-extensions -Wextra -Wformat \
		-Wunreachable-code -Wshadow \
	        -std=gnu11 -g3

.PHONY: build create_object_dir run_tests doc \
	clean

build: create_object_dir
	$(RUN_BUILD_MAKEFILE) COMMON_CFLAGS='$(COMMON_CFLAGS)'

rebuild: create_object_dir
	$(RUN_BUILD_MAKEFILE) clean
	$(RUN_BUILD_MAKEFILE) COMMON_CFLAGS='$(COMMON_CFLAGS)'

create_object_dir:
	$(shell test -d $(OBJECT_DIR) || mkdir $(OBJECT_DIR))

update_flash: build
	cp $(BIN_FILE) /media/$(USER)/MBED
	sync

run_tests:
	$(RUN_TESTS_MAKEFILE) COMMON_CFLAGS='$(COMMON_CFLAGS)'

doc:
	$(RUN_DOC_MAKEFILE)

clean:
	#$(RUN_BUILD_MAKEFILE) clean
	$(RM) -r $(OBJECT_DIR)
	$(RUN_TESTS_MAKEFILE) clean
	$(RUN_DOC_MAKEFILE) clean_obj

