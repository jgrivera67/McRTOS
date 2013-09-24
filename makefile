#
# McRTOS application top-level makefile 
#
# Copyright (C) 2013 German Rivera
# 
# Author: German Rivera - September, 2013
#

APPLICATION := autonomous_car

#
# PLATFORM values: LPC2478-STK, LM4F120-LaunchPad, FRDM-KL25Z
#
PLATFORM := FRDM-KL25Z

#
# BUILD_FLAVOR values: debug, reliability, performance
#
BUILD_FLAVOR := debug
SOURCE_DIR := src
OBJECT_DIR := $(PLATFORM)-obj-$(BUILD_FLAVOR)
UNIT_TESTS_DIR := unit_tests
SCRIPTS_DIR := scripts
DOC_DIR := doc

POWERSHELL := powershell.exe -executionPolicy RemoteSigned 
	
RUN_BUILD_MAKEFILE := $(MAKE) --file=../$(SOURCE_DIR)/build.mk \
			      --directory=$(OBJECT_DIR) \
			      $(BUILD_TARGET) \
			      APPLICATION='$(APPLICATION)' \
			      PLATFORM='$(PLATFORM)' \
			      BUILD_FLAVOR='$(BUILD_FLAVOR)' \
			      CMD_LINE_CFLAGS='$(CMD_LINE_CFLAGS)' \
			      
RUN_TESTS_MAKEFILE := $(MAKE) --directory=$(UNIT_TESTS_DIR) 

RUN_DOC_MAKEFILE := $(MAKE) --directory=$(DOC_DIR) 

RUN_2478 := ${POWERSHELL} $(SCRIPTS_DIR)/run_2478.ps1 \
			  '$$env:bin_file_prefix'

COMMON_CFLAGS = -Wall -Wstrict-prototypes -fms-extensions -Wextra -Wformat \
	        -std=gnu99 -g3

.PHONY: build create_object_dir run_tests doc \
	clean run_2478_update_flash run_2478

build: create_object_dir
	$(RUN_BUILD_MAKEFILE) COMMON_CFLAGS='$(COMMON_CFLAGS)'
			      
rebuild: create_object_dir
	$(RUN_BUILD_MAKEFILE) clean
	$(RUN_BUILD_MAKEFILE) COMMON_CFLAGS='$(COMMON_CFLAGS)'
			      
create_object_dir:
	$(shell test -d $(OBJECT_DIR) || mkdir $(OBJECT_DIR))

run_tests:
	$(RUN_TESTS_MAKEFILE) COMMON_CFLAGS='$(COMMON_CFLAGS)'

doc:
	$(RUN_DOC_MAKEFILE) 

clean:
	#$(RUN_BUILD_MAKEFILE) clean
	$(RM) -r $(OBJECT_DIR)
	$(RUN_TESTS_MAKEFILE) clean
	$(RUN_DOC_MAKEFILE) clean_obj

run_2478_update_flash:
	${RUN_2478} -updateFlash

run_2478:
	${RUN_2478}

