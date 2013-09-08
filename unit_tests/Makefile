#---------
#
# CppUTest Examples Makefile
#
#----------

#Set this to @ to keep the makefile quiet
ifndef SILENCE
	SILENCE = @
endif

#--- Inputs ----#
COMPONENT_NAME = McRTOS
CPPUTEST_HOME = ${USERPROFILE}/MyProjects/EMBSYS/Apps/CppUTest

CPPUTEST_PEDANTIC_ERRORS = N
#CPPUTEST_USE_EXTENSIONS = Y
CPP_PLATFORM = Gcc

CPPUTEST_CFLAGS += ${COMMON_CFLAGS}

SRC_DIRS = \
	../src/McRTOS/generic_data_structures

TEST_SRC_DIRS = \
	.

INCLUDE_DIRS = \
	../inc/McRTOS/generic_data_structures \
	../inc/McRTOS/reliability_monitor \
	../inc/McRTOS/kernel_services \
	../inc/McRTOS/system_call_interface \
	../inc/McRTOS/generic_data_structures \
	../inc/McRTOS/utils \
	../inc/LPC2478-STK-BoardSupport \
	$(CPPUTEST_HOME)/include \

include $(CPPUTEST_HOME)/build/MakefileWorker.mk


