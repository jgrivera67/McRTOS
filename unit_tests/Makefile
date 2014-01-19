#---------
#
# CppUTest Examples Makefile
#
#----------

#Set this to @ to keep the makefile quiet
ifndef SILENCE
	SILENCE = #@
endif

#--- Inputs ----#
COMPONENT_NAME = McRTOS
CPPUTEST_HOME = ~/embsys/tools/cpputest-3.5

CPPUTEST_PEDANTIC_ERRORS = N
#CPPUTEST_USE_EXTENSIONS = Y
CPP_PLATFORM = Gcc

CPPUTEST_CFLAGS += ${COMMON_CFLAGS} -D_RELIABILITY_CHECKS_
CPPUTEST_CPPFLAGS += -D_RELIABILITY_CHECKS_

SRC_FILES = \
	../src/McRTOS/generic_list.c

TEST_SRC_DIRS = \
	.

INCLUDE_DIRS = \
	../inc/McRTOS \
	$(CPPUTEST_HOME)/include

include $(CPPUTEST_HOME)/build/MakefileWorker.mk


