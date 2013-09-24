#
# McRTOS kernel module-level build makefile 
#
# Copyright (C) 2013 German Rivera
#

ifeq "$(PLATFORM)" "LPC2478-STK"
    local_src := $(subdirectory)/McRTOS_crt_armv4.s
endif

ifeq "$(PLATFORM)" "FRDM-KL25Z"
    local_src := $(subdirectory)/McRTOS_startup_arm_cortex_m.c
endif

local_src += $(subdirectory)/McRTOS_startup.c \
	     $(subdirectory)/McRTOS_kernel_services.c \
	     $(subdirectory)/McRTOS_debugger.c \
	     $(subdirectory)/McRTOS_kernel_services_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/McRTOS_interrupt_service_routines_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/McRTOS_run_time_exception_handlers_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/McRTOS_system_call_wrappers_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/generic_list.c \
	     $(subdirectory)/failure_data_capture.c \
	     $(subdirectory)/McRTOS_execution_controller.c \
	     $(subdirectory)/utils.c \

$(eval $(call make-library, $(subdirectory)/McRTOS.a, $(local_src)))

#
# NOTE: due to a circular dependency we need to append BoardSupport-$(PLATFORM).a
# to 'libraries', after appending this library itself to 'libraries'.
#
libraries += BoardSupport/BoardSupport-$(PLATFORM).a
