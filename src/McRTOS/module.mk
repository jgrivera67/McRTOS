local_src := $(subdirectory)/McRTOS_startup.c \
	     $(subdirectory)/McRTOS_crt_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/McRTOS_system_call_wrappers_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/McRTOS_kernel_services.c \
	     $(subdirectory)/McRTOS_kernel_services_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/generic_list.c \
	     $(subdirectory)/failure_data_capture.c \
             $(subdirectory)/run_time_exception_handlers_$(CPU_ARCHITECTURE).s \
	     $(subdirectory)/McRTOS_execution_controller.c \
	     $(subdirectory)/McRTOS_interrupt_service_routines_armv4.s \
	     $(subdirectory)/utils.c \

$(eval $(call make-library, $(subdirectory)/McRTOS.a, $(local_src)))

#
# NOTE: due to a circular dependency we need to append BoardSupport-$(PLATFORM).a
# to 'libraries', after appending this library itself to 'libraries'.
#
libraries += BoardSupport/BoardSupport-$(PLATFORM).a
