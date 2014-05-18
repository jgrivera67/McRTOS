#
# McRTOS board support module-level build makefile
#
# Copyright (C) 2013 German Rivera
#

ifeq "$(PLATFORM)" "LPC2478-STK"
    local_src := \
                 $(subdirectory)/$(PLATFORM)/lpc2478_hardware_abstractions.c \
		 $(subdirectory)/$(PLATFORM)/lpc2478_lcd.c \
		 $(subdirectory)/lcd_fonts.c \
		 $(subdirectory)/$(PLATFORM)/lpc2478_touch_screen.c \
		 $(subdirectory)/$(PLATFORM)/lpc2478_touch_screen_xy_reading_to_80x80_tile_map.c
endif

ifeq "$(PLATFORM)" "FRDM-KL25Z"
    local_src := \
                 $(subdirectory)/$(PLATFORM)/kl25z_soc_hardware_abstractions.c \
                 $(subdirectory)/$(PLATFORM)/kl25z_interrupt_service_routines.s \
                 $(subdirectory)/frdm_board_hardware_abstractions.c \
                 $(subdirectory)/$(PLATFORM)/tfc_board_hardware_abstractions.c
endif

ifeq "$(PLATFORM)" "FRDM-K20D5"
    local_src := \
                 $(subdirectory)/$(PLATFORM)/k20d5_soc_hardware_abstractions.c \
                 $(subdirectory)/$(PLATFORM)/k20d5_interrupt_service_routines.s \
                 $(subdirectory)/frdm_board_hardware_abstractions.c
endif

ifeq "$(PLATFORM)" "FRDM-K64F"
    local_src := \
                 $(subdirectory)/$(PLATFORM)/k64f_soc_hardware_abstractions.c \
                 $(subdirectory)/$(PLATFORM)/k64f_interrupt_service_routines.s \
                 $(subdirectory)/frdm_board_hardware_abstractions.c
endif

ifeq "$(PLATFORM)" "LM4F120-LaunchPad"
    local_src := \
                 $(subdirectory)/$(PLATFORM)/lm4f120_soc_hardware_abstractions.c \
                 $(subdirectory)/$(PLATFORM)/lm4f120_interrupt_service_routines.s \
                 $(subdirectory)/$(PLATFORM)/launchpad_board_hardware_abstractions.c
endif

local_subdirs := $(subdirectory)/$(PLATFORM)

create-local-output-subdirs :=				\
	$(shell for f in $(local_subdirs);		\
		do					\
		  $(TEST) -d $$f || $(MKDIR) $$f;	\
		done)

$(eval $(call make-library, $(subdirectory)/BoardSupport-$(PLATFORM).a, $(local_src)))

#
# NOTE: due to a circular dependency we need to append McRTOS.a to 'libraries',
# after appending this library itself to 'libraries'.
#
libraries += McRTOS/McRTOS.a

