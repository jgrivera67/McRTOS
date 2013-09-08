local_src := $(subdirectory)/lcd_fonts.c \

ifeq "$(PLATFORM)" "LPC2478-STK"
    local_src += $(subdirectory)/$(PLATFORM)/lpc2478_hardware_abstractions.c \
		 $(subdirectory)/$(PLATFORM)/lpc2478_lcd.c \
		 $(subdirectory)/$(PLATFORM)/lpc2478_touch_screen.c \
		 $(subdirectory)/$(PLATFORM)/lpc2478_touch_screen_xy_reading_to_80x80_tile_map.c
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

