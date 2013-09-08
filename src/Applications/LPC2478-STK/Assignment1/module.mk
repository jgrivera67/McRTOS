local_pgm  := $(subdirectory)/Assignment1.elf
local_src  := $(subdirectory)/main.c \

local_objs := $(call source-to-object,$(local_src))

programs   += $(local_pgm)
sources    += $(local_src)

$(local_pgm): $(local_objs) $(libraries)
	$(CC) $(LDFLAGS) $+ -o $@
