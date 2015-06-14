#
# Networking stack module-level build makefile
#
# Copyright (C) 2014 German Rivera
#
local_src :=	$(subdirectory)/networking.c \

$(eval $(call make-library, $(subdirectory)/Networking.a, $(local_src)))

