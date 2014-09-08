#
# Networking stack module-level build makefile
#
# Copyright (C) 2014 German Rivera
#
local_src :=	$(subdirectory)/networking.c \
		$(subdirectory)/lwip_glue/ethernetif.c \
		$(subdirectory)/lwip_glue/sys_arch.c \
		$(subdirectory)/lwip/src/api/api_lib.c \
		$(subdirectory)/lwip/src/api/api_msg.c \
		$(subdirectory)/lwip/src/api/err.c \
		$(subdirectory)/lwip/src/api/netbuf.c \
		$(subdirectory)/lwip/src/api/netdb.c \
		$(subdirectory)/lwip/src/api/netifapi.c \
		$(subdirectory)/lwip/src/api/sockets.c \
		$(subdirectory)/lwip/src/api/tcpip.c \
		$(subdirectory)/lwip/src/core/def.c \
		$(subdirectory)/lwip/src/core/dhcp.c \
		$(subdirectory)/lwip/src/core/dns.c \
		$(subdirectory)/lwip/src/core/init.c \
		$(subdirectory)/lwip/src/core/mem.c \
		$(subdirectory)/lwip/src/core/memp.c \
		$(subdirectory)/lwip/src/core/netif.c \
		$(subdirectory)/lwip/src/core/pbuf.c \
		$(subdirectory)/lwip/src/core/raw.c \
		$(subdirectory)/lwip/src/core/stats.c \
		$(subdirectory)/lwip/src/core/sys.c \
		$(subdirectory)/lwip/src/core/tcp.c \
		$(subdirectory)/lwip/src/core/tcp_in.c \
		$(subdirectory)/lwip/src/core/tcp_out.c \
		$(subdirectory)/lwip/src/core/timers.c \
		$(subdirectory)/lwip/src/core/udp.c \
		$(subdirectory)/lwip/src/core/ipv4/autoip.c \
		$(subdirectory)/lwip/src/core/ipv4/icmp.c \
		$(subdirectory)/lwip/src/core/ipv4/igmp.c \
		$(subdirectory)/lwip/src/core/ipv4/inet.c \
		$(subdirectory)/lwip/src/core/ipv4/inet_chksum.c \
		$(subdirectory)/lwip/src/core/ipv4/ip.c \
		$(subdirectory)/lwip/src/core/ipv4/ip_addr.c \
		$(subdirectory)/lwip/src/core/ipv4/ip_frag.c \
		$(subdirectory)/lwip/src/core/ipv6/icmp6.c \
		$(subdirectory)/lwip/src/core/ipv6/inet6.c \
		$(subdirectory)/lwip/src/core/ipv6/ip6.c \
		$(subdirectory)/lwip/src/core/ipv6/ip6_addr.c \
		$(subdirectory)/lwip/src/netif/etharp.c \

include_dirs += $(subdirectory)/lwip_glue \
	        $(subdirectory)/lwip/src/include \

local_subdirs := $(subdirectory)/lwip/src/api \
		 $(subdirectory)/lwip/src/core \
		 $(subdirectory)/lwip/src/core/ipv4 \
		$(subdirectory)/lwip/src/core/ipv6 \
		 $(subdirectory)/lwip/src/netif \

create-local-output-subdirs :=				\
	$(shell for f in $(local_subdirs);		\
		do					\
		  $(TEST) -d $$f || $(MKDIR) $$f;	\
		done)

$(eval $(call make-library, $(subdirectory)/lwip.a, $(local_src)))

