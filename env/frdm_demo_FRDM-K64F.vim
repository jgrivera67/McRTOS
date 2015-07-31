let SessionLoad = 1
if &cp | set nocp | endif
let s:so_save = &so | let s:siso_save = &siso | set so=0 siso=0
let v:this_session=expand("<sfile>:p")
silent only
cd ~/MyProjects/EMBSYS/projects/McRTOS
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
set shortmess=aoO
badd +249 src/Applications/LPC2478-STK/McRTOS-demo/main.c
badd +10 makefile
badd +210 src/build.mk
badd +11 src/Applications/LPC2478-STK/McRTOS-demo/module.mk
badd +5 src/McRTOS/module.mk
badd +1 src/BoardSupport/module.mk
badd +602 src/McRTOS/McRTOS_startup.c
badd +576 src/include/McRTOS/McRTOS.h
badd +162 src/include/McRTOS/McRTOS_internals.h
badd +73 src/include/McRTOS/McRTOS_config_parameters.h
badd +134 src/McRTOS/McRTOS_kernel_services.c
badd +15 src/McRTOS/McRTOS_crt_armv4.s
badd +508 src/McRTOS/McRTOS_kernel_services_armv4.s
badd +415 src/include/McRTOS/McRTOS_kernel_services.h
badd +1 src/McRTOS/McRTOS_execution_controller.c
badd +318 src/BoardSupport/LPC2478-STK/lpc2478_interrupt_handlers.s
badd +4 src/McRTOS/McRTOS_interrupt_handlers_armv4.s
badd +137 src/McRTOS/McRTOS_interrupt_service_routines_armv4.s
badd +32 src/McRTOS/failure_data_capture.c
badd +265 src/include/McRTOS/failure_data_capture.h
badd +89 src/include/McRTOS/compile_time_checks.h
badd +73 src/McRTOS/run_time_exception_handlers_armv4.s
badd +342 src/McRTOS/McRTOS_system_call_wrappers_armv4.s
badd +837 src/McRTOS/utils.c
badd +38 src/include/McRTOS/utils.h
badd +734 src/BoardSupport/LPC2478-STK/lpc2478_hardware_abstractions.c
badd +475 src/include/BoardSupport/hardware_abstractions.h
badd +156 src/include/BoardSupport/LPC2478-STK/lpc2478_stk_board.h
badd +137 src/include/BoardSupport/arm_defs.h
badd +2 src/include/BoardSupport/LPC2478-STK/lpc2478.h
badd +42 src/include/BoardSupport/LPC2478-STK/lpc2478_stk_board_public.h
badd +6 src/include/BoardSupport/LPC2478-STK/lpc2478_arm_defs.h
badd +215 src/include/BoardSupport/LPC2478-STK/lpc2478_vic.h
badd +11 src/BoardSupport/LPC2478-STK/lpc2478_lcd.c
badd +6 src/BoardSupport/LPC2478-STK/lpc2478_touch_screen.c
badd +4 src/include/BoardSupport/LPC2478-STK/lpc2478_lcd.h
badd +9 src/include/BoardSupport/lcd.h
badd +8 src/include/McRTOS/arm_defs.h
badd +9 src/Applications/McRTOS-demo/module.mk
badd +1 src/Applications/McRTOS-demo/main.c
badd +16 src/Applications/autonomous_car/main.c
badd +11 src/Applications/autonomous_car/module.mk
badd +1 src/BoardSupport/FRDM-KL25Z/FRDM-KL25Z-flash.ld
badd +655 src/include/BoardSupport/CMSIS/core_cm0plus.h
badd +662 src/include/BoardSupport/CMSIS/core_cmInstr.h
badd +340 src/include/BoardSupport/CMSIS/core_cmFunc.h
badd +1559 src/include/BoardSupport/FRDM-KL25Z/MKL25Z4.h
badd +1 src/McRTOS/McRTOS_crt_armv6-m.S
badd +476 src/McRTOS/McRTOS_startup_arm_cortex_m.c
badd +513 src/BoardSupport/FRDM-KL25Z/frdm_kl25z_hardware_abstractions.c
badd +87 src/include/McRTOS/McRTOS_startup_arm_cortex_m.h
badd +101 src/include/McRTOS/McRTOS_arm_cortex_m.h
badd +58 src/include/BoardSupport/FRDM-KL25Z/kl25z_soc.h
badd +70 src/include/BoardSupport/LPC2478-STK/lpc2478_uarts.h
badd +78 src/include/BoardSupport/LPC2478-STK/lpc2478_gpio.h
badd +2 src/McRTOS/McRTOS_kernel_services_cortex_m.c
badd +29 src/McRTOS/McRTOS_kernel_services_arm_cortex_m.c
badd +12 src/McRTOS/McRTOS_kernel_services_armv6_m.c
badd +1 src/McRTOS/McRTOS_kernel_services_armv6_m.s
badd +1 src/McRTOS/McRTOS_system_call_wrappers_armv6_m.s
badd +1 src/McRTOS/McRTOS_interrupt_service_routines_armv6_m.s
badd +1 src/McRTOS/McRTOS_startup_arm_cortex_m.h
badd +11 src/McRTOS/McRTOS_interrupt_service_routines_arm_cortex_m.s
badd +46 src/McRTOS/McRTOS_run_time_exception_handlers_arm_cortex_m.s
badd +1 src/McRTOS/McRTOS_system_call_wrappers_arm_cortex_m.s
badd +126 src/McRTOS/McRTOS_debugger.c
badd +250 src/McRTOS/McRTOS_kernel_services_arm_cortex_m.s
badd +1 src/include/McRTOS/arm_cortex_m_macros.s
badd +18 src/include/BoardSupport/FRDM-KL25Z/frdm_board.h
badd +13 src/include/BoardSupport/FRDM-KL25Z/tfc_board.h
badd +25 src/BoardSupport/FRDM-KL25Z/tfc_board_hardware_abstractions.c
badd +144 src/BoardSupport/FRDM-KL25Z/frdm_board_hardware_abstractions.c
badd +15 src/BoardSupport/FRDM-KL25Z/kl25z_hardware_abstractions.c
badd +1073 src/BoardSupport/FRDM-KL25Z/kl25z_soc_hardware_abstractions.c
badd +101 src/include/BoardSupport/FRDM-KL25Z/kl25z_soc_public.h
badd +77 src/Applications/McRTOS-FRDM/main.c
badd +1 /Users/b46482/embsys/projects/McRTOS/src/include/BoardSupport/FRDM-K64F/k64f_soc_public.h
badd +114 /Users/b46482/embsys/projects/McRTOS/src/BoardSupport/LM4F120-LaunchPad/lm4f120_soc_hardware_abstractions.c
badd +1669 src/BoardSupport/FRDM-K64F/k64f_soc_hardware_abstractions.c
badd +12 src/BoardSupport/frdm_board_hardware_abstractions.c
badd +18 src/include/BoardSupport/frdm_board.h
badd +343 src/include/BoardSupport/FRDM-K64F/k64f_soc.h
badd +4 src/Applications/McRTOS-FRDM/module.mk
badd +20 src/include/BoardSupport/FRDM-K64F/MK64F12.h
badd +48 src/include/BoardSupport/CMSIS/core_cm4.h
badd +115 /Users/b46482/embsys/projects/McRTOS/src/include/BoardSupport/FRDM-K20D50/k20d5_soc_public.h
badd +12 src/BoardSupport/FRDM-K64F/K64F_SOC-flash.ld
badd +203 src/BoardSupport/FRDM-K64F/frdm_board_hardware_abstractions.c
badd +51 src/Applications/frdm_demo/main.c
badd +611 src/BoardSupport/LPC2478-STK/lpc2478_ethernet.c
badd +259 /Users/b46482/ide/vimrc.vim
badd +498 /Users/b46482/embsys/projects/McRTOS/src/include/BoardSupport/LPC2478-STK/lpc2478_ethernet.h
badd +806 src/BoardSupport/FRDM-K64F/k64f_soc_enet.c
badd +1 src/Applications/frdm_demo/module.mk
badd +1 src/McRTOS_tcpip/module.mk
badd +18 src/McRTOS_tcpip/McRTOS_tcpip.c
badd +18 src/include/McRTOS_tcpip/McRTOS_tcpip.h
badd +7 /Users/b46482/embsys/projects/McRTOS/src/Networking/lwip/src/netif/etharp.c
badd +30 src/Networking/networking.c
badd +1 src/include/Networking/networking.h
badd +1 src/Networking/module.mk
badd +150 /Users/b46482/embsys/projects/McRTOS/src/Networking/lwip_glue/ethernetif.c
badd +37 /Users/b46482/embsys/projects/McRTOS/src/Networking/lwip/src/netif/ethernetif.c
badd +52 /Users/b46482/embsys/tmp/Freescale_KSDK.notes
badd +55 /Users/b46482/embsys/projects/McRTOS/src/Networking/lwip_glue/arch/sys_arch.h
badd +1 doc/tcpip_notes.txt
badd +6 src/Networking/lwip_glue/sys_arch.c
badd +470 /Users/b46482/embsys/projects/McRTOS/src/Networking/lwip/src/api/tcpip.c
badd +1293 /Users/b46482/embsys/projects/McRTOS/src/Networking/lwip/src/include/lwip/opt.h
badd +369 /Users/b46482/embsys/projects/McRTOS/src/Networking/lwip/src/netif/slipif.c
badd +1 src/include/BoardSupport/FRDM-K64F/k64f_soc_enet.h
badd +1 src/McRTOS/McRTOS_command_processor.c
badd +1 src/include/McRTOS/McRTOS_command_processor.h
badd +1 src/include/BoardSupport/FRDM-K64F/k64f_soc_public.h
badd +767 src/BoardSupport/LaunchPad-LM4F120/lm4f120_soc_hardware_abstractions.c
badd +90 ~/MyProjects/EMBSYS/projects/McRTOS/src/BoardSupport/LaunchPad-LM4F120/launchpad_board_hardware_abstractions.c
badd +4 src/BoardSupport/LaunchPad-LM4F120/LM4F120_SOC-flash.ld
badd +221 src/Applications/launchpad_demo/main.c
silent! argdel *
set lines=54 columns=207
winpos 0 0
edit src/Applications/frdm_demo/main.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd w
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
4wincmd k
wincmd w
wincmd w
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 24 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 126 + 103) / 207)
exe '2resize ' . ((&lines * 26 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 126 + 103) / 207)
exe '3resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 80 + 103) / 207)
exe '4resize ' . ((&lines * 21 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 80 + 103) / 207)
exe '5resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 80 + 103) / 207)
exe '6resize ' . ((&lines * 14 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 80 + 103) / 207)
exe '7resize ' . ((&lines * 4 + 27) / 54)
exe 'vert 7resize ' . ((&columns * 80 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 199 - ((11 * winheight(0) + 12) / 24)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
199
normal! 0
wincmd w
argglobal
edit src/BoardSupport/FRDM-K64F/K64F_SOC-flash.ld
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 89 - ((12 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
89
normal! 08|
wincmd w
argglobal
edit makefile
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 19 - ((0 * winheight(0) + 3) / 7)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
19
normal! 0
wincmd w
argglobal
edit src/build.mk
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 124 - ((0 * winheight(0) + 10) / 21)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
124
normal! 0
wincmd w
argglobal
edit src/Applications/frdm_demo/module.mk
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 8 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
8
normal! 0
wincmd w
argglobal
edit src/McRTOS/module.mk
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 19 - ((0 * winheight(0) + 7) / 14)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
19
normal! 029|
wincmd w
argglobal
edit src/BoardSupport/module.mk
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 33 - ((0 * winheight(0) + 2) / 4)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
33
normal! 062|
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 24 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 126 + 103) / 207)
exe '2resize ' . ((&lines * 26 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 126 + 103) / 207)
exe '3resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 80 + 103) / 207)
exe '4resize ' . ((&lines * 21 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 80 + 103) / 207)
exe '5resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 80 + 103) / 207)
exe '6resize ' . ((&lines * 14 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 80 + 103) / 207)
exe '7resize ' . ((&lines * 4 + 27) / 54)
exe 'vert 7resize ' . ((&columns * 80 + 103) / 207)
tabedit src/McRTOS/McRTOS_startup_arm_cortex_m.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
3wincmd k
wincmd w
wincmd w
wincmd w
wincmd w
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
2wincmd k
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 26 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 154 + 103) / 207)
exe '2resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 154 + 103) / 207)
exe '3resize ' . ((&lines * 12 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 154 + 103) / 207)
exe '4resize ' . ((&lines * 9 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 154 + 103) / 207)
exe '5resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 52 + 103) / 207)
exe '6resize ' . ((&lines * 36 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 52 + 103) / 207)
exe '7resize ' . ((&lines * 6 + 27) / 54)
exe 'vert 7resize ' . ((&columns * 52 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 476 - ((0 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
476
normal! 07|
wincmd w
argglobal
edit src/McRTOS/McRTOS_startup.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 605 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
605
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/McRTOS_arm_cortex_m.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 55 - ((0 * winheight(0) + 6) / 12)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
55
normal! 0
wincmd w
argglobal
edit src/McRTOS/McRTOS_kernel_services.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 2372 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
2372
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/McRTOS.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 34 - ((0 * winheight(0) + 3) / 7)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
34
normal! 03|
wincmd w
argglobal
edit src/include/McRTOS/McRTOS_config_parameters.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 18) / 36)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/McRTOS_internals.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 35 - ((0 * winheight(0) + 3) / 6)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
35
normal! 03|
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 26 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 154 + 103) / 207)
exe '2resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 154 + 103) / 207)
exe '3resize ' . ((&lines * 12 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 154 + 103) / 207)
exe '4resize ' . ((&lines * 9 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 154 + 103) / 207)
exe '5resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 52 + 103) / 207)
exe '6resize ' . ((&lines * 36 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 52 + 103) / 207)
exe '7resize ' . ((&lines * 6 + 27) / 54)
exe 'vert 7resize ' . ((&columns * 52 + 103) / 207)
tabedit src/McRTOS/McRTOS_command_processor.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe 'vert 1resize ' . ((&columns * 85 + 103) / 207)
exe 'vert 2resize ' . ((&columns * 121 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 25) / 51)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/McRTOS_command_processor.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 25) / 51)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
2wincmd w
exe 'vert 1resize ' . ((&columns * 85 + 103) / 207)
exe 'vert 2resize ' . ((&columns * 121 + 103) / 207)
tabedit src/McRTOS/McRTOS_kernel_services.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd w
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
3wincmd k
wincmd w
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 44 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 157 + 103) / 207)
exe '2resize ' . ((&lines * 6 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 157 + 103) / 207)
exe '3resize ' . ((&lines * 5 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 49 + 103) / 207)
exe '4resize ' . ((&lines * 16 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 49 + 103) / 207)
exe '5resize ' . ((&lines * 21 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 49 + 103) / 207)
exe '6resize ' . ((&lines * 6 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 49 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 3128 - ((0 * winheight(0) + 22) / 44)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
3128
normal! 0
wincmd w
argglobal
edit src/McRTOS/McRTOS_kernel_services_arm_cortex_m.s
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 3) / 6)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/McRTOS_kernel_services.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 953 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
953
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/McRTOS_internals.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 797 - ((6 * winheight(0) + 8) / 16)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
797
normal! 0
wincmd w
argglobal
edit src/McRTOS/McRTOS_system_call_wrappers_arm_cortex_m.s
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 53 - ((0 * winheight(0) + 10) / 21)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
53
normal! 05|
wincmd w
argglobal
enew
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 44 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 157 + 103) / 207)
exe '2resize ' . ((&lines * 6 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 157 + 103) / 207)
exe '3resize ' . ((&lines * 5 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 49 + 103) / 207)
exe '4resize ' . ((&lines * 16 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 49 + 103) / 207)
exe '5resize ' . ((&lines * 21 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 49 + 103) / 207)
exe '6resize ' . ((&lines * 6 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 49 + 103) / 207)
tabedit src/McRTOS/McRTOS_execution_controller.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
3wincmd k
wincmd w
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 37 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 121 + 103) / 207)
exe '2resize ' . ((&lines * 5 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 85 + 103) / 207)
exe '3resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 85 + 103) / 207)
exe '4resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 85 + 103) / 207)
exe '5resize ' . ((&lines * 27 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 85 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 18) / 37)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/arm_defs.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 13 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
13
normal! 019|
wincmd w
argglobal
edit src/include/McRTOS/arm_cortex_m_macros.s
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit src/McRTOS/McRTOS_interrupt_service_routines_arm_cortex_m.s
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 30 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
30
normal! 0
wincmd w
argglobal
edit src/McRTOS/McRTOS_run_time_exception_handlers_arm_cortex_m.s
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 46 - ((0 * winheight(0) + 13) / 27)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
46
normal! 0
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 37 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 121 + 103) / 207)
exe '2resize ' . ((&lines * 5 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 85 + 103) / 207)
exe '3resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 85 + 103) / 207)
exe '4resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 85 + 103) / 207)
exe '5resize ' . ((&lines * 27 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 85 + 103) / 207)
tabedit src/McRTOS/failure_data_capture.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd w
wincmd _ | wincmd |
split
1wincmd k
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 37 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 160 + 103) / 207)
exe '2resize ' . ((&lines * 13 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 160 + 103) / 207)
exe '3resize ' . ((&lines * 38 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 46 + 103) / 207)
exe '4resize ' . ((&lines * 12 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 46 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 32 - ((0 * winheight(0) + 18) / 37)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
32
normal! 0
wincmd w
argglobal
edit src/McRTOS/McRTOS_debugger.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 139 - ((0 * winheight(0) + 6) / 13)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
139
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/compile_time_checks.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 6 - ((0 * winheight(0) + 19) / 38)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 02|
wincmd w
argglobal
edit src/include/McRTOS/failure_data_capture.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 6 - ((0 * winheight(0) + 6) / 12)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 02|
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 37 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 160 + 103) / 207)
exe '2resize ' . ((&lines * 13 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 160 + 103) / 207)
exe '3resize ' . ((&lines * 38 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 46 + 103) / 207)
exe '4resize ' . ((&lines * 12 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 46 + 103) / 207)
tabedit src/McRTOS/utils.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 29 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 99 + 103) / 207)
exe '2resize ' . ((&lines * 29 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 107 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 837 - ((27 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
837
normal! 0
wincmd w
argglobal
edit src/include/McRTOS/utils.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 224 - ((28 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
224
normal! 0
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 29 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 99 + 103) / 207)
exe '2resize ' . ((&lines * 29 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 107 + 103) / 207)
tabedit src/BoardSupport/FRDM-K64F/k64f_soc_hardware_abstractions.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
2wincmd k
wincmd w
wincmd w
wincmd w
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
5wincmd k
wincmd w
wincmd w
wincmd w
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 28 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 122 + 103) / 207)
exe '2resize ' . ((&lines * 18 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 122 + 103) / 207)
exe '3resize ' . ((&lines * 3 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 122 + 103) / 207)
exe '4resize ' . ((&lines * 9 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 84 + 103) / 207)
exe '5resize ' . ((&lines * 10 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 84 + 103) / 207)
exe '6resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 84 + 103) / 207)
exe '7resize ' . ((&lines * 9 + 27) / 54)
exe 'vert 7resize ' . ((&columns * 84 + 103) / 207)
exe '8resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 8resize ' . ((&columns * 84 + 103) / 207)
exe '9resize ' . ((&lines * 10 + 27) / 54)
exe 'vert 9resize ' . ((&columns * 84 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1669 - ((0 * winheight(0) + 14) / 28)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1669
normal! 015|
wincmd w
argglobal
edit src/BoardSupport/FRDM-K64F/k64f_soc_enet.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 772 - ((7 * winheight(0) + 9) / 18)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
772
normal! 0
wincmd w
argglobal
edit src/BoardSupport/FRDM-K64F/frdm_board_hardware_abstractions.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 8 - ((1 * winheight(0) + 1) / 3)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
8
normal! 0
wincmd w
argglobal
edit src/include/BoardSupport/hardware_abstractions.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 8 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
8
normal! 024|
wincmd w
argglobal
edit src/include/BoardSupport/FRDM-K64F/k64f_soc_enet.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 6 - ((0 * winheight(0) + 5) / 10)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 024|
wincmd w
argglobal
edit src/include/BoardSupport/FRDM-K64F/k64f_soc_public.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 294 - ((2 * winheight(0) + 3) / 7)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
294
normal! 0
wincmd w
argglobal
edit src/include/BoardSupport/FRDM-K64F/k64f_soc.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
enew
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
wincmd w
argglobal
enew
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 28 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 122 + 103) / 207)
exe '2resize ' . ((&lines * 18 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 122 + 103) / 207)
exe '3resize ' . ((&lines * 3 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 122 + 103) / 207)
exe '4resize ' . ((&lines * 9 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 84 + 103) / 207)
exe '5resize ' . ((&lines * 10 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 84 + 103) / 207)
exe '6resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 6resize ' . ((&columns * 84 + 103) / 207)
exe '7resize ' . ((&lines * 9 + 27) / 54)
exe 'vert 7resize ' . ((&columns * 84 + 103) / 207)
exe '8resize ' . ((&lines * 1 + 27) / 54)
exe 'vert 8resize ' . ((&columns * 84 + 103) / 207)
exe '9resize ' . ((&lines * 10 + 27) / 54)
exe 'vert 9resize ' . ((&columns * 84 + 103) / 207)
tabedit src/include/BoardSupport/FRDM-K64F/MK64F12.h
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
wincmd _ | wincmd |
split
2wincmd k
wincmd w
wincmd w
wincmd w
wincmd _ | wincmd |
split
1wincmd k
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 15 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 135 + 103) / 207)
exe '2resize ' . ((&lines * 26 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 135 + 103) / 207)
exe '3resize ' . ((&lines * 8 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 135 + 103) / 207)
exe '4resize ' . ((&lines * 43 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 71 + 103) / 207)
exe '5resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 71 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 20 - ((12 * winheight(0) + 7) / 15)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
20
normal! 02|
wincmd w
argglobal
edit src/include/McRTOS/McRTOS_arm_cortex_m.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 56 - ((1 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
56
normal! 0
wincmd w
argglobal
edit src/include/BoardSupport/CMSIS/core_cm4.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 372 - ((0 * winheight(0) + 4) / 8)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
372
normal! 017|
wincmd w
argglobal
edit src/include/BoardSupport/CMSIS/core_cmInstr.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 663 - ((1 * winheight(0) + 21) / 43)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
663
normal! 0
wincmd w
argglobal
edit src/include/BoardSupport/CMSIS/core_cmFunc.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 367 - ((0 * winheight(0) + 3) / 7)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
367
normal! 059|
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 15 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 135 + 103) / 207)
exe '2resize ' . ((&lines * 26 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 135 + 103) / 207)
exe '3resize ' . ((&lines * 8 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 135 + 103) / 207)
exe '4resize ' . ((&lines * 43 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 71 + 103) / 207)
exe '5resize ' . ((&lines * 7 + 27) / 54)
exe 'vert 5resize ' . ((&columns * 71 + 103) / 207)
tabedit src/Networking/networking.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd w
wincmd _ | wincmd |
split
1wincmd k
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 47 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 117 + 103) / 207)
exe '2resize ' . ((&lines * 3 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 117 + 103) / 207)
exe '3resize ' . ((&lines * 35 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 89 + 103) / 207)
exe '4resize ' . ((&lines * 15 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 89 + 103) / 207)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 30 - ((0 * winheight(0) + 23) / 47)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
30
normal! 0
wincmd w
argglobal
edit doc/tcpip_notes.txt
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 1) / 3)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit src/include/Networking/networking.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 20 - ((0 * winheight(0) + 17) / 35)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
20
normal! 0
wincmd w
argglobal
edit src/Networking/module.mk
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 8 - ((0 * winheight(0) + 7) / 15)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
8
normal! 0
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 47 + 27) / 54)
exe 'vert 1resize ' . ((&columns * 117 + 103) / 207)
exe '2resize ' . ((&lines * 3 + 27) / 54)
exe 'vert 2resize ' . ((&columns * 117 + 103) / 207)
exe '3resize ' . ((&lines * 35 + 27) / 54)
exe 'vert 3resize ' . ((&columns * 89 + 103) / 207)
exe '4resize ' . ((&lines * 15 + 27) / 54)
exe 'vert 4resize ' . ((&columns * 89 + 103) / 207)
tabnext 1
if exists('s:wipebuf')
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20 shortmess=filnxtToO
let s:sx = expand("<sfile>:p:r")."x.vim"
if file_readable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &so = s:so_save | let &siso = s:siso_save
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
