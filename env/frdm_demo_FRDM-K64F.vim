let SessionLoad = 1
if &cp | set nocp | endif
let s:so_save = &so | let s:siso_save = &siso | set so=0 siso=0
let v:this_session=expand("<sfile>:p")
silent only
cd ~/embsys/projects/McRTOS
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
badd +411 src/McRTOS/McRTOS_startup.c
badd +210 inc/McRTOS/McRTOS.h
badd +162 inc/McRTOS/McRTOS_internals.h
badd +73 inc/McRTOS/McRTOS_config_parameters.h
badd +3193 src/McRTOS/McRTOS_kernel_services.c
badd +15 src/McRTOS/McRTOS_crt_armv4.s
badd +508 src/McRTOS/McRTOS_kernel_services_armv4.s
badd +831 inc/McRTOS/McRTOS_kernel_services.h
badd +1 src/McRTOS/McRTOS_execution_controller.c
badd +318 src/BoardSupport/LPC2478-STK/lpc2478_interrupt_handlers.s
badd +4 src/McRTOS/McRTOS_interrupt_handlers_armv4.s
badd +137 src/McRTOS/McRTOS_interrupt_service_routines_armv4.s
badd +1 src/McRTOS/failure_data_capture.c
badd +265 inc/McRTOS/failure_data_capture.h
badd +46 inc/McRTOS/compile_time_checks.h
badd +73 src/McRTOS/run_time_exception_handlers_armv4.s
badd +342 src/McRTOS/McRTOS_system_call_wrappers_armv4.s
badd +1 src/McRTOS/utils.c
badd +38 inc/McRTOS/utils.h
badd +734 src/BoardSupport/LPC2478-STK/lpc2478_hardware_abstractions.c
badd +195 inc/BoardSupport/hardware_abstractions.h
badd +156 inc/BoardSupport/LPC2478-STK/lpc2478_stk_board.h
badd +137 inc/BoardSupport/arm_defs.h
badd +2 inc/BoardSupport/LPC2478-STK/lpc2478.h
badd +42 inc/BoardSupport/LPC2478-STK/lpc2478_stk_board_public.h
badd +6 inc/BoardSupport/LPC2478-STK/lpc2478_arm_defs.h
badd +215 inc/BoardSupport/LPC2478-STK/lpc2478_vic.h
badd +11 src/BoardSupport/LPC2478-STK/lpc2478_lcd.c
badd +6 src/BoardSupport/LPC2478-STK/lpc2478_touch_screen.c
badd +4 inc/BoardSupport/LPC2478-STK/lpc2478_lcd.h
badd +9 inc/BoardSupport/lcd.h
badd +23 inc/McRTOS/arm_defs.h
badd +9 src/Applications/McRTOS-demo/module.mk
badd +1 src/Applications/McRTOS-demo/main.c
badd +16 src/Applications/autonomous_car/main.c
badd +11 src/Applications/autonomous_car/module.mk
badd +66 src/McRTOS/TMP_startup_ARMCM0plus.S
badd +1 prj/LPC2478-STK-flash.ld
badd +1 prj/FRDM-KL25Z-flash.ld
badd +52 prj/TMP_gcc_arm.ld
badd +655 inc/BoardSupport/CMSIS/core_cm0plus.h
badd +662 inc/BoardSupport/CMSIS/core_cmInstr.h
badd +340 inc/BoardSupport/CMSIS/core_cmFunc.h
badd +1559 inc/BoardSupport/FRDM-KL25Z/MKL25Z4.h
badd +31 prj/temp.ld
badd +1 src/McRTOS/McRTOS_crt_armv6-m.S
badd +1 src/McRTOS/McRTOS_startup_arm_cortex_m.c
badd +513 src/BoardSupport/FRDM-KL25Z/frdm_kl25z_hardware_abstractions.c
badd +87 inc/McRTOS/McRTOS_startup_arm_cortex_m.h
badd +101 inc/McRTOS/McRTOS_arm_cortex_m.h
badd +58 inc/BoardSupport/FRDM-KL25Z/kl25z_soc.h
badd +101 prj/KL25Z_SOC-flash.ld
badd +70 inc/BoardSupport/LPC2478-STK/lpc2478_uarts.h
badd +78 inc/BoardSupport/LPC2478-STK/lpc2478_gpio.h
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
badd +1 inc/McRTOS/arm_cortex_m_macros.s
badd +18 inc/BoardSupport/FRDM-KL25Z/frdm_board.h
badd +13 inc/BoardSupport/FRDM-KL25Z/tfc_board.h
badd +25 src/BoardSupport/FRDM-KL25Z/tfc_board_hardware_abstractions.c
badd +18 src/BoardSupport/FRDM-KL25Z/frdm_board_hardware_abstractions.c
badd +15 src/BoardSupport/FRDM-KL25Z/kl25z_hardware_abstractions.c
badd +458 src/BoardSupport/FRDM-KL25Z/kl25z_soc_hardware_abstractions.c
badd +101 inc/BoardSupport/FRDM-KL25Z/kl25z_soc_public.h
badd +77 src/Applications/McRTOS-FRDM/main.c
badd +56 ~/embsys/projects/McRTOS/inc/BoardSupport/FRDM-K64F/k64f_soc_public.h
badd +114 ~/embsys/projects/McRTOS/src/BoardSupport/LM4F120-LaunchPad/lm4f120_soc_hardware_abstractions.c
badd +1 src/BoardSupport/FRDM-K64F/k64f_soc_hardware_abstractions.c
badd +12 src/BoardSupport/frdm_board_hardware_abstractions.c
badd +18 inc/BoardSupport/frdm_board.h
badd +343 inc/BoardSupport/FRDM-K64F/k64f_soc.h
badd +4 src/Applications/McRTOS-FRDM/module.mk
badd +1 inc/BoardSupport/FRDM-K64F/MK64F12.h
badd +48 inc/BoardSupport/CMSIS/core_cm4.h
badd +115 ~/embsys/projects/McRTOS/inc/BoardSupport/FRDM-K20D50/k20d5_soc_public.h
badd +1 prj/K64F_SOC-flash.ld
badd +5 src/BoardSupport/FRDM-K64F/frdm_board_hardware_abstractions.c
badd +24 src/Applications/frdm_demo/main.c
badd +384 src/BoardSupport/LPC2478-STK/lpc2478_ethernet.c
badd +259 ~/ide/vimrc.vim
badd +498 ~/embsys/projects/McRTOS/inc/BoardSupport/LPC2478-STK/lpc2478_ethernet.h
badd +6 src/BoardSupport/FRDM-K64F/K64f_soc_enet.c
badd +1 src/BoardSupport/FRDM-K64F/k64f_soc_enet.c
badd +0 src/Applications/frdm_demo/module.mk
badd +44 src/McRTOS_tcpip/module.mk
badd +11 src/McRTOS_tcpip/McRTOS_tcpip.c
badd +15 inc/McRTOS_tcpip/McRTOS_tcpip.h
silent! argdel *
set lines=51 columns=205
winpos 0 31
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
exe '1resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 102 + 102) / 205)
exe '2resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 102 + 102) / 205)
exe '3resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 102 + 102) / 205)
exe '4resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 102 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 102 + 102) / 205)
exe '6resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 102 + 102) / 205)
exe '7resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 102 + 102) / 205)
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
let s:l = 188 - ((13 * winheight(0) + 11) / 22)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
188
normal! 014|
wincmd w
argglobal
edit prj/K64F_SOC-flash.ld
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 41 - ((15 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
41
normal! 0
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
let s:l = 19 - ((0 * winheight(0) + 4) / 9)
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
let s:l = 150 - ((5 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
150
normal! 039|
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
let s:l = 8 - ((5 * winheight(0) + 4) / 9)
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
let s:l = 19 - ((0 * winheight(0) + 4) / 9)
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
let s:l = 33 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
33
normal! 062|
wincmd w
7wincmd w
exe '1resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 102 + 102) / 205)
exe '2resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 102 + 102) / 205)
exe '3resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 102 + 102) / 205)
exe '4resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 102 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 102 + 102) / 205)
exe '6resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 102 + 102) / 205)
exe '7resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 102 + 102) / 205)
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
exe '1resize ' . ((&lines * 5 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 130 + 102) / 205)
exe '2resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 130 + 102) / 205)
exe '3resize ' . ((&lines * 13 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 130 + 102) / 205)
exe '4resize ' . ((&lines * 2 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 130 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 74 + 102) / 205)
exe '6resize ' . ((&lines * 27 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 74 + 102) / 205)
exe '7resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 74 + 102) / 205)
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
let s:l = 253 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
253
normal! 0
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
let s:l = 409 - ((0 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
409
normal! 0
wincmd w
argglobal
edit inc/McRTOS/McRTOS_arm_cortex_m.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 55 - ((0 * winheight(0) + 6) / 13)
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
let s:l = 2372 - ((0 * winheight(0) + 1) / 2)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
2372
normal! 0
wincmd w
argglobal
edit inc/McRTOS/McRTOS.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 34 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
34
normal! 03|
wincmd w
argglobal
edit inc/McRTOS/McRTOS_config_parameters.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 13) / 27)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit inc/McRTOS/McRTOS_internals.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 35 - ((0 * winheight(0) + 5) / 11)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
35
normal! 03|
wincmd w
7wincmd w
exe '1resize ' . ((&lines * 5 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 130 + 102) / 205)
exe '2resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 130 + 102) / 205)
exe '3resize ' . ((&lines * 13 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 130 + 102) / 205)
exe '4resize ' . ((&lines * 2 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 130 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 74 + 102) / 205)
exe '6resize ' . ((&lines * 27 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 74 + 102) / 205)
exe '7resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 74 + 102) / 205)
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
exe '1resize ' . ((&lines * 40 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 137 + 102) / 205)
exe '2resize ' . ((&lines * 8 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 137 + 102) / 205)
exe '3resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 67 + 102) / 205)
exe '4resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 67 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 67 + 102) / 205)
exe '6resize ' . ((&lines * 8 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 67 + 102) / 205)
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
let s:l = 3128 - ((0 * winheight(0) + 20) / 40)
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
let s:l = 1 - ((0 * winheight(0) + 4) / 8)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
edit inc/McRTOS/McRTOS_kernel_services.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 953 - ((0 * winheight(0) + 3) / 7)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
953
normal! 0
wincmd w
argglobal
edit inc/McRTOS/McRTOS_internals.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 797 - ((8 * winheight(0) + 11) / 22)
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
let s:l = 53 - ((0 * winheight(0) + 4) / 9)
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
7wincmd w
exe '1resize ' . ((&lines * 40 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 137 + 102) / 205)
exe '2resize ' . ((&lines * 8 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 137 + 102) / 205)
exe '3resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 67 + 102) / 205)
exe '4resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 67 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 67 + 102) / 205)
exe '6resize ' . ((&lines * 8 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 67 + 102) / 205)
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
exe '1resize ' . ((&lines * 41 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 124 + 102) / 205)
exe '2resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 80 + 102) / 205)
exe '3resize ' . ((&lines * 3 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 80 + 102) / 205)
exe '4resize ' . ((&lines * 2 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 80 + 102) / 205)
exe '5resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 80 + 102) / 205)
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
let s:l = 474 - ((4 * winheight(0) + 20) / 41)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
474
normal! 0
wincmd w
argglobal
edit inc/McRTOS/arm_defs.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 13 - ((0 * winheight(0) + 3) / 7)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
13
normal! 019|
wincmd w
argglobal
edit inc/McRTOS/arm_cortex_m_macros.s
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
let s:l = 30 - ((0 * winheight(0) + 1) / 2)
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
let s:l = 46 - ((0 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
46
normal! 0
wincmd w
7wincmd w
exe '1resize ' . ((&lines * 41 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 124 + 102) / 205)
exe '2resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 80 + 102) / 205)
exe '3resize ' . ((&lines * 3 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 80 + 102) / 205)
exe '4resize ' . ((&lines * 2 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 80 + 102) / 205)
exe '5resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 80 + 102) / 205)
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
exe '1resize ' . ((&lines * 29 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 143 + 102) / 205)
exe '2resize ' . ((&lines * 19 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 143 + 102) / 205)
exe '3resize ' . ((&lines * 31 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 61 + 102) / 205)
exe '4resize ' . ((&lines * 17 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 61 + 102) / 205)
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
let s:l = 32 - ((0 * winheight(0) + 14) / 29)
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
let s:l = 139 - ((0 * winheight(0) + 9) / 19)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
139
normal! 0
wincmd w
argglobal
edit inc/McRTOS/compile_time_checks.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 6 - ((0 * winheight(0) + 15) / 31)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 02|
wincmd w
argglobal
edit inc/McRTOS/failure_data_capture.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 6 - ((0 * winheight(0) + 8) / 17)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 02|
wincmd w
7wincmd w
exe '1resize ' . ((&lines * 29 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 143 + 102) / 205)
exe '2resize ' . ((&lines * 19 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 143 + 102) / 205)
exe '3resize ' . ((&lines * 31 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 61 + 102) / 205)
exe '4resize ' . ((&lines * 17 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 61 + 102) / 205)
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
exe '1resize ' . ((&lines * 41 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 142 + 102) / 205)
exe '2resize ' . ((&lines * 41 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 62 + 102) / 205)
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
let s:l = 330 - ((0 * winheight(0) + 20) / 41)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
330
normal! 0
wincmd w
argglobal
edit inc/McRTOS/utils.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 74 - ((0 * winheight(0) + 20) / 41)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
74
normal! 0
wincmd w
7wincmd w
exe '1resize ' . ((&lines * 41 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 142 + 102) / 205)
exe '2resize ' . ((&lines * 41 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 62 + 102) / 205)
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
4wincmd k
wincmd w
wincmd w
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 102 + 102) / 205)
exe '2resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 102 + 102) / 205)
exe '3resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 102 + 102) / 205)
exe '4resize ' . ((&lines * 12 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 102 + 102) / 205)
exe '5resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 102 + 102) / 205)
exe '6resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 102 + 102) / 205)
exe '7resize ' . ((&lines * 19 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 102 + 102) / 205)
exe '8resize ' . ((&lines * 12 + 25) / 51)
exe 'vert 8resize ' . ((&columns * 102 + 102) / 205)
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
let s:l = 1078 - ((0 * winheight(0) + 8) / 16)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1078
normal! 0
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
let s:l = 1 - ((0 * winheight(0) + 8) / 16)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
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
let s:l = 8 - ((0 * winheight(0) + 7) / 15)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
8
normal! 0
wincmd w
argglobal
edit inc/BoardSupport/hardware_abstractions.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 352 - ((0 * winheight(0) + 6) / 12)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
352
normal! 0
wincmd w
argglobal
edit inc/McRTOS/arm_defs.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 3 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
3
normal! 02|
wincmd w
argglobal
edit ~/embsys/projects/McRTOS/inc/BoardSupport/FRDM-K64F/k64f_soc_public.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 115 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
115
normal! 03|
wincmd w
argglobal
edit src/BoardSupport/FRDM-K64F/k64f_soc_hardware_abstractions.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1078 - ((18 * winheight(0) + 9) / 19)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1078
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
7wincmd w
exe '1resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 102 + 102) / 205)
exe '2resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 102 + 102) / 205)
exe '3resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 102 + 102) / 205)
exe '4resize ' . ((&lines * 12 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 102 + 102) / 205)
exe '5resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 102 + 102) / 205)
exe '6resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 102 + 102) / 205)
exe '7resize ' . ((&lines * 19 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 102 + 102) / 205)
exe '8resize ' . ((&lines * 12 + 25) / 51)
exe 'vert 8resize ' . ((&columns * 102 + 102) / 205)
tabedit inc/BoardSupport/FRDM-K64F/MK64F12.h
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
exe '1resize ' . ((&lines * 21 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 105 + 102) / 205)
exe '2resize ' . ((&lines * 14 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 105 + 102) / 205)
exe '3resize ' . ((&lines * 12 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 105 + 102) / 205)
exe '4resize ' . ((&lines * 39 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 99 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 99 + 102) / 205)
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
let s:l = 20 - ((16 * winheight(0) + 10) / 21)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
20
normal! 02|
wincmd w
argglobal
edit inc/McRTOS/McRTOS_arm_cortex_m.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 56 - ((0 * winheight(0) + 7) / 14)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
56
normal! 0
wincmd w
argglobal
edit inc/BoardSupport/CMSIS/core_cm4.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 372 - ((0 * winheight(0) + 6) / 12)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
372
normal! 017|
wincmd w
argglobal
edit inc/BoardSupport/CMSIS/core_cmInstr.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 663 - ((0 * winheight(0) + 19) / 39)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
663
normal! 0
wincmd w
argglobal
edit inc/BoardSupport/CMSIS/core_cmFunc.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 367 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
367
normal! 059|
wincmd w
7wincmd w
exe '1resize ' . ((&lines * 21 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 105 + 102) / 205)
exe '2resize ' . ((&lines * 14 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 105 + 102) / 205)
exe '3resize ' . ((&lines * 12 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 105 + 102) / 205)
exe '4resize ' . ((&lines * 39 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 99 + 102) / 205)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 99 + 102) / 205)
tabedit src/BoardSupport/LPC2478-STK/lpc2478_ethernet.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe 'vert 1resize ' . ((&columns * 102 + 102) / 205)
exe 'vert 2resize ' . ((&columns * 102 + 102) / 205)
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
let s:l = 511 - ((0 * winheight(0) + 24) / 49)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
511
normal! 025|
wincmd w
argglobal
edit src/BoardSupport/LPC2478-STK/lpc2478_ethernet.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 611 - ((45 * winheight(0) + 24) / 49)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
611
normal! 023|
wincmd w
7wincmd w
exe 'vert 1resize ' . ((&columns * 102 + 102) / 205)
exe 'vert 2resize ' . ((&columns * 102 + 102) / 205)
tabedit src/McRTOS_tcpip/McRTOS_tcpip.c
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
wincmd _ | wincmd |
split
1wincmd k
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe 'vert 1resize ' . ((&columns * 112 + 102) / 205)
exe '2resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 92 + 102) / 205)
exe '3resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 92 + 102) / 205)
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
let s:l = 35 - ((34 * winheight(0) + 24) / 49)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
35
normal! 018|
wincmd w
argglobal
edit inc/McRTOS_tcpip/McRTOS_tcpip.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 15 - ((7 * winheight(0) + 12) / 24)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
15
normal! 028|
wincmd w
argglobal
edit src/McRTOS_tcpip/module.mk
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 7 - ((6 * winheight(0) + 12) / 24)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
7
normal! 017|
wincmd w
7wincmd w
exe 'vert 1resize ' . ((&columns * 112 + 102) / 205)
exe '2resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 92 + 102) / 205)
exe '3resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 92 + 102) / 205)
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
