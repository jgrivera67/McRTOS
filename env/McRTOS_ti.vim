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
badd +158 src/build.mk
badd +11 src/Applications/LPC2478-STK/McRTOS-demo/module.mk
badd +5 src/McRTOS/module.mk
badd +25 src/BoardSupport/module.mk
badd +299 src/McRTOS/McRTOS_startup.c
badd +116 inc/McRTOS/McRTOS.h
badd +162 inc/McRTOS/McRTOS_internals.h
badd +125 inc/McRTOS/McRTOS_config_parameters.h
badd +2856 src/McRTOS/McRTOS_kernel_services.c
badd +15 src/McRTOS/McRTOS_crt_armv4.s
badd +508 src/McRTOS/McRTOS_kernel_services_armv4.s
badd +831 inc/McRTOS/McRTOS_kernel_services.h
badd +1 src/McRTOS/McRTOS_execution_controller.c
badd +318 src/BoardSupport/LPC2478-STK/lpc2478_interrupt_handlers.s
badd +4 src/McRTOS/McRTOS_interrupt_handlers_armv4.s
badd +137 src/McRTOS/McRTOS_interrupt_service_routines_armv4.s
badd +1 src/McRTOS/failure_data_capture.c
badd +13 inc/McRTOS/failure_data_capture.h
badd +46 inc/McRTOS/compile_time_checks.h
badd +73 src/McRTOS/run_time_exception_handlers_armv4.s
badd +342 src/McRTOS/McRTOS_system_call_wrappers_armv4.s
badd +1 src/McRTOS/utils.c
badd +38 inc/McRTOS/utils.h
badd +915 src/BoardSupport/LPC2478-STK/lpc2478_hardware_abstractions.c
badd +108 inc/BoardSupport/hardware_abstractions.h
badd +25 inc/BoardSupport/LPC2478-STK/lpc2478_stk_board.h
badd +137 inc/BoardSupport/arm_defs.h
badd +2 inc/BoardSupport/LPC2478-STK/lpc2478.h
badd +130 inc/BoardSupport/LPC2478-STK/lpc2478_stk_board_public.h
badd +6 inc/BoardSupport/LPC2478-STK/lpc2478_arm_defs.h
badd +215 inc/BoardSupport/LPC2478-STK/lpc2478_vic.h
badd +11 src/BoardSupport/LPC2478-STK/lpc2478_lcd.c
badd +6 src/BoardSupport/LPC2478-STK/lpc2478_touch_screen.c
badd +4 inc/BoardSupport/LPC2478-STK/lpc2478_lcd.h
badd +9 inc/BoardSupport/lcd.h
badd +286 inc/McRTOS/arm_defs.h
badd +9 src/Applications/McRTOS-demo/module.mk
badd +293 src/Applications/McRTOS-demo/main.c
badd +17 src/Applications/autonomous_car/main.c
badd +17 src/Applications/autonomous_car/module.mk
badd +66 src/McRTOS/TMP_startup_ARMCM0plus.S
badd +1 prj/LPC2478-STK-flash.ld
badd +1 prj/FRDM-KL25Z-flash.ld
badd +52 prj/TMP_gcc_arm.ld
badd +2 inc/BoardSupport/CMSIS/core_cm0plus.h
badd +336 inc/BoardSupport/CMSIS/core_cmInstr.h
badd +340 inc/BoardSupport/CMSIS/core_cmFunc.h
badd +3432 inc/BoardSupport/FRDM-KL25Z/MKL25Z4.h
badd +31 prj/temp.ld
badd +1 src/McRTOS/McRTOS_crt_armv6-m.S
badd +144 src/McRTOS/McRTOS_startup_arm_cortex_m.c
badd +513 src/BoardSupport/FRDM-KL25Z/frdm_kl25z_hardware_abstractions.c
badd +87 inc/McRTOS/McRTOS_startup_arm_cortex_m.h
badd +101 inc/McRTOS/McRTOS_arm_cortex_m.h
badd +55 inc/BoardSupport/FRDM-KL25Z/kl25z_soc.h
badd +112 prj/KL25Z_SOC-flash.ld
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
badd +1 src/McRTOS/McRTOS_run_time_exception_handlers_arm_cortex_m.s
badd +1 src/McRTOS/McRTOS_system_call_wrappers_arm_cortex_m.s
badd +141 src/McRTOS/McRTOS_debugger.c
badd +1 src/McRTOS/McRTOS_kernel_services_arm_cortex_m.s
badd +1 inc/McRTOS/arm_cortex_m_macros.s
badd +22 inc/BoardSupport/FRDM-KL25Z/frdm_board.h
badd +11 inc/BoardSupport/FRDM-KL25Z/tfc_board.h
badd +25 src/BoardSupport/FRDM-KL25Z/tfc_board_hardware_abstractions.c
badd +87 src/BoardSupport/FRDM-KL25Z/frdm_board_hardware_abstractions.c
badd +15 src/BoardSupport/FRDM-KL25Z/kl25z_hardware_abstractions.c
badd +523 src/BoardSupport/FRDM-KL25Z/kl25z_soc_hardware_abstractions.c
badd +145 inc/BoardSupport/FRDM-KL25Z/kl25z_soc_public.h
badd +1 src/BoardSupport/LM4F120-LaunchPad/lm4f120_soc_hardware_abstractions.c
badd +75 src/BoardSupport/LM4F120-LaunchPad/launchpad_board_hardware_abstractions.c
badd +141 inc/BoardSupport/LM4F120-LaunchPad/lm4f120_soc_public.h
badd +12 inc/BoardSupport/LM4F120-LaunchPad/lm4f120_soc.h
badd +34 inc/BoardSupport/LM4F120-LaunchPad/launchpad_board.h
badd +21 prj/LM24F120_SOC-flash.ld
badd +1 prj/LM4F120_SOC-flash.ld
badd +703 inc/BoardSupport/LM4F120-LaunchPad/tivaware/tm4c123gh6pm.h
badd +1 inc/BoardSupport/LM4F120-LaunchPad/tivaware/rom.h
badd +153 inc/BoardSupport/LM4F120-LaunchPad/tivaware/hw_memmap.h
badd +1 src/Applications/McRTOS-launchpad/main.c
badd +5 src/Applications/McRTOS-launchpad/module.mk
badd +1379 inc/BoardSupport/CMSIS/core_cm4.h
badd +34 inc/BoardSupport/LM4F120-LaunchPad/tivaware/hw_gpio.h
silent! argdel *
set lines=51 columns=191
winpos 18 31
edit src/Applications/McRTOS-launchpad/main.c
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
wincmd _ | wincmd |
split
6wincmd k
wincmd w
wincmd w
wincmd w
wincmd w
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 91 + 95) / 191)
exe '2resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 91 + 95) / 191)
exe '3resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 91 + 95) / 191)
exe '4resize ' . ((&lines * 5 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 99 + 95) / 191)
exe '5resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 99 + 95) / 191)
exe '6resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 99 + 95) / 191)
exe '7resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 99 + 95) / 191)
exe '8resize ' . ((&lines * 23 + 25) / 51)
exe 'vert 8resize ' . ((&columns * 99 + 95) / 191)
exe '9resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 9resize ' . ((&columns * 99 + 95) / 191)
exe '10resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 10resize ' . ((&columns * 99 + 95) / 191)
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
let s:l = 124 - ((0 * winheight(0) + 8) / 16)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
124
normal! 0
wincmd w
argglobal
edit src/McRTOS/McRTOS_startup_arm_cortex_m.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 144 - ((0 * winheight(0) + 8) / 16)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
144
normal! 032|
wincmd w
argglobal
edit prj/LM4F120_SOC-flash.ld
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 7) / 15)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
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
let s:l = 9 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
9
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
let s:l = 8 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
8
normal! 08|
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
let s:l = 201 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
201
normal! 03|
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
let s:l = 18 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
18
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
let s:l = 32 - ((0 * winheight(0) + 11) / 23)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
32
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
exe '1resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 91 + 95) / 191)
exe '2resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 91 + 95) / 191)
exe '3resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 91 + 95) / 191)
exe '4resize ' . ((&lines * 5 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 99 + 95) / 191)
exe '5resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 99 + 95) / 191)
exe '6resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 99 + 95) / 191)
exe '7resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 99 + 95) / 191)
exe '8resize ' . ((&lines * 23 + 25) / 51)
exe 'vert 8resize ' . ((&columns * 99 + 95) / 191)
exe '9resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 9resize ' . ((&columns * 99 + 95) / 191)
exe '10resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 10resize ' . ((&columns * 99 + 95) / 191)
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
exe '1resize ' . ((&lines * 6 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 104 + 95) / 191)
exe '2resize ' . ((&lines * 10 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 104 + 95) / 191)
exe '3resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 104 + 95) / 191)
exe '4resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 104 + 95) / 191)
exe '5resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 86 + 95) / 191)
exe '6resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 86 + 95) / 191)
exe '7resize ' . ((&lines * 14 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 86 + 95) / 191)
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
let s:l = 257 - ((2 * winheight(0) + 3) / 6)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
257
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
let s:l = 739 - ((0 * winheight(0) + 5) / 10)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
739
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
let s:l = 55 - ((0 * winheight(0) + 7) / 15)
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
let s:l = 2372 - ((0 * winheight(0) + 7) / 15)
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
let s:l = 34 - ((0 * winheight(0) + 5) / 11)
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
let s:l = 29 - ((0 * winheight(0) + 11) / 22)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
29
normal! 029|
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
let s:l = 35 - ((0 * winheight(0) + 7) / 14)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
35
normal! 03|
wincmd w
exe '1resize ' . ((&lines * 6 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 104 + 95) / 191)
exe '2resize ' . ((&lines * 10 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 104 + 95) / 191)
exe '3resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 104 + 95) / 191)
exe '4resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 104 + 95) / 191)
exe '5resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 86 + 95) / 191)
exe '6resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 86 + 95) / 191)
exe '7resize ' . ((&lines * 14 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 86 + 95) / 191)
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
2wincmd k
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winheight=1 winwidth=1
exe '1resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 111 + 95) / 191)
exe '2resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 111 + 95) / 191)
exe '3resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 79 + 95) / 191)
exe '4resize ' . ((&lines * 20 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 79 + 95) / 191)
exe '5resize ' . ((&lines * 18 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 79 + 95) / 191)
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
let s:l = 2866 - ((0 * winheight(0) + 12) / 24)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
2866
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
let s:l = 27 - ((1 * winheight(0) + 12) / 24)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
27
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
let s:l = 953 - ((0 * winheight(0) + 4) / 9)
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
let s:l = 788 - ((1 * winheight(0) + 10) / 20)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
788
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
let s:l = 1 - ((0 * winheight(0) + 9) / 18)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
exe '1resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 111 + 95) / 191)
exe '2resize ' . ((&lines * 24 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 111 + 95) / 191)
exe '3resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 79 + 95) / 191)
exe '4resize ' . ((&lines * 20 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 79 + 95) / 191)
exe '5resize ' . ((&lines * 18 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 79 + 95) / 191)
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
exe 'vert 1resize ' . ((&columns * 93 + 95) / 191)
exe '2resize ' . ((&lines * 8 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 97 + 95) / 191)
exe '3resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 97 + 95) / 191)
exe '4resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 97 + 95) / 191)
exe '5resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 97 + 95) / 191)
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
let s:l = 15 - ((0 * winheight(0) + 20) / 41)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
15
normal! 03|
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
let s:l = 13 - ((0 * winheight(0) + 4) / 8)
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
let s:l = 1 - ((0 * winheight(0) + 3) / 7)
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
let s:l = 30 - ((0 * winheight(0) + 3) / 7)
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
let s:l = 17 - ((0 * winheight(0) + 8) / 16)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
17
normal! 0
wincmd w
exe '1resize ' . ((&lines * 41 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 93 + 95) / 191)
exe '2resize ' . ((&lines * 8 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 97 + 95) / 191)
exe '3resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 97 + 95) / 191)
exe '4resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 97 + 95) / 191)
exe '5resize ' . ((&lines * 16 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 97 + 95) / 191)
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
exe '1resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 117 + 95) / 191)
exe '2resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 117 + 95) / 191)
exe '3resize ' . ((&lines * 28 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 73 + 95) / 191)
exe '4resize ' . ((&lines * 20 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 73 + 95) / 191)
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
let s:l = 32 - ((0 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
32
normal! 061|
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
let s:l = 139 - ((0 * winheight(0) + 11) / 22)
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
let s:l = 6 - ((0 * winheight(0) + 14) / 28)
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
let s:l = 6 - ((0 * winheight(0) + 10) / 20)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 02|
wincmd w
exe '1resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 117 + 95) / 191)
exe '2resize ' . ((&lines * 22 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 117 + 95) / 191)
exe '3resize ' . ((&lines * 28 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 73 + 95) / 191)
exe '4resize ' . ((&lines * 20 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 73 + 95) / 191)
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
exe '1resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 146 + 95) / 191)
exe '2resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 44 + 95) / 191)
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
let s:l = 330 - ((0 * winheight(0) + 13) / 26)
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
let s:l = 74 - ((0 * winheight(0) + 13) / 26)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
74
normal! 0
wincmd w
exe '1resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 146 + 95) / 191)
exe '2resize ' . ((&lines * 26 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 44 + 95) / 191)
tabedit src/BoardSupport/LM4F120-LaunchPad/lm4f120_soc_hardware_abstractions.c
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
exe '1resize ' . ((&lines * 23 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 80 + 95) / 191)
exe '2resize ' . ((&lines * 25 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 80 + 95) / 191)
exe '3resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 110 + 95) / 191)
exe '4resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 110 + 95) / 191)
exe '5resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 110 + 95) / 191)
exe '6resize ' . ((&lines * 17 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 110 + 95) / 191)
exe '7resize ' . ((&lines * 3 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 110 + 95) / 191)
exe '8resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 8resize ' . ((&columns * 110 + 95) / 191)
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
let s:l = 412 - ((10 * winheight(0) + 11) / 23)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
412
normal! 022|
wincmd w
argglobal
edit src/BoardSupport/LM4F120-LaunchPad/launchpad_board_hardware_abstractions.c
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 29 - ((15 * winheight(0) + 12) / 25)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
29
normal! 027|
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
let s:l = 47 - ((6 * winheight(0) + 3) / 7)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
47
normal! 019|
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
let s:l = 395 - ((0 * winheight(0) + 0) / 1)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
395
normal! 02|
wincmd w
argglobal
edit inc/BoardSupport/LM4F120-LaunchPad/lm4f120_soc_public.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 141 - ((12 * winheight(0) + 7) / 15)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
141
normal! 0
wincmd w
argglobal
edit inc/BoardSupport/LM4F120-LaunchPad/lm4f120_soc.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 13 - ((6 * winheight(0) + 8) / 17)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
13
normal! 030|
wincmd w
argglobal
edit inc/BoardSupport/LM4F120-LaunchPad/launchpad_board.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 39 - ((2 * winheight(0) + 1) / 3)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
39
normal! 032|
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
exe '1resize ' . ((&lines * 23 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 80 + 95) / 191)
exe '2resize ' . ((&lines * 25 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 80 + 95) / 191)
exe '3resize ' . ((&lines * 7 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 110 + 95) / 191)
exe '4resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 110 + 95) / 191)
exe '5resize ' . ((&lines * 15 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 110 + 95) / 191)
exe '6resize ' . ((&lines * 17 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 110 + 95) / 191)
exe '7resize ' . ((&lines * 3 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 110 + 95) / 191)
exe '8resize ' . ((&lines * 1 + 25) / 51)
exe 'vert 8resize ' . ((&columns * 110 + 95) / 191)
tabedit inc/BoardSupport/LM4F120-LaunchPad/tivaware/rom.h
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
wincmd _ | wincmd |
split
4wincmd k
wincmd w
wincmd w
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
exe '1resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 115 + 95) / 191)
exe '2resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 115 + 95) / 191)
exe '3resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 115 + 95) / 191)
exe '4resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 115 + 95) / 191)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 115 + 95) / 191)
exe '6resize ' . ((&lines * 37 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 75 + 95) / 191)
exe '7resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 75 + 95) / 191)
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
let s:l = 35 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
35
normal! 0
wincmd w
argglobal
edit inc/BoardSupport/LM4F120-LaunchPad/tivaware/hw_memmap.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 68 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
68
normal! 052|
wincmd w
argglobal
edit inc/BoardSupport/LM4F120-LaunchPad/tivaware/tm4c123gh6pm.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 706 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
706
normal! 067|
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
let s:l = 26 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
26
normal! 07|
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
let s:l = 1 - ((0 * winheight(0) + 4) / 9)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
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
let s:l = 366 - ((0 * winheight(0) + 18) / 37)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
366
normal! 044|
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
let s:l = 367 - ((0 * winheight(0) + 5) / 11)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
367
normal! 059|
wincmd w
3wincmd w
exe '1resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 1resize ' . ((&columns * 115 + 95) / 191)
exe '2resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 2resize ' . ((&columns * 115 + 95) / 191)
exe '3resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 3resize ' . ((&columns * 115 + 95) / 191)
exe '4resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 4resize ' . ((&columns * 115 + 95) / 191)
exe '5resize ' . ((&lines * 9 + 25) / 51)
exe 'vert 5resize ' . ((&columns * 115 + 95) / 191)
exe '6resize ' . ((&lines * 37 + 25) / 51)
exe 'vert 6resize ' . ((&columns * 75 + 95) / 191)
exe '7resize ' . ((&lines * 11 + 25) / 51)
exe 'vert 7resize ' . ((&columns * 75 + 95) / 191)
tabnext 8
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
