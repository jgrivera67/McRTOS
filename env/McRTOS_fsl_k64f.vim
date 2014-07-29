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
badd +228 src/build.mk
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
badd +915 src/BoardSupport/LPC2478-STK/lpc2478_hardware_abstractions.c
badd +195 inc/BoardSupport/hardware_abstractions.h
badd +25 inc/BoardSupport/LPC2478-STK/lpc2478_stk_board.h
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
badd +25 src/Applications/McRTOS-demo/main.c
badd +1 src/Applications/autonomous_car/main.c
badd +17 src/Applications/autonomous_car/module.mk
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
badd +1 inc/BoardSupport/frdm_board.h
badd +32 inc/BoardSupport/FRDM-K64F/k64f_soc.h
badd +4 src/Applications/McRTOS-FRDM/module.mk
badd +1 inc/BoardSupport/FRDM-K64F/MK64F12.h
badd +48 inc/BoardSupport/CMSIS/core_cm4.h
badd +115 ~/embsys/projects/McRTOS/inc/BoardSupport/FRDM-K20D50/k20d5_soc_public.h
badd +1 prj/K64F_SOC-flash.ld
badd +1 src/BoardSupport/FRDM-K64F/frdm_board_hardware_abstractions.c
silent! argdel *
set lines=53 columns=209
winpos 0 31
edit src/Applications/autonomous_car/main.c
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
exe '1resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 88 + 104) / 209)
exe '2resize ' . ((&lines * 31 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 88 + 104) / 209)
exe '3resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 120 + 104) / 209)
exe '4resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 120 + 104) / 209)
exe '5resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 120 + 104) / 209)
exe '6resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 120 + 104) / 209)
exe '7resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 7resize ' . ((&columns * 120 + 104) / 209)
exe '8resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 8resize ' . ((&columns * 120 + 104) / 209)
exe '9resize ' . ((&lines * 15 + 26) / 53)
exe 'vert 9resize ' . ((&columns * 120 + 104) / 209)
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
let s:l = 1 - ((0 * winheight(0) + 9) / 19)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
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
let s:l = 6 - ((0 * winheight(0) + 15) / 31)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
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
let s:l = 19 - ((0 * winheight(0) + 2) / 5)
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
let s:l = 78 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
78
normal! 020|
wincmd w
argglobal
edit src/Applications/autonomous_car/module.mk
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
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
let s:l = 19 - ((0 * winheight(0) + 2) / 5)
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
let s:l = 52 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
52
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
let s:l = 215 - ((0 * winheight(0) + 2) / 5)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
215
normal! 03|
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
5wincmd w
exe '1resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 88 + 104) / 209)
exe '2resize ' . ((&lines * 31 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 88 + 104) / 209)
exe '3resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 120 + 104) / 209)
exe '4resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 120 + 104) / 209)
exe '5resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 120 + 104) / 209)
exe '6resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 120 + 104) / 209)
exe '7resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 7resize ' . ((&columns * 120 + 104) / 209)
exe '8resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 8resize ' . ((&columns * 120 + 104) / 209)
exe '9resize ' . ((&lines * 15 + 26) / 53)
exe 'vert 9resize ' . ((&columns * 120 + 104) / 209)
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
exe '1resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 88 + 104) / 209)
exe '2resize ' . ((&lines * 27 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 88 + 104) / 209)
exe '3resize ' . ((&lines * 4 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 88 + 104) / 209)
exe '4resize ' . ((&lines * 2 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 88 + 104) / 209)
exe '5resize ' . ((&lines * 9 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 82 + 104) / 209)
exe '6resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 82 + 104) / 209)
exe '7resize ' . ((&lines * 11 + 26) / 53)
exe 'vert 7resize ' . ((&columns * 82 + 104) / 209)
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
let s:l = 409 - ((11 * winheight(0) + 13) / 27)
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
let s:l = 55 - ((0 * winheight(0) + 2) / 4)
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
let s:l = 1 - ((0 * winheight(0) + 9) / 19)
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
5wincmd w
exe '1resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 88 + 104) / 209)
exe '2resize ' . ((&lines * 27 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 88 + 104) / 209)
exe '3resize ' . ((&lines * 4 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 88 + 104) / 209)
exe '4resize ' . ((&lines * 2 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 88 + 104) / 209)
exe '5resize ' . ((&lines * 9 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 82 + 104) / 209)
exe '6resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 82 + 104) / 209)
exe '7resize ' . ((&lines * 11 + 26) / 53)
exe 'vert 7resize ' . ((&columns * 82 + 104) / 209)
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
exe '1resize ' . ((&lines * 32 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 95 + 104) / 209)
exe '2resize ' . ((&lines * 8 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 95 + 104) / 209)
exe '3resize ' . ((&lines * 7 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 75 + 104) / 209)
exe '4resize ' . ((&lines * 22 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 75 + 104) / 209)
exe '5resize ' . ((&lines * 1 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 75 + 104) / 209)
exe '6resize ' . ((&lines * 8 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 75 + 104) / 209)
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
let s:l = 3128 - ((0 * winheight(0) + 16) / 32)
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
normal! 021|
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
let s:l = 53 - ((0 * winheight(0) + 0) / 1)
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
5wincmd w
exe '1resize ' . ((&lines * 32 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 95 + 104) / 209)
exe '2resize ' . ((&lines * 8 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 95 + 104) / 209)
exe '3resize ' . ((&lines * 7 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 75 + 104) / 209)
exe '4resize ' . ((&lines * 22 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 75 + 104) / 209)
exe '5resize ' . ((&lines * 1 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 75 + 104) / 209)
exe '6resize ' . ((&lines * 8 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 75 + 104) / 209)
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
exe '1resize ' . ((&lines * 41 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 82 + 104) / 209)
exe '2resize ' . ((&lines * 7 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 88 + 104) / 209)
exe '3resize ' . ((&lines * 3 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 88 + 104) / 209)
exe '4resize ' . ((&lines * 2 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 88 + 104) / 209)
exe '5resize ' . ((&lines * 26 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 88 + 104) / 209)
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
5wincmd w
exe '1resize ' . ((&lines * 41 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 82 + 104) / 209)
exe '2resize ' . ((&lines * 7 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 88 + 104) / 209)
exe '3resize ' . ((&lines * 3 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 88 + 104) / 209)
exe '4resize ' . ((&lines * 2 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 88 + 104) / 209)
exe '5resize ' . ((&lines * 26 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 88 + 104) / 209)
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
exe '1resize ' . ((&lines * 21 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 101 + 104) / 209)
exe '2resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 101 + 104) / 209)
exe '3resize ' . ((&lines * 23 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 69 + 104) / 209)
exe '4resize ' . ((&lines * 17 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 69 + 104) / 209)
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
let s:l = 32 - ((0 * winheight(0) + 10) / 21)
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
let s:l = 6 - ((0 * winheight(0) + 11) / 23)
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
5wincmd w
exe '1resize ' . ((&lines * 21 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 101 + 104) / 209)
exe '2resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 101 + 104) / 209)
exe '3resize ' . ((&lines * 23 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 69 + 104) / 209)
exe '4resize ' . ((&lines * 17 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 69 + 104) / 209)
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
exe '1resize ' . ((&lines * 41 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 100 + 104) / 209)
exe '2resize ' . ((&lines * 41 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 70 + 104) / 209)
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
5wincmd w
exe '1resize ' . ((&lines * 41 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 100 + 104) / 209)
exe '2resize ' . ((&lines * 41 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 70 + 104) / 209)
tabedit src/BoardSupport/FRDM-K64F/k64f_soc_hardware_abstractions.c
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
exe '1resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 81 + 104) / 209)
exe '2resize ' . ((&lines * 21 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 81 + 104) / 209)
exe '3resize ' . ((&lines * 7 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 89 + 104) / 209)
exe '4resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 89 + 104) / 209)
exe '5resize ' . ((&lines * 1 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 89 + 104) / 209)
exe '6resize ' . ((&lines * 8 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 89 + 104) / 209)
exe '7resize ' . ((&lines * 16 + 26) / 53)
exe 'vert 7resize ' . ((&columns * 89 + 104) / 209)
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
let s:l = 539 - ((0 * winheight(0) + 9) / 19)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
539
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
let s:l = 1 - ((0 * winheight(0) + 10) / 21)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
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
let s:l = 352 - ((0 * winheight(0) + 3) / 7)
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
let s:l = 3 - ((0 * winheight(0) + 2) / 5)
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
edit inc/BoardSupport/FRDM-K64F/k64f_soc.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 32 - ((0 * winheight(0) + 4) / 8)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
32
normal! 025|
wincmd w
argglobal
edit inc/BoardSupport/frdm_board.h
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 13 - ((0 * winheight(0) + 8) / 16)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
13
normal! 0
wincmd w
5wincmd w
exe '1resize ' . ((&lines * 19 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 81 + 104) / 209)
exe '2resize ' . ((&lines * 21 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 81 + 104) / 209)
exe '3resize ' . ((&lines * 7 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 89 + 104) / 209)
exe '4resize ' . ((&lines * 5 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 89 + 104) / 209)
exe '5resize ' . ((&lines * 1 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 89 + 104) / 209)
exe '6resize ' . ((&lines * 8 + 26) / 53)
exe 'vert 6resize ' . ((&columns * 89 + 104) / 209)
exe '7resize ' . ((&lines * 16 + 26) / 53)
exe 'vert 7resize ' . ((&columns * 89 + 104) / 209)
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
exe '1resize ' . ((&lines * 21 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 75 + 104) / 209)
exe '2resize ' . ((&lines * 6 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 75 + 104) / 209)
exe '3resize ' . ((&lines * 12 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 75 + 104) / 209)
exe '4resize ' . ((&lines * 31 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 95 + 104) / 209)
exe '5resize ' . ((&lines * 9 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 95 + 104) / 209)
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
let s:l = 56 - ((0 * winheight(0) + 3) / 6)
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
let s:l = 663 - ((0 * winheight(0) + 15) / 31)
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
5wincmd w
exe '1resize ' . ((&lines * 21 + 26) / 53)
exe 'vert 1resize ' . ((&columns * 75 + 104) / 209)
exe '2resize ' . ((&lines * 6 + 26) / 53)
exe 'vert 2resize ' . ((&columns * 75 + 104) / 209)
exe '3resize ' . ((&lines * 12 + 26) / 53)
exe 'vert 3resize ' . ((&columns * 75 + 104) / 209)
exe '4resize ' . ((&lines * 31 + 26) / 53)
exe 'vert 4resize ' . ((&columns * 95 + 104) / 209)
exe '5resize ' . ((&lines * 9 + 26) / 53)
exe 'vert 5resize ' . ((&columns * 95 + 104) / 209)
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
