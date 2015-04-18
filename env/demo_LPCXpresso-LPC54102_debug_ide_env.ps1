$env:platform = "LPCXpresso-LPC54102"
$env:application = "demo"
$env:build_flavor = "debug"

. "$env:userprofile/MyProjects/EMBSYS/projects/McRTOS/env/McRTOS_common_ide_env.ps1"

function my_update_flash
{
    echo "copying $env:elf_file to $env:PLATFORM flash ..."
    C:\nxp\LPCXpresso_7.6.2_326\lpcxpresso\bin\crt_emu_cm_redlink -pLPC54100 -vendor=NXP -flash-load-exec "$env:elf_file"
}

