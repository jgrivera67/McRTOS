$env:platform = "FRDM-K64F"
$env:application = "frdm_demo"
$env:build_flavor = "debug"

. "$env:userprofile/MyProjects/EMBSYS/projects/McRTOS/env/McRTOS_common_ide_env.ps1"

function my_update_flash
{
    echo "copying $env:bin_file to $env:PLATFORM flash (drive E:\) ..."
    copy-item "$env:bin_file" e:\
}

function my_update_flash2
{
    echo "copying $env:bin_file to $env:PLATFORM flash (drive F:\) ..."
    copy-item "$env:bin_file" f:\
}
