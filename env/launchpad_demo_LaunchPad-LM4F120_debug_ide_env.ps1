$env:platform = "LaunchPad-LM4F120"
$env:application = "launchpad_demo"
$env:build_flavor = "debug"

. "$env:userprofile/MyProjects/EMBSYS/projects/McRTOS/env/McRTOS_common_ide_env.ps1"

function my_update_flash
{
    echo "copying $env:bin_file to $env:PLATFORM flash ..."
    ~\MyProjects\EMBSYS\Stellaris\tools\LMFlash.exe --quick-set=ek-lm4f120xl --verify --reset "$env:bin_file"
}

