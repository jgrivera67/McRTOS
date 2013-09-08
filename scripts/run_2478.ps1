param(
    [string]$bin_file_prefix, 
    [switch]$updateFlash
)

function run2478([string]$bin_file_prefix, [switch]$updateFlash)
{
    $usage_str = "Usage: run2478 <ARM binary file prefix> [-updateFlash]"

    if ($bin_file_prefix -eq "") {
        echo $usage_str
        return 1
    }

    #$bin_file_prefix = $bin_file_prefix -replace '\.\\(\S+)\..*', '$1'
    $bin_file_prefix = $bin_file_prefix -replace '\\', '/'
    $bin_file_prefix = $bin_file_prefix -replace '(\S+)\..*', '$1'

    $hex_file = "$bin_file_prefix.hex"
    $elf_file = "$bin_file_prefix.elf"

    if (! (test-path $hex_file)) {
        echo " *** Error: $hex_file not found"
            return 1
    }

    if (! (test-path $elf_file)) {
        echo " *** Error: $elf_file not found"
        return 1
    }

    #
    # Generate gdb init script:
    #

    $init_script = "$bin_file_prefix.gdb"

    $dbg_cmds = @"
target remote localhost:3333
show remotetimeout
set remotetimeout 180
monitor arm7_9 dcc_downloads enable
monitor arm7_9 fast_memory_access enable
monitor reset init
monitor wait_halt
monitor poll

"@

    if ($updateFlash)
    {
        $dbg_cmds += @"
monitor flash probe 0
monitor flash erase_sector 0 0 26
monitor reset init
monitor wait_halt
monitor poll
monitor flash write_image erase $hex_file 0 ihex

"@
    }

    $dbg_cmds += @"
symbol-file $elf_file
monitor soft_reset_halt
set remotetimeout 10
set print pretty on
set print array on
set print array-indexes on
set radix 16
#break main
#continue
"@

    $dbg_cmds | out-file $init_script -encoding ascii

    #
    # Stop any old gdb GUI
    # 
    $gdb_proc = get-process | where { $_.name -eq "arm-elf-insight" }
    if ($gdb_proc -ne $null) {
        echo "*** Stopping old arm-elf-insight.exe ..."
        Stop-process -InputObject $gdb_proc
    }

    #
    # Stop any old openocd-libftdi
    # 
    $gdb_proc = get-process | where { $_.name -eq "openocd-libftdi" }
    if ($gdb_proc -ne $null) {
        echo "*** Stopping old openocd-libftdi.exe ..."
        Stop-process -InputObject $gdb_proc
    }

    #
    # Run TeraTerm if not already running:
    #
    $openocd_proc = get-process | where { $_.name -eq "ttermpro" }
    if ($openocd_proc -eq $null) {
        echo "*** Starting ttermpro.exe ..."
        Start-process `
            -filepath "$embsys_apps_dir\TeraTerm\TTermPro\ttermpro.exe"

        if (! $?) {
            echo " *** Error: starting ttermpro failed with error $LastExitCode"
            return $LastExitCode
        }
    }

    #
    # Run openocd-libftdi
    #
    echo "*** Starting openocd-libftdi.exe ..."
    Start-process `
        -filepath "$embsys_apps_dir\openocd-r717\bin\openocd-libftdi.exe" `
            -argumentlist --file,"$embsys_apps_dir\openocd-r717\bin\scripts\lpc2478\lpc2478_armusbocdh.cfg" `
            #-NoNewWindow

    if (! $?) {
        echo " *** Error: starting openocd-libftdi failed with error $LastExitCode"
        return $LastExitCode
    }

    #
    # Run gdb GUI
    #
    echo "*** Starting arm-elf-insight.exe for $elf_file ..."
    Start-process `
        -filepath "$embsys_apps_dir\yagarto-old\bin\arm-elf-insight.exe" `
        -argumentlist --command=$init_script `
        -NoNewWindow

    #&"$embsys_apps_dir\yagarto\bin\arm-elf-gdb.exe" --command=$init_script
    #adg --debugger "$embsys_apps_dir\yagarto\bin\arm-elf-gdb.exe" --command=$init_script

    if (! $?) {
        echo " *** Error: starting arm-elf-insight failed with error $LastExitCode"
        return $LastExitCode
    }
}


run2478 $bin_file_prefix $updateFlash
