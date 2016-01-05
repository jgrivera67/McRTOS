$base_dir = "$env:userprofile/MyProjects/EMBSYS/projects"
$project = "McRTOS"
$env:src_tree_dir = "$base_dir\$project"
$env:src_db_dir = "$base_dir\$project`_src_database"
$env:src_subdirs = ""
$env:cscope_db = $env:src_db_dir + "\cscope.out"
$env:cscope_root_dir = $env:src_tree_dir
$env:cscope_root_dir_exports = ""
$env:ide_env_dir = "$base_dir/$project/env"
$env:ide_shell_rc_file="$env:ide_env_dir/$env:application`_$env:platform`_$env:build_flavor`_ide_env.ps1"
$env:ide_vim_session = "$env:ide_env_dir/$env:application`_$env:platform.vim"
$env:default_obj_flavor_subdir = "$env:platform`-obj-$env:build_flavor"
$env:bin_file_prefix = "$env:default_obj_flavor_subdir/Applications/$env:application/$env:application"
$env:bin_file = "$env:bin_file_prefix.bin"
$env:elf_file = "$env:bin_file_prefix.elf"

$env:makefile_dir = "$env:src_tree_dir"

$env:make = "C:\MinGW\msys\1.0\bin\make.exe"

function my_build_all
{
    run_build build
}

function my_clean_all
{
    run_build clean
}

function my_rebuild_all
{
	run_build rebuild
}

function run_build([string]$target)
{

    if ($target -eq "") {
        echo "Usage: run_build <make target>"
        return 1
    }

       if ((test-path "$MyHomeDir\tmp\make.log")) {
        remove-item -force "$MyHomeDir\tmp\make.log"
    }

    if ((test-path env:\makefile_dir)) {
        $makefileDir = $env:makefile_dir
    } else {
        $makefileDir = $pwd
    }

    if (! (test-path "$makefileDir\Makefile")) {
        echo "*** ERROR: A Makefile does not exist at $makefileDir"
        return 1
    }

    if ((test-path "$MyHomeDir\tmp\make.log")) {
        remove-item -force "$MyHomeDir\tmp\make.log"
    }

    cd $makefileDir

    echo "*** Building $env:application for platform $env:platform, flavor $env:build_flavor ..."

    $make_args = "$target",
                 "--jobs=2",
                 "APPLICATION=$env:application",
                 "PLATFORM=$env:platform",
                 "BUILD_FLAVOR=$env:build_flavor"

    start-process make $make_args `
        -RedirectStandardError $MyHomeDir\tmp\make.log `
        -NoNewWindow -wait

    if (! $?) {
       echo " *** Error: starting make failed with error $LastExitCode"
       return $LastExitCode
    }

    #echo "*** Make error log: ***"
    #cat $MyHomeDir\tmp\make.log
    $num_warnings = $(grep -c '[Ww]arning:' $MyHomeDir\tmp\make.log)
    $num_errors = $(grep -c '[Ee]rror:' $MyHomeDir\tmp\make.log)
    $num_todos = $(grep -c 'TODO:' $MyHomeDir\tmp\make.log)
    echo "*** Total number of errors: $num_errors ***"
    echo "*** Total number of warnings: $num_warnings ***"
    echo "*** Total number of TODOs: $num_todos ***"
}

function my_stack([string]$raw_stack_trace_file)
{
    if ($raw_stack_trace_file -eq "") {
        echo "Usage: my_stack <raw stack trace file>"
        return 1
    }

    cat $raw_stack_trace_file | perl scripts/print_stack_trace.pl $env:bin_file_prefix`.elf
}

cd $env:src_tree_dir
#gvim -S $env:ide_vim_session
