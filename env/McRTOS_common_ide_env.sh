base_dir=$HOME/embsys/projects
project="McRTOS"
export SRC_TREE_DIR="$base_dir/$project"
export SRC_DB_DIR="$base_dir/${project}_src_database"
export SRC_SUBDIRS=""
export CSCOPE_DB="$SRC_DB_DIR/cscope.out"
export CSCOPE_ROOT_DIR=$SRC_TREE_DIR
export CSCOPE_ROOT_DIR_EXPORTS=""
export IDE_ENV_DIR=$base_dir/$project/env
export IDE_SHELL_RC_FILE=$IDE_ENV_DIR/${APPLICATION}_${PLATFORM}_${BUILD_FLAVOR}_ide_env.sh
export IDE_VIM_SESSION="$IDE_ENV_DIR/${APPLICATION}_${PLATFORM}.vim"
export DEFAULT_OBJ_FLAVOR_SUBDIR="${PLATFORM}-obj-${BUILD_FLAVOR}"
export BIN_FILE="$DEFAULT_OBJ_FLAVOR_SUBDIR/Applications/$APPLICATION/$APPLICATION.bin"
export ELF_FILE="$DEFAULT_OBJ_FLAVOR_SUBDIR/Applications/$APPLICATION/$APPLICATION.elf"
#export CROSS_COMPILE=$HOME/embsys/tools/gcc-arm-none-eabi-4_8-2013q4
#export CROSS_COMPILE=$HOME/embsys/tools/gcc-arm-none-eabi-4_8-2014q2
export CROSS_COMPILE=$HOME/embsys/tools/gcc-arm-none-eabi-4_9-2015q1
export GCC_VERSION=4.8.4
export TOOLCHAIN=$CROSS_COMPILE/bin/arm-none-eabi
export EXTERNAL_INCLUDE_DIRS="$CROSS_COMPILE/arm-none-eabi/include
			      $CROSS_COMPILE/lib/gcc/arm-none-eabi/*/include"
export MAKEFILE_DIR="$SRC_TREE_DIR"

function my_build_all
{
	run_build build
}

function my_rebuild_all
{
	run_build rebuild
}

function run_build
{
	typeset status
	typeset target

	if [ $# != 1 ]; then
		echo "Usage: $FUNCNAME <make target>"
		return 1
	fi

	target=$1
	cd $MAKEFILE_DIR
	echo "*** Building $APPLICATION for platform $PLATFORM, flavor $BUILD_FLAVOR ..."
	make $target 2>&1 | tee ~/tmp/make.$$.log
	status=$?
	if [ $status != 0 ]; then
		echo "*** Build failed with error $status ***"
	else
		echo "*** Build succeeded ***"
	fi

	num_warnings=$(grep -c '[Ww]arning:' ~/tmp/make.$$.log)
	num_errors=$(grep -c '[Ee]rror:' ~/tmp/make.$$.log)
	num_todos=$(grep -c 'TODO:' ~/tmp/make.$$.log)
	echo "*** Total number of errors: $num_errors ***"
	echo "*** Total number of warnings: $num_warnings ***"
	echo "*** Total number of TODOs: $num_todos ***"
	return $status
}


function my_gdb
{
    typeset init_script
    typeset gdb_log_file
    typeset gdb_history_file
    typeset gdb_commands
    typeset openocd_cfg_file
    typeset openocd_log_file
    typeset bin_file_prefix

    bin_file_prefix=$DEFAULT_OBJ_FLAVOR_SUBDIR/Applications/$APPLICATION/$APPLICATION
    init_script=$bin_file_prefix.gdb
    gdb_log_file=~/tmp/gdb.log
    gdb_history_file=~/tmp/gdb_history.txt
    openocd_cfg_file=~/embsys/tools/openocd/share/openocd/scripts/target/kinetis.cfg
    openocd_log_file=~/tmp/openocd.log

    echo "$FUNCNAME: Generating $init_script ..."
    echo "
target remote localhost:3333
#symbol-file $bin_file_prefix.elf
set print pretty on
set print array on
set print array-indexes on
set radix 16
set logging file $gdb_log_file
set logging overwrite on
set logging on
show logging
set history expansion on
set history file $gdb_history_file
set history save on
show history
load
continue
" > $init_script

    #sudo python ~/embsys/tools/pyOCD/test/gdb_server.py &

    ddd --gdb --trace --debugger "${CROSS_COMPILE}/bin/arm-none-eabi-gdb --command=$init_script $bin_file_prefix.elf"
    #${CROSS_COMPILE}/bin/arm-none-eabi-gdb --command=$init_script $bin_file_prefix.elf
}

function my_gdb_old
{
    typeset init_script
    typeset gdb_log_file
    typeset gdb_history_file
    typeset gdb_commands
    typeset openocd_cfg_file
    typeset openocd_log_file
    typeset bin_file_prefix

    bin_file_prefix=$DEFAULT_OBJ_FLAVOR_SUBDIR/Applications/$APPLICATION/$APPLICATION
    init_script=$bin_file_prefix.gdb
    gdb_log_file=~/tmp/gdb.log
    gdb_history_file=~/tmp/gdb_history.txt
    openocd_cfg_file=~/embsys/tools/openocd/share/openocd/scripts/target/kinetis.cfg
    openocd_log_file=~/tmp/openocd.log

    if [ ! -f $init_script ]; then
	echo "$FUNCNAME: Generating $init_script ..."
	echo "
target remote localhost:3333
#target remote | ~/linux_apps/open_ocd/bin/openocd -f $openocd_cfg_file -c 'gdb_port pipe; log_output $openocd_log_file'
show remotetimeout
set remotetimeout 180
#monitor arm7_9 dcc_downloads enable
#monitor arm7_9 fast_memory_access enable
monitor reset init
monitor wait_halt
monitor poll
symbol-file $bin_file_prefix.elf
set print pretty on
set print array on
set print array-indexes on
set radix 16
set logging file $gdb_log_file
set logging overwrite on
set logging on
show logging
set history expansion on
set history file $gdb_history_file
set history save on
show history
" > $init_script

    fi

    #~/embsys/tools/openocd/bin/openocd -f $openocd_cfg_file &
    ~/linux_apps/open_ocd/bin/openocd -f $openocd_cfg_file &

    ddd --gdb --trace --debugger "${CROSS_COMPILE}/bin/arm-none-eabi-gdb --command=$init_script"
    #${CROSS_COMPILE}/bin/arm-none-eabi-gdb --command=$init_script
}

cd $SRC_TREE_DIR
#gvim -S $IDE_VIM_SESSION
