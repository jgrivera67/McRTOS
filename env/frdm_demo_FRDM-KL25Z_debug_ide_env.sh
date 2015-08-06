export PLATFORM="FRDM-KL25Z"
export APPLICATION="frdm_demo"
export BUILD_FLAVOR=debug

. $HOME/embsys/projects/McRTOS/env/McRTOS_common_ide_env.sh

function my_update_flash
{
    typeset dest_dir

    echo "copying $BIN_FILE to $PLATFORM flash ..."
    #dest_dir=/media/$USER/$PLATFORM
    dest_dir=/media/$USER/MBED

    if [ ! -d $dest_dir ]; then
	echo "*** Error: $dest_dir is not a directory"
	return 1
    fi

    cp $BIN_FILE $dest_dir
    status=$?
    if [ $status != 0 ]; then
	echo "*** cp failed with error $status ***"
	return 1
    fi

    sync
    return 0
}

function my_update_flash2
{
    typeset dest_dir

    echo "copying $BIN_FILE to $PLATFORM flash ..."
    #dest_dir=/media/$USER/$PLATFORM
    dest_dir=/media/$USER/MBED1

    if [ ! -d $dest_dir ]; then
	echo "*** Error: $dest_dir is not a directory"
	return 1
    fi

    cp $BIN_FILE $dest_dir
    status=$?
    if [ $status != 0 ]; then
	echo "*** cp failed with error $status ***"
	return 1
    fi

    sync
    return 0
}

