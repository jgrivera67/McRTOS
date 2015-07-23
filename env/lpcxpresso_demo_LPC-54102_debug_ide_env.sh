export PLATFORM="LPC-54102"
export APPLICATION="lpcxpresso_demo"
export BUILD_FLAVOR=debug

. $HOME/embsys/projects/McRTOS/env/McRTOS_common_ide_env.sh


function my_update_flash
{
    typeset dest_dir

    echo "copying $ELF_FILE to $PLATFORM flash ..."

    /usr/local/lpcxpresso_7.8.0_426/lpcxpresso/bin/crt_emu_cm_redlink -flash-load-exec "$BIN_FILE" -g -2 -vendor=NXP -pLPC54102J512 #-flash-driver=LPC5410x_512K.cfx
    status=$?
    if [ $status != 0 ]; then
	echo "*** crt_emu_cm_redlink failed with error $status ***"
	return 1
    fi

    sync
    return 0
}


