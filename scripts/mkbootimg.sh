#!/bin/bash

# common setting
OUTDIR=$1
TARGET_ARCH=$2

# mkbootimg parameter,from device/qcom/<product>/BoardConfig.mk

KERNEL_CMDLINES="console=ttyHSL0,115200,n8 androidboot.console=ttyHSL0 androidboot.hardware=qcom user_debug=31 msm_rtb.filter=0x237 ehci-hcd.park=3 androidboot.bootdevice=7824900.sdhci lpm_levels.sleep_disabled=1 earlyprintk"
KERNEL=$OUTDIR/arch/$TARGET_ARCH/boot/zImage-dtb

BASEADDR="--base 0x80000000"
PAGESIZE="--pagesize 2048"

# get product name
BOARD_CFG=`cat $OUTDIR/.config | grep CONFIG_HISENSE_PRODUCT_NAME | sed "s/.*=\"\([A-Za-z0-9_\-]*\)\"/\1/"`

RAMDISK=./bootimg/ramdisk_${BOARD_CFG}.img
BOOTIMG=./bootimg/boot.img

# mkbootimg
if [ -f $KERNEL ]; then
	rm -f $BOOTIMG

	./bootimg/mkbootimg --kernel $KERNEL --ramdisk $RAMDISK --cmdline "$KERNEL_CMDLINES" $BASEADDR $PAGESIZE --output $BOOTIMG
	echo "Build boot.img complete."
else
	echo "Kernel image \"$KERNEL\" not found."
fi

