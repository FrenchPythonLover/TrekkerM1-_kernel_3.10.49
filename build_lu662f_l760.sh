#!/bin/bash

# product configure
DEFCONFIG=l760_defconfig

if [ "$1"z = "user"z ]; then
	echo -e "\n\n\t\t===== Compilation du noyau de production =====\n\n"
	DEFCONFIG=l760_release_defconfig
else
	echo -e "\n\n\t\t===== Compilation du noyau de déboggage =====\n\n"
fi
echo "Initialisation...."
OUTDIR=kout
TARGET_ARCH=arm
PATH=`pwd`/toolchain/arm-eabi-4.8/bin:$PATH
CROSS_COMPILER=arm-eabi-

NR_CPU=$(nproc)

mkdir -p $OUTDIR
echo "Make..."
sleep 1
make O=$OUTDIR ARCH=$TARGET_ARCH $DEFCONFIG

if [ "$1"z = "check"z ]; then
	echo "run static code analysis"
	shift
	CHECK_TARGET=$*
	. ./scripts/check_code.sh $DEFCONFIG $CHECK_TARGET
else
	find $OUTDIR/arch -name *.dtb -delete
	make O=$OUTDIR ARCH=$TARGET_ARCH CROSS_COMPILE=$CROSS_COMPILER -j${NR_CPU}

	. ./scripts/mkbootimg.sh $OUTDIR $TARGET_ARCH
fi
echo "Compilation terminée ! L'image boot.img se trouve dans kout/boot.img ."
