export KBUILD_BUILD_USER="Mohancm"
export KBUILD_BUILD_HOST="TEAM_ZERO"
export CROSS_COMPILE="/home/mohanmanjappa/aarch64-linux-android-4.9/bin/aarch64-linux-android-"

make clean && make mrproper

mkdir out

make ARCH=arm64 O=out k5fpr_defconfig;make ARCH=arm64 -j30 O=out ;
