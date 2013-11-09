export KBUILD_BUILD_USER="Mohancm"
export KBUILD_BUILD_HOST="A.M.T"
export CROSS_COMPILE=/home/mohanmanjappa/aarch64-linux-android-4.9/bin/aarch64-linux-android-

export ARCH=arm64

make clean && make mrproper

make k5fpr_defconfig

make -j8
