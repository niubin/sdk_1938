#!/bin/bash


current_dir=$(pwd)
echo ${current_dir}
cp arch/arm/boot/zImage ../scm801_mkimg/img/zImage
cp arch/arm/boot/dts/faraday-leo.dtb ../scm801_mkimg/img/faraday_leo.dtb


cd ../scm801_mkimg/fit

./mkimage -f kernel.its kernel.img

mv kernel.img /mnt/hgfs/vmware/kernel.img -f

cd ${current_dir}
