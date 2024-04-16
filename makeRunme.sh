#!/bin/bash
make clean
make
cp bin/lynsyn_boot.bin ../lynsyn-host-software/fwbin/lite/lynsyn_boot_1.1.bin
cp bin/lynsyn_main.bin ../lynsyn-host-software/fwbin/lite/lynsyn_main_2.2.bin

cp bin/lynsyn_boot.bin ../lynsyn-host-software/lynsyn_tester/fwbin/lite/lynsyn_boot_1.1.bin
cp bin/lynsyn_main.bin ../lynsyn-host-software/lynsyn_tester/fwbin/lite/lynsyn_main_2.2.bin

cd ../lynsyn-host-software
#make (if you need to change anything in lynsyn sampler you dont need commander if it is jast main.cpp in lynsyn sampler)

#cd lynsyn_tester/ #you have to calibrate again if you use it 
#./lynsyn_tester  #you have to calibrate again if you use it 

cd lynsyn_tester/fwbin/lite #you dont have to calibrate again if you use it 

commander flash lynsyn_boot_1.1.bin --halt --device EFM32GG332F1024

commander flash lynsyn_main_2.2.bin --address 0x10000 --device EFM32GG332F1024

echo "Reboot the Lynsyn Lite device"

