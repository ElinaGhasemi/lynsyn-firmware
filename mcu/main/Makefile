PART=EFM32GG332F1024

AS = ${TOOLCHAIN}/arm-none-eabi-as
CC = ${TOOLCHAIN}/arm-none-eabi-gcc
OBJCOPY= ${TOOLCHAIN}/arm-none-eabi-objcopy
OBJDUMP= ${TOOLCHAIN}/arm-none-eabi-objdump

CFLAGS = \
	-g -O2 -Wall -std=gnu99 \
	-mcpu=cortex-m3 -mthumb \
	-Wno-format-truncation \
	-DVERSION${HW} \
	-D${PART} \
	-I${SDK}/platform/Device/SiliconLabs/EFM32GG/Include/ \
	-I${SDK}/hardware/kit/common/drivers/ \
	-I${SDK}/platform/CMSIS/Include/ \
	-I${SDK}/platform/emlib/inc/ \
	-I${SDK}/platform/middleware/usb_gecko/inc/ \
	-I.
LDFLAGS = -mcpu=cortex-m3 -mthumb -Wl,-Tmain.ld --specs=nosys.specs 
ASFLAGS =

.PHONY: default
default: lynsyn_main.bin

%.bin : %.elf
	${OBJCOPY} -O binary $< $@

%.dis : %.elf
	${OBJDUMP} -l -S -D $< > $@

lynsyn_main.elf: \
	adc.o config.o descriptors.o jtag.o arm.o main.o usb.o fpga.o jtag_lowlevel.o EnergyManagement.o \
	../common/swo.o ../common/lynsyn.o \
	${SDK}/platform/emlib/src/em_cmu.o \
	${SDK}/platform/emlib/src/em_core.o \
	${SDK}/platform/emlib/src/em_gpio.o \
	${SDK}/platform/emlib/src/em_system.o \
	${SDK}/platform/emlib/src/em_timer.o \
	${SDK}/platform/emlib/src/em_usart.o \
	${SDK}/platform/emlib/src/em_adc.o \
	${SDK}/platform/emlib/src/em_dma.o \
	${SDK}/platform/emlib/src/em_prs.o \
	${SDK}/platform/emlib/src/em_rtc.o \
	${SDK}/platform/emlib/src/em_emu.o \
	${SDK}/platform/emlib/src/em_msc.o \
	${SDK}/platform/emlib/src/em_i2c.o \
	${SDK}/platform/middleware/usb_gecko/src/em_usbd.o \
	${SDK}/platform/middleware/usb_gecko/src/em_usbdch9.o \
	${SDK}/platform/middleware/usb_gecko/src/em_usbdep.o \
	${SDK}/platform/middleware/usb_gecko/src/em_usbdint.o \
	${SDK}/platform/middleware/usb_gecko/src/em_usbhal.o \
	${SDK}/platform/middleware/usb_gecko/src/em_usbtimer.o \
	${SDK}/platform/Device/SiliconLabs/EFM32GG/Source/GCC/startup_efm32gg.o \
	${SDK}/platform/Device/SiliconLabs/EFM32GG/Source/system_efm32gg.o \
	${SDK}/hardware/kit/common/drivers/dmactrl.o
	${CC} ${LDFLAGS} $^ -o $@

%.o : %.c
	${CC} ${CFLAGS} -c $< -o $@

%.o : %.s
	${AS} ${ASFLAGS} -c $< -o $@

%.o : %.S
	${AS} ${ASFLAGS} -c $< -o $@

.PHONY: upload
upload: lynsyn_main.bin
	JLinkExe jlink.cmd

.PHONY: clean
clean:
	rm -rf *.o *.elf *.bin *.hex *.dis ${SDK}/platform/emlib/src/*.o ${SDK}/platform/middleware/usb_gecko/src/*.o ${SDK}/Device/SiliconLabs/EFM32GG/Source/GCC/*.o ${SDK}/Device/SiliconLabs/EFM32GG/Source/*.o ${SDK}/platform/Device/SiliconLabs/EFM32GG/Source/GCC/startup_efm32gg.o
