subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
	include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif

#lenovo-sw, chenzz3, remove unused spi devices, begin
ifeq ($(CONFIG_SENSORS_FPC_1020),y)
obj-$(CONFIG_MTK_SPI) += spi.o spi-gpio.o
else
obj-$(CONFIG_MTK_SPI) += spi.o spi-dev.o
endif
#lenovo-sw, chenzz3, remove unused spi devices, end
