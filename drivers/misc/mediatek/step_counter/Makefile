# In case the platform does NOT support this type of sensors

ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
obj-y	+= step_counter.o

ifeq ($(CONFIG_CUSTOM_KERNEL_STEP_COUNTER),"OTHER_VENDOR")
obj-y += other_vendor/
endif
