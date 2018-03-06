#include $(srctree)/drivers/misc/mediatek/Makefile.custom
# Makefile for the fingerprint device.
ccflags-y += -I$(srctree)/drivers/spi/mediatek/mt6580
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6580/include
subdir-ccflags-y += -Werror
#subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
#subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6580/include
#subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
#subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/mt6580
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/mt6580/



obj-y := fingerprint.o madev.o

