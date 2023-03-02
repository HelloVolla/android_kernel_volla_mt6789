# Makefile for the focaltech touchscreen drivers.


ccflags-y += -I$(srctree)/fs
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include


ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/



obj-m += ft8722.o
ft8722-y +=  focaltech_core.o
ft8722-y +=  focaltech_ex_fun.o
ft8722-y +=  focaltech_ex_mode.o
ft8722-y +=  focaltech_gesture.o
ft8722-y +=  focaltech_esdcheck.o
ft8722-y +=  focaltech_point_report_check.o
ft8722-y +=  focaltech_flash.o
ft8722-y +=  focaltech_spi.o
