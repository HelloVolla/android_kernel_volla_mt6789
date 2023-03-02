# Makefile for the focaltech touchscreen drivers.
ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/

obj-m += focaltech_v3.o

focaltech_v3-y	+=  focaltech_core.o
focaltech_v3-y	+=  focaltech_ex_fun.o
focaltech_v3-y	+=  focaltech_ex_mode.o
focaltech_v3-y	+=  focaltech_gesture.o
focaltech_v3-y	+=  focaltech_esdcheck.o
focaltech_v3-y   +=  focaltech_point_report_check.o

focaltech_v3-y   +=  focaltech_i2c.o
focaltech_v3-y	+=  focaltech_flash.o
focaltech_v3-y   +=  focaltech_upgrade_ft8756.o
