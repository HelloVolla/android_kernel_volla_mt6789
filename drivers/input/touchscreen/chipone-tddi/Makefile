ccflags-y += -I$(srctree)/fs
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include


ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/

obj-m += chipone-tddi.o

chipone-tddi-y += cts_tpd_driver.o cts_driver.o cts_i2c_driver.o cts_spi_driver.o
chipone-tddi-y += cts_core.o cts_sfctrlv2.o cts_spi_flash.o cts_firmware.o cts_test.o
chipone-tddi-y += cts_strerror.o
chipone-tddi-y += cts_charger_detect.o
chipone-tddi-y += cts_earjack_detect.o
chipone-tddi-y += cts_platform.o
chipone-tddi-y += cts_tool.o
chipone-tddi-y += cts_sysfs.o
chipone-tddi-y += cts_ioctl.o
chipone-tddi-y += cts_cdev.o
chipone-tddi-y +=  tpd_control.o
chipone-tddi-y +=  tpd_button.o
chipone-tddi-y +=  tpd_debug.o
chipone-tddi-y +=  tpd_default.o
chipone-tddi-y +=  tpd_misc.o
chipone-tddi-y +=  tpd_setting.o
