BUILD_INFAE := i2c
BUILD_PLATFORM := qcom
BUILD_MODULE := n

ccflags-y += -Wall

ifeq ($(BUILD_PLATFORM),mtk)
platform=ilitek_v3_mtk

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ILITEK_V3/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ILITEK_V3/firmware/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
endif

ifeq ($(BUILD_PLATFORM),qcom)
platform=ilitek_v3_qcom
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ILITEK_V3/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ILITEK_V3/firmware/
endif

ifeq ($(BUILD_INFAE),i2c)
interface=ilitek_v3_i2c
fwupdate=ilitek_v3_flash
endif

ifeq ($(BUILD_INFAE),spi)
interface=ilitek_v3_spi
fwupdate=ilitek_v3_hostdl
endif
#For display ESD recovery
ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/

obj-m += ilitek_v3.o
ilitek_v3-y += ilitek_v3_core.o
ilitek_v3-y += $(interface).o
ilitek_v3-y += $(platform).o
ilitek_v3-y += ilitek_v3_ic.o
ilitek_v3-y += ilitek_v3_touch.o
ilitek_v3-y += ilitek_v3_mp.o
ilitek_v3-y += $(fwupdate).o
ilitek_v3-y += ilitek_v3_node.o


