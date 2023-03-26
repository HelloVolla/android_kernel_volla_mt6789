ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/imgsensor/inc/

obj-m += prize_camera.o
prize_camera-y	+= prize_dual_camera.o
prize_camera-y	+= gc032a.o
prize_camera-y	+= gc0310.o
prize_camera-y	+= ov5645.o

ccflags-y += $(addprefix -DPRIZE_MB_, $(shell echo $(CONFIG_PRIZE_BOARD) | tr a-z A-Z))