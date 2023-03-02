subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/prize/prize_exergen/
# In case the platform does NOT support this type of sensors
obj-m   +=  prize_measure_temp.o
prize_measure_temp-y  +=  prize_exergen.o
prize_measure_temp-y  +=  fm78100b.o
prize_measure_temp-y  +=  fm78100.o

