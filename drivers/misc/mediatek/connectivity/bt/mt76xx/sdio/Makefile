###############################################################################
# Bluetooth character device driver

###############################################################################
ccflags-y += -DCONFIG_MP_WAKEUP_SOURCE_SYSFS_STAT

MODULE_NAME := btmtksdio
obj-m += $(MODULE_NAME).o

#ccflags-y += -D CREATE_NODE_DYNAMIC=1

$(MODULE_NAME)-objs += btmtk_sdio.o \
                       btmtk_main.o
