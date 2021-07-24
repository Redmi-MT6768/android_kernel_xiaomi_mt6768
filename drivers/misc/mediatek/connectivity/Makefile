#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# Connectivity combo driver
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
    subdir-ccflags-y += -I$(srctree)/
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1/$(MTK_PLATFORM)
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ifeq ($(CONFIG_MTK_PMIC_CHIP_MT6359),y)
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6359
endif
ifeq ($(CONFIG_MTK_PMIC_NEW_ARCH),y)
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include
endif
    subdir-ccflags-y += -I$(srctree)/drivers/mmc/core
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/$(MTK_PLATFORM)
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/
    subdir-ccflags-y += -I$(srctree)/drivers/clk/mediatek/
    subdir-ccflags-y += -I$(srctree)/drivers/pinctrl/mediatek/

    # Do Nothing, move to standalone repo
    MODULE_NAME := connadp
    obj-y += $(MODULE_NAME).o
    $(MODULE_NAME)-objs += common/connectivity_build_in_adapter.o
    $(MODULE_NAME)-objs += common/wmt_build_in_adapter.o


    # Do build-in for Makefile checking
    # export CONFIG_WLAN_DRV_BUILD_IN=y

    ifeq ($(CONFIG_WLAN_DRV_BUILD_IN),y)
        $(info $$CONFIG_MTK_COMBO_CHIP is [${CONFIG_MTK_COMBO_CHIP}])
        MTK_PLATFORM_ID := $(patsubst CONSYS_%,%,$(subst ",,$(CONFIG_MTK_COMBO_CHIP)))
        $(info MTK_PLATFORM_ID is [${MTK_PLATFORM_ID}])

        ifneq (,$(filter $(CONFIG_MTK_COMBO_CHIP), "CONSYS_6885" "CONSYS_6893"))
            export MTK_COMBO_CHIP=CONNAC2X2_SOC3_0
        else ifneq (,$(filter $(CONFIG_MTK_COMBO_CHIP), "CONSYS_6779" "CONSYS_6873" "CONSYS_6853"))
            export MTK_COMBO_CHIP=CONNAC2X2
        else
            export MTK_COMBO_CHIP=CONNAC
        endif

        # for gen4m options
        export CONFIG_MTK_COMBO_WIFI_HIF=axi
        export WLAN_CHIP_ID=$(MTK_PLATFORM_ID)
        export MTK_ANDROID_WMT=y
        export MTK_ANDROID_EMI=y
        export ADAPTOR_OPTS=$(MTK_COMBO_CHIP)

        WLAN_IP_SET_1_SERIES := 6765 6761 6885 6893
        WLAN_IP_SET_2_SERIES := 3967 6785
        WLAN_IP_SET_3_SERIES := 6779 6873 6853
        ifneq ($(filter $(WLAN_IP_SET_3_SERIES), $(WLAN_CHIP_ID)),)
            $(info WIFI_IP_SET is 3)
            export WIFI_IP_SET=3
        else ifneq ($(filter $(WLAN_IP_SET_2_SERIES), $(WLAN_CHIP_ID)),)
            $(info WIFI_IP_SET is 2)
            export WIFI_IP_SET=2
        else
            $(info WIFI_IP_SET is 1)
            export WIFI_IP_SET=1
        endif

        # Do build-in for xxx.c checking
        subdir-ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE
        subdir-ccflags-y += -D MTK_WCN_BUILT_IN_DRIVER
        obj-y += common/
        obj-y += wlan/adaptor/
        obj-y += wlan/core/gen4m/
        obj-y += gps/

        # For FM built-in mode
        # for fmradio options
        FM_CHIP_ID := $(patsubst CONSYS_%,%,$(subst ",,$(CONFIG_MTK_COMBO_CHIP)))
        $(info FM_CHIP_ID is [${FM_CHIP_ID}])

        ifneq (,$(filter $(CONFIG_MTK_COMBO_CHIP), "CONSYS_6885"))
            export CFG_BUILD_CONNAC2=true
        else
            export CFG_BUILD_CONNAC2=false
        endif

        FM_SOC_CHIPS  := 6580 6570 0633
        FM_6627_CHIPS := 6572 6582 6592 8127 6752 0321 0335 0337 6735 8163 6755 0326 6757 6763 6739 6625
        FM_6630_CHIPS := 6630 8167
        FM_6631_CHIPS := 6758 6759 6771 6775 6765 6761 3967 6797 6768 6785 8168
        FM_6632_CHIPS := 6632
        FM_6635_CHIPS := 6885 6873 6893
        FM_6631_6635_CHIPS := 6779 6853

        ifneq ($(filter $(FM_SOC_CHIPS), $(FM_CHIP_ID)),)
        FM_CHIP := soc
        else ifneq ($(filter $(FM_6627_CHIPS), $(FM_CHIP_ID)),)
        FM_CHIP := mt6627
        else ifneq ($(filter $(FM_6630_CHIPS), $(FM_CHIP_ID)),)
        FM_CHIP := mt6630
        else ifneq ($(filter $(FM_6631_CHIPS), $(FM_CHIP_ID)),)
        FM_CHIP := mt6631
        else ifneq ($(filter $(FM_6632_CHIPS), $(FM_CHIP_ID)),)
        FM_CHIP := mt6632
        else ifneq ($(filter $(FM_6635_CHIPS), $(FM_CHIP_ID)),)
            ifeq ($(strip $(MTK_CONSYS_ADIE)), MT6631)
                FM_CHIP := mt6631
            else
                FM_CHIP := mt6635
            endif
        else
            ifeq ($(strip $(MTK_COMBO_CHIP)), MT6632)
                FM_CHIP := mt6632
            else ifeq ($(strip $(MTK_COMBO_CHIP)), MT6627)
                FM_CHIP := mt6627
            else ifeq ($(strip $(MTK_COMBO_CHIP)), MT6630)
                FM_CHIP := mt6630
            endif
        endif

        export CFG_FM_CHIP_ID=$(FM_CHIP_ID)
        export CFG_FM_CHIP=$(FM_CHIP)
        $(info CFG_FM_CHIP_ID is [${CFG_FM_CHIP_ID}])
        $(info CFG_FM_CHIP is [${CFG_FM_CHIP}])
        $(info CFG_BUILD_CONNAC2 is [${CFG_BUILD_CONNAC2}])

        obj-y += fmradio/
    endif

# Otherwise we were called directly from the command line;
# invoke the kernel build system.
else
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    PWD  := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
