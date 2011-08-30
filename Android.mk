LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(TARGET_UBOOT_CONFIG_NAME),)
$(error "TARGET_UBOOT_CONFIG_NAME not set")
endif

UBOOT_IMG_NAME := u-boot-no-padding.bin

uboot: UBOOT_ROOT := $(LOCAL_PATH)

UBOOT_OUT := $(ANDROID_BUILD_TOP)/$(PRODUCT_OUT)/obj/uboot
UBOOT_ENV := O="$(UBOOT_OUT)" \
	CROSS_COMPILE="$(ANDROID_BUILD_TOP)/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-"

ifneq ($(PRODUCT_UBOOT_LOGO_FILE),)
UBOOT_ENV += UBOOT_CUSTOM_LOGO=$(ANDROID_BUILD_TOP)/$(PRODUCT_UBOOT_LOGO_FILE)
endif

.PHONY: FORCE
$(PRODUCT_OUT)/$(UBOOT_IMG_NAME): FORCE
	@echo "start build u-boot with cfgfile: $(TARGET_UBOOT_CONFIG_NAME)"
	-mkdir -p "$(UBOOT_OUT)"
	+$(MAKE) -C "$(UBOOT_ROOT)" $(UBOOT_ENV) $(TARGET_UBOOT_CONFIG_NAME)
	+$(MAKE) -C "$(UBOOT_ROOT)" $(UBOOT_ENV)
	@echo "Install: $@"
	$(hide) dd if="$(UBOOT_OUT)/u-boot.bin" of="$@" bs=1024 skip=1
	@echo "Install: $(ANDROID_BUILD_TOP)/$(PRODUCT_OUT)/u-boot.img"
	$(hide) cp "$(UBOOT_OUT)/u-boot.img" "$(ANDROID_BUILD_TOP)/$(PRODUCT_OUT)/"

uboot: $(PRODUCT_OUT)/$(UBOOT_IMG_NAME)

ALL_PREBUILT += uboot

