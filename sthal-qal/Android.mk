
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm

AUDIO_PLATFORM := $(TARGET_BOARD_PLATFORM)

LOCAL_CFLAGS += -Wall -Werror
LOCAL_CFLAGS += -DSOUND_TRIGGER_PLATFORM=$(TARGET_BOARD_PLATFORM)

LOCAL_HEADER_LIBRARIES := libhardware_headers \
                          libsystem_headers

LOCAL_SRC_FILES := \
    SoundTriggerDevice.cpp \
    SoundTriggerSession.cpp

LOCAL_SHARED_LIBRARIES := \
    libbase \
    liblog \
    libcutils \
    libdl \
    libaudioutils \
    libexpat \
    libhwbinder \
    libhidlbase \
    libhidltransport \
    libprocessgroup \
    libutils \
    libqal

LOCAL_C_INCLUDES += \
    external/tinyalsa/include \
    system/media/audio_utils/include \
    external/expat/lib \
    vendor/qcom/opensource/core-utils/fwk-detect \
    vendor/qcom/opensource/qal \

LOCAL_MODULE := sound_trigger.primary.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE_OWNER := qti
LOCAL_MODULE_TAGS := optional
LOCAL_VENDOR_MODULE := true
LOCAL_MULTILIB := $(AUDIOSERVER_MULTILIB)

include $(BUILD_SHARED_LIBRARY)

