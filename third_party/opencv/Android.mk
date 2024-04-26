LOCAL_PATH := $(call my-dir)

ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
include $(CLEAR_VARS)
LOCAL_MODULE := libcpufeatures
LOCAL_SRC_FILES := libs/arm64-v8a/libcpufeatures.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libilmimf
LOCAL_SRC_FILES := libs/arm64-v8a/libIlmImf.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libittnotify
LOCAL_SRC_FILES := libs/arm64-v8a/libittnotify.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libjpeg
LOCAL_SRC_FILES := libs/arm64-v8a/liblibjpeg-turbo.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtbb
LOCAL_SRC_FILES := libs/arm64-v8a/libtbb.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtiff
LOCAL_SRC_FILES := libs/arm64-v8a/liblibtiff.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libwebp
LOCAL_SRC_FILES := libs/arm64-v8a/liblibwebp.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtegra_hal
LOCAL_SRC_FILES := libs/arm64-v8a/libtegra_hal.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_core
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_STATIC_LIBRARIES := libcpufeatures libtegra_hal libittnotify libilmimf libjpeg libtbb libtiff libwebp
LOCAL_SRC_FILES := libs/arm64-v8a/libopencv_core.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_features2d
LOCAL_STATIC_LIBRARIES := opencv_core opencv_flann
LOCAL_SRC_FILES := libs/arm64-v8a/libopencv_features2d.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_flann
LOCAL_STATIC_LIBRARIES := opencv_core
LOCAL_SRC_FILES := libs/arm64-v8a/libopencv_flann.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libopenjp2
LOCAL_SRC_FILES := libs/arm64-v8a/liblibopenjp2.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_imgcodecs
LOCAL_STATIC_LIBRARIES := opencv_core opencv_imgproc libopenjp2
LOCAL_SRC_FILES := libs/arm64-v8a/libopencv_imgcodecs.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_imgproc
LOCAL_STATIC_LIBRARIES := opencv_core
LOCAL_SRC_FILES := libs/arm64-v8a/libopencv_imgproc.a
include $(PREBUILT_STATIC_LIBRARY)


endif
