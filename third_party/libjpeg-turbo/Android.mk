LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := libjpeg-turbo

#LOCAL_ARM_NEON := true
#LOCAL_CFLAGS += -D__ARM_HAVE_NEON
LOCAL_ASMFLAGS += -DELF

#LOCAL_SRC_FILES += \
	src/simd/jsimd_arm.c \
	src/simd/jsimd_arm_neon.S \
	
LOCAL_SRC_FILES += \
	src/jsimd_none.c \


ifneq ($(filter $(TARGET_ARCH_ABI), armeabi-v7a armeabi-v7a-hard),)
LOCAL_CFLAGS += \
	-DSIZEOF_SIZE_T=4 \

else ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
LOCAL_CFLAGS += \
	-DSIZEOF_SIZE_T=8 \

endif

# libjpeg_la_SOURCES from Makefile.am
LOCAL_SRC_FILES += \
	src/jcapimin.c \
	src/jcapistd.c \
	src/jccoefct.c \
	src/jccolor.c \
	src/jcdctmgr.c \
	src/jchuff.c \
	src/jcinit.c \
	src/jcmainct.c \
	src/jcmarker.c \
	src/jcmaster.c \
	src/jcomapi.c \
	src/jcparam.c \
	src/jcphuff.c \
	src/jcprepct.c \
	src/jcsample.c \
	src/jctrans.c \
	src/jdapimin.c \
	src/jdapistd.c \
	src/jdatadst.c \
	src/jdatasrc.c \
	src/jdcoefct.c \
	src/jdcolor.c \
	src/jddctmgr.c \
	src/jdhuff.c \
	src/jdinput.c \
	src/jdmainct.c \
	src/jdmarker.c \
	src/jdmaster.c \
	src/jdmerge.c \
	src/jdphuff.c \
	src/jdpostct.c \
	src/jdsample.c \
	src/jdtrans.c \
	src/jerror.c \
	src/jfdctflt.c \
	src/jfdctfst.c \
	src/jfdctint.c \
	src/jidctflt.c \
	src/jidctfst.c \
	src/jidctint.c \
	src/jidctred.c \
	src/jquant1.c \
	src/jquant2.c \
	src/jutils.c \
	src/jmemmgr.c \
	src/jmemnobs.c \
	src/jaricom.c \
	src/jcarith.c \
	src/jdarith.c \
	src/turbojpeg.c \
	src/transupp.c \
	src/jdatadst-tj.c \
	src/jdatasrc-tj.c

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/include \

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/src/simd \
	$(LOCAL_PATH)/src \

LOCAL_EXPORT_C_INCLUDES := \
	$(LOCAL_PATH)/src \

LOCAL_CFLAGS += \
	-DBUILD="20141110" \
	-DC_ARITH_CODING_SUPPORTED=1 \
	-DD_ARITH_CODING_SUPPORTED=1 \
	-DBITS_IN_JSAMPLE=8 \
	-DHAVE_DLFCN_H=1 \
	-DHAVE_INTTYPES_H=1 \
	-DHAVE_LOCALE_H=1 \
	-DHAVE_MEMCPY=1 \
	-DHAVE_MEMORY_H=1 \
	-DHAVE_MEMSET=1 \
	-DHAVE_STDDEF_H=1 \
	-DHAVE_STDINT_H=1 \
	-DHAVE_STDLIB_H=1 \
	-DHAVE_STRINGS_H=1 \
	-DHAVE_STRING_H=1 \
	-DHAVE_SYS_STAT_H=1 \
	-DHAVE_SYS_TYPES_H=1 \
	-DHAVE_UNISTD_H=1 \
	-DHAVE_UNSIGNED_CHAR=1 \
	-DHAVE_UNSIGNED_SHORT=1 \
	-DINLINE="inline __attribute__((always_inline))" \
	-DJPEG_LIB_VERSION=62 \
	-DLIBJPEG_TURBO_VERSION="1.3.90" \
	-DMEM_SRCDST_SUPPORTED=1 \
	-DNEED_SYS_TYPES_H=1 \
	-DSTDC_HEADERS=1 \
	-DWITH_SIMD=1 \

include $(BUILD_STATIC_LIBRARY)
