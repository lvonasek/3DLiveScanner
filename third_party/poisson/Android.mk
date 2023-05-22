LOCAL_PATH := $(call my-dir)
 
include $(CLEAR_VARS)
 
LOCAL_MODULE := libpoisson
LOCAL_SRC_FILES = CmdLineParser.cpp \
Factor.cpp \
Geometry.cpp \
MarchingCubes.cpp \
PlyFile.cpp \
PoissonRecon.cpp

LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_EXPORT_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_EXPORT_LDLIBS := -lz
 
include $(BUILD_STATIC_LIBRARY)
