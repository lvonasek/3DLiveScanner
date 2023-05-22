LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../../..
PROJECT_ROOT:= $(call my-dir)/../../../../..

include $(CLEAR_VARS)
LOCAL_MODULE           := lib3dscanner
LOCAL_CFLAGS           := -DCERES_FOUND=1
LOCAL_SHARED_LIBRARIES := arcore arengine opencv_imgcodecs opencv_features2d opencv_core tango_3d_reconstruction
LOCAL_STATIC_LIBRARIES := jpeg-turbo png poisson

LOCAL_C_INCLUDES := \
                    $(PROJECT_ROOT)/third_party/delaunay/ \
                    $(PROJECT_ROOT)/third_party/glm/ \
                    $(PROJECT_ROOT)/third_party/libjpeg-turbo/include/ \
                    $(PROJECT_ROOT)/third_party/libpng/include/ \
                    $(PROJECT_ROOT)/third_party/opencv/include/ \
                    $(PROJECT_ROOT)/common/

LOCAL_SRC_FILES := ../../../../../common/arcore/arcore.cc \
                   ../../../../../common/arcore/arengine.cc \
                   ../../../../../common/arcore/camera.cc \
                   ../../../../../common/arcore/service.cc \
                   ../../../../../common/data/dataset.cc \
                   ../../../../../common/data/depthmap.cc \
                   ../../../../../common/data/file3d.cc \
                   ../../../../../common/data/image.cc \
                   ../../../../../common/data/mesh.cc \
                   ../../../../../common/editor/effector.cc \
                   ../../../../../common/editor/rasterizer.cc \
                   ../../../../../common/editor/selector.cc \
                   ../../../../../common/exporter/csvposes.cc \
                   ../../../../../common/exporter/exporter.cc \
                   ../../../../../common/exporter/floorpln.cc \
                   ../../../../../common/gl/camera.cc \
                   ../../../../../common/gl/glsl.cc \
                   ../../../../../common/gl/renderer.cc \
                   ../../../../../common/gl/scene.cc \
                   ../../../../../common/postproc/optimizer.cc \
                   ../../../../../common/postproc/poisson.cc \
                   ../../../../../common/postproc/texturize.cc \
                   ../../../../../common/tango/retango.cc \
                   ../../../../../common/tango/scan.cc \
                   ../../../../../common/tango/texturize.cc \
                   ../../../../../common/thread/reconstr.cc \
                   ../../../../../common/thread/scene.cc \
                   app.cc \
                   renderer.cc

LOCAL_LDLIBS    := -llog -lGLESv2 -L$(SYSROOT)/usr/lib -lz -landroid -lmediandk
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT))
$(call import-add-path, $(PROJECT_ROOT)/third_party)
$(call import-module,arcore)
$(call import-module,arengine)
$(call import-module,libjpeg-turbo)
$(call import-module,libpng)
$(call import-module,opencv)
$(call import-module,poisson)
$(call import-module,tango_3d_reconstruction)
