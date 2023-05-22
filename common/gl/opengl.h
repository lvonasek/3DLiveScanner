#ifndef GL_OPENGL_H
#define GL_OPENGL_H

#ifdef ANDROID
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#else
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <algorithm>
#include <stdio.h>
#include <vector>
#endif

#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>

#ifdef ANDROID
#include <android/log.h>
#include <stdlib.h>
#include <string>
#include <vector>
#define LOGI(...) \
  __android_log_print(ANDROID_LOG_INFO, "arcore_app", __VA_ARGS__)
#define LOGE(...) \
  __android_log_print(ANDROID_LOG_ERROR, "arcore_app", __VA_ARGS__)
#else
#define LOGI(...); \
  printf(__VA_ARGS__); printf("\n")
#define LOGE(...); \
  printf(__VA_ARGS__); printf("\n")
#endif

#endif
