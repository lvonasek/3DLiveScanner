#ifndef RENDERER_H
#define RENDERER_H

#include <GLES2/gl2.h>
#include <jni.h>

#include <memory>
#include <string>
#include <vector>

#include "gl/scene.h"

class Renderer {
public:
  void DrawModel(float* view);
  void InitializeGl();
  void Load(std::string filename);
  void OnTriggerEvent(float x, float y, float z, float* view);
  void Update();

private:
  GLuint model_program_;
  GLint model_position_param_;
  GLint model_texture_param_;
  GLint model_uv_param_;
  GLint model_translatex_param_;
  GLint model_translatey_param_;
  GLint model_translatez_param_;
  GLint model_modelview_projection_param_;

  glm::vec4 cur_position;
  glm::vec4 dst_position;
  oc::GLScene scene;
};

#endif
