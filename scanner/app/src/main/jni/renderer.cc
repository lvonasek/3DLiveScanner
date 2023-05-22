#include "renderer.h"

static const char* kTextureShader[] = {R"glsl(
    #version 100
    precision highp float;
    uniform mat4 u_MVP;
    uniform float u_X;
    uniform float u_Y;
    uniform float u_Z;
    attribute vec4 a_Position;
    attribute vec2 a_UV;
    varying vec2 v_UV;
    varying float v_Z;

    void main() {
      v_UV = a_UV;
      v_UV.t = 1.0 - a_UV.t;
      vec4 pos = a_Position;
      pos.x += u_X;
      pos.y += u_Y;
      pos.z += u_Z;
      gl_Position = u_MVP * pos;
      v_Z = gl_Position.z * 0.0015;
    })glsl",

    R"glsl(
    #version 100
    precision highp float;
    uniform sampler2D u_color_texture;
    varying vec2 v_UV;
    varying float v_Z;

    void main() {
      gl_FragColor = texture2D(u_color_texture, v_UV);
      if (gl_FragColor.a < 0.5)
        discard;
      gl_FragColor.r = clamp(gl_FragColor.r - v_Z, 0.0, 1.0);
      gl_FragColor.g = clamp(gl_FragColor.g - v_Z, 0.0, 1.0);
      gl_FragColor.b = clamp(gl_FragColor.b - v_Z, 0.0, 1.0);
      gl_FragColor.a = 1.0;
    })glsl"
};

void Renderer::Load(std::string filename) {
  scene.Clear();
  if (!filename.empty())
  {
    std::vector<oc::Mesh> meshes;
    oc::File3d io(filename, false);
    io.ReadModel(50000, meshes);
    scene.Load(meshes);
    scene.Process();
  }
  cur_position = glm::vec4();
  dst_position = glm::vec4();
}

void Renderer::InitializeGl() {
  GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &kTextureShader[0], nullptr);
  glCompileShader(vertex_shader);

  GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader, 1, &kTextureShader[1], nullptr);
  glCompileShader(fragment_shader);

  model_program_ = glCreateProgram();
  glAttachShader(model_program_, vertex_shader);
  glAttachShader(model_program_, fragment_shader);
  glLinkProgram(model_program_);
  glUseProgram(model_program_);

  model_position_param_ = glGetAttribLocation(model_program_, "a_Position");
  model_uv_param_ = glGetAttribLocation(model_program_, "a_UV");
  model_modelview_projection_param_ = glGetUniformLocation(model_program_, "u_MVP");
  model_texture_param_ = glGetUniformLocation(model_program_, "u_color_texture");
  model_translatex_param_ = glGetUniformLocation(model_program_, "u_X");
  model_translatey_param_ = glGetUniformLocation(model_program_, "u_Y");
  model_translatez_param_ = glGetUniformLocation(model_program_, "u_Z");
}

void Renderer::OnTriggerEvent(float x, float y, float z, float* view) {
  glm::vec4 dir = glm::vec4(x, y, z, 1) * glm::make_mat4(view);
  dir /= fabs(dir.w);
  dst_position += dir;
}

void Renderer::DrawModel(float* view) {
  glUseProgram(model_program_);
  glUniform1i(model_texture_param_, 2);
  glUniform1f(model_translatex_param_, cur_position.x);
  glUniform1f(model_translatey_param_, cur_position.y);
  glUniform1f(model_translatez_param_, cur_position.z);
  glUniformMatrix4fv(model_modelview_projection_param_, 1, GL_FALSE, view);

  glActiveTexture(GL_TEXTURE2);
  scene.UpdateVisibility(glm::make_mat4(view), cur_position);
  scene.Render(model_position_param_, model_uv_param_);
  glActiveTexture(GL_TEXTURE0);
  glUseProgram(0);
}

void Renderer::Update()
{
  cur_position = 0.95f * cur_position + 0.05f * dst_position;
}

#define JNI_METHOD(return_type, method_name) \
  JNIEXPORT return_type JNICALL              \
      Java_com_lvonasek_arcore3dscanner_vr_VRActivity_##method_name

extern "C" {

Renderer renderer;

JNI_METHOD(void, nativeLoadModel)
(JNIEnv *env, jobject, jstring filename) {
  const char *s = env->GetStringUTFChars(filename,NULL);
  std::string str( s );
  env->ReleaseStringUTFChars(filename,s);
  renderer.Load(str);
}

JNI_METHOD(void, nativeInitializeGl)
(JNIEnv *, jobject) {
  renderer.InitializeGl();
}

JNI_METHOD(void, nativeOnTriggerEvent)
(JNIEnv *env, jobject, jfloat x, jfloat y, jfloat z, jfloatArray matrix_) {
  jfloat *matrix = env->GetFloatArrayElements(matrix_, NULL);
  renderer.OnTriggerEvent(x, y, z, matrix);
  env->ReleaseFloatArrayElements(matrix_, matrix, 0);
}

JNI_METHOD(void, nativeDrawFrame)
(JNIEnv *env, jobject, jfloatArray matrix_) {
  jfloat *matrix = env->GetFloatArrayElements(matrix_, NULL);
  renderer.DrawModel(matrix);
  env->ReleaseFloatArrayElements(matrix_, matrix, 0);
}

JNI_METHOD(void, nativeUpdate)
(JNIEnv *, jobject) {
  renderer.Update();
}

}  // extern "C"
