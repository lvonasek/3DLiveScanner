#include <arcore/camera.h>
#include <data/image.h>
#include "camera.h"


namespace oc {

    bool operator<(const id3d& lhs, const id3d& rhs)
    {
        return lhs.x < rhs.x ||
               (lhs.x == rhs.x && (lhs.y < rhs.y || (lhs.y == rhs.y && lhs.z < rhs.z)));
    }

    float Diff(id3d a, id3d b) {
        int dx = abs(a.x - b.x);
        int dy = abs(a.y - b.y);
        int dz = abs(a.z - b.z);
        return dx * dx + dy * dy + dz * dz;
    }

    namespace {
        float kVertices[] = {
                -1.0f, -1.0f, 0.0f, +1.0f, -1.0f, 0.0f,
                -1.0f, +1.0f, 0.0f, +1.0f, +1.0f, 0.0f,
        };

        float kUvs[] = {
                0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        };

        std::string kVertexShader = "attribute vec4 vertex;\n"\
    "attribute vec2 textureCoords;\n"\
    "varying vec2 v_textureCoords;\n"\
    "void main() {\n"\
    "  v_textureCoords = textureCoords;\n"\
    "  gl_Position = vertex;\n"\
    "}";

        std::string kFragmentShader = \
    "uniform sampler2D depth;\n"\
    "uniform float u_effect;\n"\
    "uniform float u_sx;\n"\
    "uniform float u_sy;\n"\
    "uniform float u_texture_r;\n"\
    "uniform float u_texture_g;\n"\
    "uniform float u_texture_b;\n"\
    "varying vec2 v_textureCoords;\n"\
    "void main() {\n"\
    "  gl_FragColor = texture2D(texture, v_textureCoords);\n"\
    "  gl_FragColor.r *= u_texture_r;\n"\
    "  gl_FragColor.g *= u_texture_g;\n"\
    "  gl_FragColor.b *= u_texture_b;\n"\
    "  if (u_effect > 2.5) {\n"\
    "     gl_FragColor = texture2D(depth, v_textureCoords);\n"\
    "  } else if (u_effect > 1.5) {\n"\
    "    vec3 l = texture2D(texture, vec2(v_textureCoords.x - u_sx, v_textureCoords.y)).rgb;\n"\
    "    vec3 r = texture2D(texture, vec2(v_textureCoords.x + u_sx, v_textureCoords.y)).rgb;\n"\
    "    vec3 u = texture2D(texture, vec2(v_textureCoords.x, v_textureCoords.y - u_sy)).rgb;\n"\
    "    vec3 d = texture2D(texture, vec2(v_textureCoords.x, v_textureCoords.y + u_sy)).rgb;\n"\
    "    gl_FragColor.rgb = max(abs(l - r), abs(u - d));\n"\
    "    gl_FragColor.r = gl_FragColor.r > 0.05 ? gl_FragColor.r : 0.0;\n"\
    "    gl_FragColor.g = gl_FragColor.g > 0.05 ? gl_FragColor.g : 0.0;\n"\
    "    gl_FragColor.b = gl_FragColor.b > 0.05 ? gl_FragColor.b : 0.0;\n"\
    "    gl_FragColor.a = (gl_FragColor.r + gl_FragColor.g + gl_FragColor.b) / 3.0;\n"\
    "  } else if (u_effect > 0.5) {\n"\
    "    float c = (gl_FragColor.r + gl_FragColor.g + gl_FragColor.b) / 3.0;\n"\
    "    c = pow(c, 0.5);\n"\
    "    gl_FragColor.rgb = 0.5 * vec3(c, c, c);\n"\
    "  }\n"\
    "}";

    }  // namespace

    void ARCoreCamera::InitializeGlContent() {
        std::string ext = "#extension GL_OES_EGL_image_external : require\n";
        std::string uniform = "uniform samplerExternalOES texture;\n";
        shader_program_ = new GLSL(kVertexShader, uniform + kFragmentShader, ext);

        GLenum target = GL_TEXTURE_EXTERNAL_OES;
        glGenTextures(1, &texture_id_);
        glBindTexture(target, texture_id_);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        attribute_vertices_ = glGetAttribLocation(shader_program_->GetId(), "vertex");
        attribute_uvs_ = glGetAttribLocation(shader_program_->GetId(), "textureCoords");
    }


    unsigned char ARCoreCamera::Convert(int depth, int index) {
        switch (scheme) {
            case WHITE2RED:
                break;
            case WHITE2BLUE:
                index = 2 - index;
                break;
            case GREEN:
                return index == 1 ? glm::clamp(depth / 3, 0, 255) : 0;
            case BW:
                return glm::clamp(depth / 3, 0, 255);
        }
        switch (index) {
            case 0:
                return glm::clamp(depth, 0, 255);
            case 1:
                return glm::clamp(depth - 256, 0, 255);
            case 2:
                return glm::clamp(depth - 512, 0, 255);
        }
        return 0;
    }

    void ARCoreCamera::DrawARCore(const ArSession *session, const ArFrame *frame, Effect effect, int w, int h) {
        InitARCore(session, frame);
        Draw(effect, w, h);
    }

    void ARCoreCamera::DrawAREngine(const HwArSession *session, const HwArFrame *frame, Effect effect, int w, int h) {
        InitAREngine(session, frame);
        Draw(effect, w, h);
    }

    void ARCoreCamera::InitARCore(const ArSession *session, const ArFrame *frame) {
        int32_t geometry_changed = 0;
        ArFrame_getDisplayGeometryChanged(session, frame, &geometry_changed);
        if (geometry_changed != 0 || !uvs_initialized_) {
            ArFrame_transformDisplayUvCoords(session, frame, 8, kUvs, transformed_uvs_);
            minX = 9999;
            minY = 9999;
            maxX =-9999;
            maxY =-9999;
            aabb_initialized_ = false;
            uvs_initialized_ = true;
        }
    }

    void ARCoreCamera::InitAREngine(const HwArSession *session, const HwArFrame *frame) {
        int32_t geometry_changed = 0;
        HwArFrame_getDisplayGeometryChanged(session, frame, &geometry_changed);
        if (geometry_changed != 0 || !uvs_initialized_) {
            HwArFrame_transformDisplayUvCoords(session, frame, 8, kUvs, transformed_uvs_);
            minX = 9999;
            minY = 9999;
            maxX =-9999;
            maxY =-9999;
            aabb_initialized_ = false;
            uvs_initialized_ = true;
        }
    }

    void ARCoreCamera::Draw(Effect effect, int w, int h) {

        float correction[4] = {1, 1, 1, 1};

        glDepthMask(GL_FALSE);
        shader_program_->Bind();
        shader_program_->UniformFloat("u_sx", 1 / (float)w);
        shader_program_->UniformFloat("u_sy", 1 / (float)h);
        shader_program_->UniformFloat("u_texture_r", correction[0]);
        shader_program_->UniformFloat("u_texture_g", correction[1]);
        shader_program_->UniformFloat("u_texture_b", correction[2]);
        shader_program_->UniformFloat("u_effect", (int)effect);

        if (effect != DEPTH) {
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id_);
            shader_program_->UniformInt("texture", 1);
        }

        glEnableVertexAttribArray(attribute_vertices_);
        glVertexAttribPointer(attribute_vertices_, 3, GL_FLOAT, GL_FALSE, 0, kVertices);
        glEnableVertexAttribArray(attribute_uvs_);
        glVertexAttribPointer(attribute_uvs_, 2, GL_FLOAT, GL_FALSE, 0, transformed_uvs_);

        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDepthMask(GL_TRUE);
        shader_program_->Unbind();
    }

    glm::vec2 ARCoreCamera::Transform(int x, int y, float w, float h) {
        if (!aabb_initialized_) {
            for (int i = 0; i < 8; i += 2) {
                minX = glm::min(minX, transformed_uvs_[i] * w);
                maxX = glm::max(maxX, transformed_uvs_[i] * w);
            }
            for (int i = 1; i < 8; i += 2) {
                minY = glm::min(minY, transformed_uvs_[i] * h);
                maxY = glm::max(maxY, transformed_uvs_[i] * h);
            }
            dx = Axis(glm::vec2(transformed_uvs_[2] - transformed_uvs_[0], transformed_uvs_[3] - transformed_uvs_[1]));
            dy = Axis(glm::vec2(transformed_uvs_[6] - transformed_uvs_[2], transformed_uvs_[7] - transformed_uvs_[3]));
            aabb_initialized_ = true;
        }

        float ox = 2.0f * (x - minX) / (maxX - minX) - 1.0f;
        float oy = 2.0f * (y - minY) / (maxY - minY) - 1.0f;
        return glm::vec2(dx.x * ox + dx.y * oy, dy.x * ox + dy.y * oy);
    }

    glm::ivec2 ARCoreCamera::Axis(glm::vec2 v) {
        if (fabs(v.x) > fabs(v.y)) {
            return glm::ivec2(glm::sign(v.x), 0);
        } else {
            return glm::ivec2(0, glm::sign(v.y));
        }
    }
}
