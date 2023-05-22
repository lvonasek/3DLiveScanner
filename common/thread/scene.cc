#include <gl/opengl.h>
#include <thread/scene.h>

namespace oc {

    Scene::Scene() : color_vertex_shader(0),
                     showNormals(false),
                     projectTexture(-1),
                     lastGrid(0),
                     textured_shader(0),
                     uniform(0) {
        vertex = TexturedVertexShader();
        fragment = TexturedFragmentShader();
        lastVertex = vertex;
        lastFragment = fragment;
    }

    Scene::~Scene() {
        if (color_vertex_shader)
            delete color_vertex_shader;
        if (mixed_shader)
            delete mixed_shader;
        if (textured_shader)
            delete textured_shader;
    }

    void Scene::SetupViewPort(int w, int h) {
        glViewport(0, 0, w, h);
        renderer = new GLRenderer();
        renderer->Init(w, h, w, h);
    }

    void Scene::BindDepthShader() {
        if (!depth_shader)
            depth_shader = new GLSL(DepthVertexShader(), DepthFragmentShader());
        depth_shader->Bind();
        depth_shader->UniformMatrix("u_projectMatrix", glm::value_ptr(projectMatrix));
    }

    void Scene::BindMixedShader() {
        if (!mixed_shader)
            mixed_shader = new GLSL(MixedVertexShader(), MixedFragmentShader());

        if (showNormals || projectTexture < 0) {
            color_vertex_shader->Bind();
        } else {
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, projectTexture);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, projectDepth);
            mixed_shader->Bind();
            mixed_shader->UniformMatrix("u_projectMatrix", glm::value_ptr(projectMatrix));
            mixed_shader->UniformInt("u_projectTexture", 1);
            mixed_shader->UniformInt("u_projectDepth", 0);
        }
    }

    void Scene::CustomRender(glm::mat4 matrix) {

        if (!textured_shader)
            textured_shader = new GLSL(vertex, fragment);

        long lastTexture = INT_MAX;
        for (Mesh& mesh : static_meshes_) {
            if (mesh.image && (mesh.image->GetTexture() == -1))
                mesh.image->SetTexture((long) GLSL::Image2GLTexture(mesh.image));
            if (!mesh.image || (mesh.image->GetTexture() == -1)) {
                if (color_vertex_shader) {
                    glm::vec3 p = renderer->camera.position;
                    color_vertex_shader->Bind();
                    color_vertex_shader->UniformFloat("u_uniformBegin", 65536);
                    color_vertex_shader->UniformFloat("u_uniformFactor", 1);
                    color_vertex_shader->UniformVec3("u_uniformCamera", p.x, p.y, p.z);
                    renderer->Render(&mesh.vertices[0].x, &mesh.vertices[0].x, 0, mesh.colors.data(), mesh.vertices.size(), 0, GL_POINTS);
                }
            } else {
                glActiveTexture(GL_TEXTURE2);
                if (lastTexture != mesh.image->GetTexture()) {
                    lastTexture = mesh.image->GetTexture();
                    glBindTexture(GL_TEXTURE_2D, (unsigned int)mesh.image->GetTexture());
                }
                if (textured_shader) {
                    textured_shader->Bind();
                    textured_shader->UniformInt("u_texture", 2);
                    textured_shader->UniformFloat("u_uniformNormals", 0);

                    int size = mesh.vertices.size();
                    GLSL::CurrentShader()->UniformMatrix("MVP", glm::value_ptr(matrix));
                    GLSL::CurrentShader()->Attrib(&mesh.vertices[0].x, &mesh.normals[0].x, &mesh.uv[0].s, mesh.colors.data());
                    if (size > 0) glDrawArrays(GL_TRIANGLES, 0, (GLsizei) size);
                }
                glActiveTexture(GL_TEXTURE0);
            }
        }
    }

    void Scene::Render(bool frustum) {

        if ((lastVertex.compare(vertex) != 0) || (lastFragment.compare(fragment) != 0))
        {
            delete textured_shader;
            textured_shader = 0;
            lastVertex = vertex;
            lastFragment = fragment;
        }

        if (!color_vertex_shader)
            color_vertex_shader = new GLSL(ColorVertexShader(), ColorFragmentShader());
        if (!textured_shader)
            textured_shader = new GLSL(vertex, fragment);

        long lastTexture = INT_MAX;
        for (Mesh& mesh : static_meshes_) {
            if (mesh.image && (mesh.image->GetTexture() == -1))
                mesh.image->SetTexture((long) GLSL::Image2GLTexture(mesh.image));
            if (!mesh.image || (mesh.image->GetTexture() == -1)) {
                if (color_vertex_shader) {
                    color_vertex_shader->Bind();
                    renderer->Render(&mesh.vertices[0].x, &mesh.vertices[0].x, 0, mesh.colors.data(), mesh.vertices.size(), 0, GL_POINTS);
                }
            } else {
                glActiveTexture(GL_TEXTURE2);
                if (lastTexture != mesh.image->GetTexture()) {
                    lastTexture = mesh.image->GetTexture();
                    glBindTexture(GL_TEXTURE_2D, (unsigned int)mesh.image->GetTexture());
                }
                if (textured_shader) {
                    textured_shader->Bind();
                    textured_shader->UniformInt("u_texture", 2);
                    textured_shader->UniformFloat("u_uniform", uniform);
                    textured_shader->UniformFloat("u_uniformNormals", showNormals ? 1 : 0);
                    textured_shader->UniformFloat("u_uniformPitch", uniformPitch);
                    textured_shader->UniformVec3("u_uniformPos", uniformPos.x, uniformPos.y, uniformPos.z);
                    renderer->Render(&mesh.vertices[0].x, &mesh.normals[0].x, &mesh.uv[0].s, mesh.colors.data(), mesh.vertices.size());
                }
                glActiveTexture(GL_TEXTURE0);
            }
        }
        if (color_vertex_shader) {
            color_vertex_shader->Bind();
            color_vertex_shader->UniformFloat("u_uniformNormals", 0);
            glDepthFunc(GL_LEQUAL);
            glLineWidth(2);

            if(!anchors_.vertices.empty() && frustum)
                renderer->Render(&anchors_.vertices[0].x, &anchors_.vertices[0].x, 0, anchors_.colors.data(),
                                 anchors_.indices.size(), anchors_.indices.data());
            if(!frustum_.vertices.empty() && frustum) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
                renderer->Render(&frustum_.vertices[0].x, &frustum_.vertices[0].x, 0, frustum_.colors.data(),
                                 frustum_.indices.size(), frustum_.indices.data(), GL_LINES);
                glDisable(GL_BLEND);
            }
            if (!selection_.vertices.empty() && !frustum) {
                glDisable(GL_DEPTH_TEST);
                renderer->Render(&selection_.vertices[0].x, &selection_.vertices[0].x, 0, selection_.colors.data(),
                                 selection_.indices.size(), selection_.indices.data(), GL_LINES);
                glEnable(GL_DEPTH_TEST);
            }
            glDepthFunc(GL_LESS);
            color_vertex_shader->UniformFloat("u_uniformNormals", showNormals ? 1 : 0);
        }

        for (unsigned int & i : Image::TexturesToDelete())
            glDeleteTextures(1, (const GLuint *) &i);
    }

    void Scene::RenderGrid(glm::vec3 p, int size, unsigned int color) {
        if (grid_.vertices.empty() || (glm::length(glm::vec3(lastGrid - p)) > 0.1f)) {
            lastGrid = p;
            grid_.colors.clear();
            grid_.vertices.clear();
            for (int x = -size; x <= size; x++) {
                for (int z = -size; z <= size; z++) {
                    int alpha = 255 * glm::max(0, size - abs(x) - abs(z)) / size;
                    unsigned int c = 0x01000000 * alpha + color;
                    if (x > -size) {
                        grid_.colors.push_back(c);
                        grid_.colors.push_back(c);
                        grid_.vertices.push_back(glm::vec3(x, 0, z + 1) + p);
                        grid_.vertices.push_back(glm::vec3(x, 0, z) + p);
                    }
                    if (z > -size) {
                        grid_.colors.push_back(c);
                        grid_.colors.push_back(c);
                        grid_.vertices.push_back(glm::vec3(x, 0, z) + p);
                        grid_.vertices.push_back(glm::vec3(x + 1, 0, z) + p);
                    }
                }
            }
        }

        color_vertex_shader->Bind();
        color_vertex_shader->UniformFloat("u_uniformNormals", 0);
        color_vertex_shader->UniformFloat("u_uniformBegin", 65536);
        color_vertex_shader->UniformFloat("u_uniformFactor", 1);
        color_vertex_shader->UniformVec3("u_uniformCamera", p.x, p.y, p.z);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
        renderer->Render(&grid_.vertices[0][0], &grid_.vertices[0][0], 0, &grid_.colors[0], grid_.vertices.size(), 0, GL_LINES);
        color_vertex_shader->UniformFloat("u_uniformNormals", showNormals ? 1 : 0);
        glDisable(GL_BLEND);
    }

    void Scene::UpdateAnchors(std::vector<glm::vec3> pos, float zoom) {
        anchors_.colors.clear();
        anchors_.indices.clear();
        anchors_.vertices.clear();

        for (glm::vec3& v : pos) {
            unsigned int offset = anchors_.vertices.size();

            float f = zoom * 0.005f;
            anchors_.vertices.push_back(v + glm::vec3(f, 0, 0));
            anchors_.vertices.push_back(v + glm::vec3(-f, 0, 0));
            anchors_.vertices.push_back(v + glm::vec3(0, 0, f));
            anchors_.vertices.push_back(v + glm::vec3(0, 0, -f));
            anchors_.vertices.push_back(v + glm::vec3(0, f, 0));
            anchors_.vertices.push_back(v + glm::vec3(0, -f, 0));

            for (int i = 0; i < 6; i++) {
                anchors_.colors.push_back(0x80FFFF00);
            }
            anchors_.indices.push_back(offset + 0);
            anchors_.indices.push_back(offset + 1);
            anchors_.indices.push_back(offset + 2);
            anchors_.indices.push_back(offset + 0);
            anchors_.indices.push_back(offset + 1);
            anchors_.indices.push_back(offset + 3);
            anchors_.indices.push_back(offset + 0);
            anchors_.indices.push_back(offset + 1);
            anchors_.indices.push_back(offset + 4);
            anchors_.indices.push_back(offset + 0);
            anchors_.indices.push_back(offset + 1);
            anchors_.indices.push_back(offset + 5);
            anchors_.indices.push_back(offset + 2);
            anchors_.indices.push_back(offset + 3);
            anchors_.indices.push_back(offset + 4);
            anchors_.indices.push_back(offset + 2);
            anchors_.indices.push_back(offset + 3);
            anchors_.indices.push_back(offset + 5);
        }
    }

    void Scene::UpdateFrustum(glm::mat4 mvp) {
        if(frustum_.colors.empty()) {
            for (int i = 0; i < 4; i++) {
                frustum_.colors.push_back(0x80FFFF00);
            }
            frustum_.indices.push_back(0); frustum_.indices.push_back(1);
            frustum_.indices.push_back(1); frustum_.indices.push_back(3);
            frustum_.indices.push_back(2); frustum_.indices.push_back(3);
            frustum_.indices.push_back(2); frustum_.indices.push_back(0);
        }
        if(!frustum_.vertices.empty())
            frustum_.vertices.clear();

        glm::mat4 matrix = glm::inverse(mvp);
        glm::vec4 c = matrix * glm::vec4(0, 0, 0, 1);
        c /= fabs(c.w);
        c.w = 1.0;
        for (int y = -1; y <= 1; y += 2) {
            for (int x = -1; x <= 1; x += 2) {
                glm::vec4 dx = matrix * glm::vec4(x, 0, 0, 1);
                dx /= fabs(dx.w);
                dx.w = 1.0;
                dx = glm::normalize(dx - c);
                glm::vec4 dy = matrix * glm::vec4(0, y, 0, 1);
                dy /= fabs(dy.w);
                dy.w = 1.0;
                dy = glm::normalize(dy - c);
                frustum_.vertices.emplace_back(c + dx * 0.036f + dy * 0.08f);
            }
        }
    }

    void Scene::UpdateSelected(bool process) {
        selection_.vertices.clear();
        selection_.colors.clear();
        selection_.indices.clear();

        if (process) {
            for (Mesh& m : static_meshes_) {
                for (int i = 0; i < m.vertices.size(); i += 3) {
                    bool a = m.colors[i + 0] == 0;
                    bool b = m.colors[i + 1] == 0;
                    bool c = m.colors[i + 2] == 0;

                    int count = 0;
                    if (a) count++;
                    if (b) count++;
                    if (c) count++;

                    if (count > 1) {
                        int size = selection_.vertices.size();
                        selection_.vertices.push_back(m.vertices[i + 0]);
                        selection_.vertices.push_back(m.vertices[i + 1]);
                        selection_.vertices.push_back(m.vertices[i + 2]);
                        selection_.colors.push_back(0xFF00FF00);
                        selection_.colors.push_back(0xFF00FF00);
                        selection_.colors.push_back(0xFF00FF00);

                        if (a && b) {
                            selection_.indices.push_back(size + 0);
                            selection_.indices.push_back(size + 1);
                        }
                        if (b && c) {
                            selection_.indices.push_back(size + 1);
                            selection_.indices.push_back(size + 2);
                        }
                        if (c && a) {
                            selection_.indices.push_back(size + 2);
                            selection_.indices.push_back(size + 0);
                        }
                    }
                }
            }
            selection_.GenerateNormals();
        }
    }

    std::string Scene::ColorFragmentShader() {
        return "varying vec4 f_color;\n"
               "void main() {\n"
               "  gl_FragColor = f_color;\n"
               "}";
    }

    std::string Scene::ColorVertexShader() {
        return "attribute vec4 v_vertex;\n"
               "attribute vec3 v_normal;\n"
               "attribute vec4 v_color;\n"
               "uniform mat4 MVP;\n"
               "uniform float u_uniformNormals;\n"
               "uniform float u_uniformBegin;\n"
               "uniform float u_uniformFactor;\n"
               "uniform vec3 u_uniformCamera;\n"
               "varying vec4 f_color;\n"
               "void main() {\n"
               "  gl_Position = MVP * v_vertex;\n"
               "  vec3 diff = abs(v_vertex.xyz - u_uniformCamera);\n"
               "  float dst = max(max(diff.x, diff.y), diff.z);\n"
               "  f_color = v_color * (1.0 - u_uniformNormals);\n"
               "  f_color.a = u_uniformNormals > 0.5 ? 1.0 : f_color.a;\n"
               "  f_color.r += abs(0.5 * v_normal.x + 0.5) * u_uniformNormals;\n"
               "  f_color.g += abs(0.5 * v_normal.y + 0.5) * u_uniformNormals;\n"
               "  f_color.b += abs(0.5 * v_normal.z + 0.5) * u_uniformNormals;\n"
               "  f_color.a *= min(max(1.0 - (dst - u_uniformBegin) * u_uniformFactor, 0.0), 1.0);\n"
               "}";
    }

    std::string Scene::DepthFragmentShader() {
        return "varying float f_depth;\n"
               "void main() {\n"
               "  float r = clamp(f_depth, 0.0, 1.0);\n"
               "  float g = clamp(f_depth - 1.0, 0.0, 1.0);\n"
               "  float b = clamp(f_depth - 2.0, 0.0, 1.0);\n"
               "  gl_FragColor = vec4(r, g, b, 1.0);\n"
               "}";
    }

    std::string Scene::DepthVertexShader() {
        return "attribute vec4 v_vertex;\n"
               "uniform mat4 MVP;\n"
               "uniform mat4 u_projectMatrix;\n"
               "varying float f_depth;\n"
               "void main() {\n"
               "  gl_Position = u_projectMatrix * v_vertex;\n"
               "  f_depth = gl_Position.z * 0.5;\n"
               "}";
    }

    std::string Scene::MixedFragmentShader() {
        return "uniform sampler2D u_projectDepth;\n"
               "uniform sampler2D u_projectTexture;\n"
               "varying vec4 f_color;\n"
               "varying vec4 f_uv;\n"
               "void main() {\n"
               "  if ((abs(f_uv.x) < 1.0) && (abs(f_uv.y) < 1.0) && (f_uv.z > 0.0)) { \n"
               "    vec4 depth = texture2D(u_projectDepth, f_uv.xy * 0.5 + 0.5) * 2.0;\n"
               "    float z = depth.r + depth.g + depth.b;\n"
               "    if (abs(z - f_uv.z) < 0.25)\n"
               "      gl_FragColor = texture2D(u_projectTexture, f_uv.xy * 0.5 + 0.5);\n"
               "    else\n"
               "      gl_FragColor = f_color;\n"
               "  } else\n"
               "    gl_FragColor = f_color;\n"
               "  gl_FragColor.a = f_color.a;\n"
               "}";
    }

    std::string Scene::MixedVertexShader() {
        return "attribute vec4 v_vertex;\n"
               "attribute vec3 v_normal;\n"
               "attribute vec4 v_color;\n"
               "uniform mat4 MVP;\n"
               "uniform mat4 u_projectMatrix;\n"
               "uniform float u_uniformNormals;\n"
               "uniform float u_uniformBegin;\n"
               "uniform float u_uniformFactor;\n"
               "uniform vec3 u_uniformCamera;\n"
               "varying vec4 f_color;\n"
               "varying vec4 f_uv;\n"
               "void main() {\n"
               "  gl_Position = MVP * v_vertex;\n"
               "  vec3 diff = abs(v_vertex.xyz - u_uniformCamera);\n"
               "  float dst = max(max(diff.x, diff.y), diff.z);\n"
               "  f_color = v_color * (1.0 - u_uniformNormals);\n"
               "  f_color.a *= min(max(1.0 - (dst - u_uniformBegin) * u_uniformFactor, 0.0), 1.0);\n"
               "  f_uv = u_projectMatrix * v_vertex;\n"
               "  f_uv.xy /= abs(f_uv.w);\n"
               "}";
    }

    std::string Scene::TexturedFragmentShader() {
        return "uniform sampler2D u_texture;\n"
                "varying vec4 f_color;\n"
                "varying vec2 v_uv;\n"
                "uniform float u_uniformNormals;\n"
                "void main() {\n"
                "  vec4 color =  texture2D(u_texture, v_uv);\n"
                "  if (color.a < 0.5) discard;\n"
                "  gl_FragColor = (1.0 - u_uniformNormals) * color - f_color;\n"
                "}";
    }

    std::string Scene::TexturedVertexShader() {
        return "attribute vec4 v_vertex;\n"
                "attribute vec2 v_coord;\n"
                "attribute vec3 v_normal;\n"
                "attribute vec4 v_color;\n"
                "varying vec4 f_color;\n"
                "varying vec2 v_uv;\n"
                "uniform mat4 MVP;\n"
                "uniform float u_uniformNormals;\n"
                "void main() {\n"
                "  f_color = v_color;\n"
                "  f_color.r -= abs(0.5 * v_normal.x + 0.5) * u_uniformNormals;\n"
                "  f_color.g -= abs(0.5 * v_normal.y + 0.5) * u_uniformNormals;\n"
                "  f_color.b -= abs(0.5 * v_normal.z + 0.5) * u_uniformNormals;\n"
                "  v_uv.x = v_coord.x;\n"
                "  v_uv.y = 1.0 - v_coord.y;\n"
                "  gl_Position = MVP * v_vertex;\n"
                "}";
    }
}
