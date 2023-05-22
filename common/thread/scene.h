#ifndef THREAD_SCENE_H
#define THREAD_SCENE_H

#include <vector>
#include <data/file3d.h>
#include <gl/glsl.h>
#include <gl/renderer.h>

namespace oc {
    class Scene {
    public:
        Scene();
        ~Scene();
        void BindDepthShader();
        void BindMixedShader();
        void CustomRender(glm::mat4 matrix);
        void Render(bool frustum);
        void RenderGrid(glm::vec3 p, int size, unsigned int color);
        void SetupViewPort(int w, int h);
        void UpdateAnchors(std::vector<glm::vec3> pos, float zoom);
        void UpdateFrustum(glm::mat4 mvp);
        void UpdateSelected(bool process);

        std::string ColorFragmentShader();
        std::string ColorVertexShader();
        std::string DepthFragmentShader();
        std::string DepthVertexShader();
        std::string MixedFragmentShader();
        std::string MixedVertexShader();
        std::string TexturedFragmentShader();
        std::string TexturedVertexShader();

        Mesh anchors_;
        Mesh frustum_;
        Mesh grid_;
        Mesh selection_;
        std::vector<Mesh> static_meshes_;
        GLSL* color_vertex_shader;
        GLSL* depth_shader;
        GLSL* mixed_shader;
        GLSL* textured_shader;
        GLRenderer* renderer;
        bool showNormals;
        int projectDepth;
        int projectTexture;
        glm::mat4 projectMatrix;

        std::string vertex;
        std::string fragment;
        float uniform;
        float uniformPitch;
        glm::vec3 uniformPos;
    private:
        glm::vec3 lastGrid;
        std::string lastVertex;
        std::string lastFragment;
    };
}

#endif
