#ifndef SCENE_H
#define SCENE_H

#include <set>
#include <vector>
#include "data/file3d.h"

struct RenderNode {
    std::vector<glm::vec3> aabb;
    float distance;
    std::vector<oc::Mesh*> meshes;
};

namespace oc {
    class GLScene {
    public:
        GLScene();
        ~GLScene();
        void Clear();
        void Load(std::vector<oc::Mesh>& input);
        void Process();
        void Render(GLint position_param, GLint uv_param);
        void UpdateVisibility(glm::mat4 mvp, glm::vec4 translate);
    private:
        int BasicTest(glm::mat4& mvp, glm::vec4& translate);
        float DistanceToAABB(glm::vec3 p);
        std::vector<RenderNode> GetRenderable();
        glm::vec4 GetTransform(int i);
        void PatchAABB();
        void ProcessRecursive();
        void SetVisibility(bool on, bool childs);
        void UpdateAABB(bool x, bool y, bool z);

        glm::vec3 aabb[2];
        glm::vec4 camera;
        GLScene* child[8];
        std::vector<Mesh> models;
        float size;
    };
}

#endif
