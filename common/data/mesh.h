#ifndef DATA_MESH_H
#define DATA_MESH_H

#include "data/image.h"
#include "gl/opengl.h"

namespace oc {

    class Mesh {
    public:

        Mesh();
        void Destroy();
        void GenerateFaceNormals();
        void GenerateNormals();
        float GetFloorLevel(glm::vec3 pos);
        void MirrorZ();
        void Normals2Color();
        void Reindex();
        void SwapYZ();
        static glm::vec3 Key2Vector(std::string a);
        static std::string Vector2key(glm::vec3 a);
    private:
        bool IsInAABB(glm::vec3& p, glm::vec3& min, glm::vec3& max);
        void UpdateAABB(glm::vec3& p, glm::vec3& min, glm::vec3& max);

        glm::vec3 aabbMin;
        glm::vec3 aabbMax;
        unsigned long aabbUpdate;
    public:
        std::vector<glm::vec3> vertices;
        std::vector<glm::vec3> normals;
        std::vector<unsigned int> colors;
        std::vector<unsigned int> indices;
        std::vector<glm::vec2> uv;
        Image* image;
        bool imageOwner;
        bool visible;
    };
}
#endif
