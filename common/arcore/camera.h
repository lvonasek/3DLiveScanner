#ifndef ARCORE_CAMERA_H
#define ARCORE_CAMERA_H

#include <cstdlib>
#include <arcore/platform.h>
#include <gl/glsl.h>

namespace oc {

    struct id3d
    {
        int x;
        int y;
        int z;
        int layer;
        glm::mat4 matrix;

        bool operator==(id3d v)
        {
            return (v.x == x) && (v.y == y) && (v.z == z) && (v.layer == layer);
        }

        bool operator!=(id3d v)
        {
            return !(*this==v);
        }
    };

    bool operator<(const id3d& lhs, const id3d& rhs);

    float Diff(id3d a, id3d b);

    class ARCoreCamera {
    public:
        enum Effect {NONE, GRAYSCALE, EDGES, DEPTH, DEPTH_INV, NIGHTVISION};

        enum NightVisionScheme {WHITE2RED, WHITE2BLUE, GREEN, BW};

        void InitializeGlContent();

        unsigned char Convert(int depth, int index);

        void DrawARCore(const ArSession *session, const ArFrame *frame, Effect effect, int w = 0, int h = 0);

        void DrawAREngine(const HwArSession *session, const HwArFrame *frame, Effect effect, int w = 0, int h = 0);

        GLSL* GetShader() { return shader_program_; }

        GLuint GetTextureName() { return texture_id_; }

        void InitARCore(const ArSession *session, const ArFrame *frame);

        void InitAREngine(const HwArSession *session, const HwArFrame *frame);

        void SetNVScheme(NightVisionScheme s) { scheme = s; }

        glm::vec2 Transform(int x, int y, float w, float h);

    private:
        glm::ivec2 Axis(glm::vec2 v);

        void Draw(Effect effect, int w, int h);

        GLSL *shader_program_;
        GLuint texture_id_;

        GLuint attribute_vertices_;
        GLuint attribute_uvs_;

        glm::ivec2 dx, dy;
        float minX, minY, maxX, maxY;
        float transformed_uvs_[8];
        bool aabb_initialized_ = false;
        bool uvs_initialized_ = false;
        NightVisionScheme scheme = WHITE2RED;
    };
}
#endif
