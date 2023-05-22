#ifndef ARCORE_ARENGINE_H
#define ARCORE_ARENGINE_H

#include <map>
#include <jni.h>
#include <arcore/camera.h>
#include <data/image.h>
#include <data/mesh.h>
#include <gl/renderer.h>
#include <media/NdkImageReader.h>

namespace oc {

    class AREngine {
    public:
        AREngine(void *env, void *context, bool depthCamera = true, bool faceMode = false, bool flashlight = false);

        ~AREngine();

        void Clear(bool detach);

        void OnPause();

        void OnResume();

        void OnDisplayGeometryChanged(int display_rotation, int width, int height);

        void Configure(void* session, void* frame);

        float CountFrameError();

        bool Process(bool update = true);

        std::vector<glm::vec3> GetActiveAnchors();

        std::vector<float> GetDistortion();

        glm::vec3 HitTest(int x, int y);

        Mesh GetFace(glm::mat4 matrix) { UpdateFace(matrix); return face_mesh; };

        std::vector<glm::vec4> GetPointCloud() { UpdateFeaturePoints(); return points; }

        glm::mat4 GetProjection() { return projection_mat; }

        glm::mat4 GetView() { return view_mat; }

        bool HasCoordinateSystem() { return has_coordinate_system_; }

        void RenderCamera(ARCoreCamera::Effect effect = ARCoreCamera::GRAYSCALE, int scale = 1);

        void SetNVScheme(ARCoreCamera::NightVisionScheme s) { camera.SetNVScheme(s); }

        void SetOffset(float value) { offset = value; }

        void SetResolution(float res) { resolution = res; }

        Image* GetDepthMap(bool confidence, bool increasing, int s = 1);
    private:
        bool UpdateAnchor();

        void UpdateFace(glm::mat4 matrix);

        void UpdateFeaturePoints();

        std::map<id3d, HwArAnchor*> ar_anchor_list;
        HwArSession *ar_session_ = nullptr;
        HwArFrame *ar_frame_ = nullptr;
        Mesh face_mesh;

        ARCoreCamera camera;
        glm::mat4 view_mat;
        glm::mat4 projection_mat;

        bool face_mode_;
        bool has_coordinate_system_;
        std::vector<glm::vec4> points;
        bool texture_initialized_ = false;
        float offset;
        float resolution;
        bool useDepth;
        int viewportWidth;
        int viewportHeight;
    };
}

#endif
