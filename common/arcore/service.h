#ifndef ARCORE_SERVICE_H
#define ARCORE_SERVICE_H

#include <map>
#include <queue>
#include <jni.h>
#include <arcore/arcore.h>
#include <arcore/arengine.h>

namespace oc {

    class ARCoreService {
    public:
        enum Mode {
            GOOGLE_SFM,
            GOOGLE_TOF,
            GOOGLE_FACE,
            HUAWEI_SFM,
            HUAWEI_TOF,
            HUAWEI_FACE
        };

        ARCoreService(void *env, void *context, Mode mode = GOOGLE_SFM, bool flashlight = false);

        ~ARCoreService();

        void Clear(bool detach = true);

        void OnPause();

        void OnResume();

        void OnDisplayGeometryChanged(int display_rotation, int width, int height, bool fullhd = false);

        void Configure(void* session, void* frame);

        float CountFrameError();

        bool Process(bool update = true);

        std::vector<glm::vec3> GetActiveAnchors();

        std::vector<float> GetDistortion();
        
        Mesh GetFace();

        Image* GetDepthmap();

        Image* GetImage(ARCoreCamera::Effect effect = ARCoreCamera::NONE);

        Mode GetMode();

        bool HasCoordinateSystem();

        std::vector<glm::mat4> GetPose();

        static std::vector<glm::mat4> GetPose(glm::mat4 projection, glm::mat4 view);

        std::vector<glm::vec4> GetPointCloud(float maxDiff = INT_MAX);

        float GetPoseDiff() { return last_diff; }

        glm::mat4 GetProjection();

        glm::mat4 GetView();

        glm::vec3 HitTest(int x, int y);

        bool IsFaceMode();

        void RemoveFaceDetails();

        void RenderCamera(int effect = ARCoreCamera::GRAYSCALE, int scale = 1);

        void SetNVScheme(ARCoreCamera::NightVisionScheme s);

        void SetOffset(float offset);

        void SetResolution(float res);

    private:
        ARCore* google;
        AREngine* huawei;

        GLRenderer *renderer = nullptr;

        Mode mode_;

        float last_diff = -1;
        glm::vec3 image_position = glm::vec3(0);
        glm::quat image_rotation = glm::quat(0, 0, 0, 1);
    };
}

#endif
