#ifndef TANGO_TEXTURIZE_H
#define TANGO_TEXTURIZE_H

#include <data/dataset.h>
#include <gl/opengl.h>

namespace oc {

    class TangoTexturize {
    public:
        TangoTexturize();
        void Add(Image* image, double timestamp, Tango3DR_CameraCalibration* camera, std::vector<glm::mat4> matrix, Dataset* dataset);
        void ApplyDistortion(Tango3DR_CameraCalibration& camera, std::vector<float>& frame_distortion);
        void ApplyFrames(Dataset* dataset);
        void ApplyFrames(Dataset* dataset, std::vector<int> frames);
        Tango3DR_CameraCalibration Camera() { return camera; }
        void Clear(Dataset* dataset);
        void CreateContext(Tango3DR_Mesh* mesh, bool verbose, bool ignoreConfig);
        Tango3DR_TexturingContext Context() { return context; }
        void DeleteLast(Dataset* dataset);
        Tango3DR_Pose Extract3DRPose(glm::mat4 matrix);
        std::string GetEvent() { return event; }
        int GetLatestIndex(Dataset* dataset);
        double GetTimestamp(Dataset* dataset, int index);
        int GetWidth() { return width; }
        int GetHeight() { return height; }
        bool Init(std::string filename, bool verbose, bool ignoreConfig);
        void Process(std::string filename, bool verbose = true, bool poisson = false);
        void SetCalibration(Tango3DR_ReconstructionContext context, Dataset* dataset, int scale = 1);
        void SetDistortion(bool on) { useDistortion = on; }
        void SetEvent(std::string value) { event = value; }
        void SetTextureParams(int detail, int res, int count) { meshSimplification = detail, textureResolution = res; textureCount = count; }
        void UpdatePoses(Dataset* dataset);

    private:
        void ScaleMesh(Tango3DR_Mesh* mesh, float s);

        int poses;
        std::string event;
        Tango3DR_CameraCalibration camera;
        Tango3DR_TexturingContext context;
        int width, height;
        double cx, cy, fx, fy;
        bool useDistortion;

        int meshSimplification;
        int textureCount;
        int textureResolution;
        float scale;
    };
}
#endif
