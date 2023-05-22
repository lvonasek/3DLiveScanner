#ifndef DATA_DATASET_H
#define DATA_DATASET_H

#include <glm/glm.hpp>
#include <tango_3d_reconstruction_api.h>
#include <string>
#include <vector>

namespace oc {

    struct GridIndex {
        Tango3DR_GridIndex indices;

        bool operator==(const GridIndex &other) const;

        std::string toString() const {
            char buffer[32];
            sprintf(buffer, "%d %d %d", indices[0], indices[1], indices[2]);
            return "@ " + std::string(buffer);
        }
    };

    struct GridIndexHasher {
        std::size_t operator()(const oc::GridIndex &index) const {
            std::size_t val = std::hash<int>()(index.indices[0]);
            val = hash_combine(val, std::hash<int>()(index.indices[1]));
            val = hash_combine(val, std::hash<int>()(index.indices[2]));
            return val;
        }

        static std::size_t hash_combine(std::size_t val1, std::size_t val2) {
            return (val1 << 1) ^ val2;
        }
    };

    enum Pose { COLOR_CAMERA, OPENGL_CAMERA, SCREEN_CAMERA, MAX_CAMERA };

    class Dataset {
    public:

        Dataset(std::string path);
        std::string GetFileName(int index, std::string extension);
        std::string GetPath() { return dataset; }

        std::vector<float> ReadDistortion();
        Tango3DR_PointCloud ReadPointCloud(int index);
        std::vector<glm::mat4> ReadPose(int index);
        std::vector<std::pair<GridIndex, Tango3DR_Mesh*> > ReadPreview(int index, bool empty);
        void ReadState(int& count, int& width, int& height, double& cx, double& cy, double& fx, double& fy);
        float ReadYaw();

        void WriteDistortion(std::vector<float> data);
        void WritePointCloud(int index, Tango3DR_PointCloud t3dr_depth);
        void WritePose(int index, std::vector<glm::mat4> pose);
        void WritePreview(int index, std::vector<std::pair<GridIndex, Tango3DR_Mesh*> > preview);
        void WriteState(int count, int width, int height, double cx, double cy, double fx, double fy);
        void WriteYaw(float yaw);

    private:
        std::string dataset;
    };
}
#endif
