#ifndef EXPORTER_EXPORTER_H
#define EXPORTER_EXPORTER_H

#include <data/dataset.h>
#include <data/mesh.h>

namespace oc {

class Exporter {
public:
    int GetPoseCount(Dataset *dataset);

    virtual void Process(oc::Mesh &pcl, std::vector<glm::mat4> &pose, int poseIndex) = 0;

    void SetCallback(void(*callback)(int current, int count));

protected:
    void ConvertFrame(Dataset *dataset, int poseIndex, int poseCount);

    std::string GetId(int poseIndex);
};
}
#endif
