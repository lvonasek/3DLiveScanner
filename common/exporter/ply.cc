#include <data/file3d.h>
#include <exporter/ply.h>

namespace oc {

void ExporterPLY::Process(Dataset* dataset, std::string output)
{
    //init dataset
    this->dataset = dataset;
    int poseCount = GetPoseCount(dataset);

    //convert each frame
    for (int i = 0; i < poseCount; i++)
    {
        ConvertFrame(dataset, i, poseCount);
    }

    //merge all frames together
    File3d(output, true).WriteModel(merged);
}

void ExporterPLY::Process(oc::Mesh& pcl, std::vector<glm::mat4>& pose, int poseIndex)
{
    std::vector<oc::Mesh> output;
    output.push_back(pcl);
    merged.push_back(pcl);
    oc::File3d(dataset->GetFileName(poseIndex, ".ply"), true).WriteModel(output);
}
}
