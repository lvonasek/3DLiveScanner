#ifndef EXPORTER_PLY_H
#define EXPORTER_PLY_H

#include "exporter/exporter.h"

namespace oc {

class ExporterPLY : Exporter {
public:
    void Process(Dataset* dataset, std::string output);

    void Process(oc::Mesh& pcl, std::vector<glm::mat4>& pose, int poseIndex);
private:
    std::vector<oc::Mesh> merged;
    Dataset* dataset;
};
}

#endif
