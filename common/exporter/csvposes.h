#ifndef EXPORTER_CSVPOSES_H
#define EXPORTER_CSVPOSES_H

#include "exporter/exporter.h"

namespace oc {

class ExporterCSVPoses : Exporter {
public:
    void Process(Dataset* dataset, std::string output, bool rotated);

    void Process(Mesh& pcl, std::vector<glm::mat4>& pose, int poseIndex);
};
}

#endif
