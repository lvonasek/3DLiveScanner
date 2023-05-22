#ifndef EXPORTER_FLOORPLAN_H
#define EXPORTER_FLOORPLAN_H

#include "exporter/exporter.h"
#include <map>

namespace oc {

struct FloorPlanPoint
{
    int x;
    int z;
};

bool operator<(const FloorPlanPoint& lhs, const FloorPlanPoint& rhs);

class ExporterFloorplan : public Exporter {
public:
    void Process(Dataset* dataset, std::string output);

    void Process(oc::Mesh& pcl, std::vector<glm::mat4>& pose, int poseIndex);

private:
    void Export(std::string output);

    void FillHoles();

    FloorPlanPoint min, max;

    std::string path;

    std::map<FloorPlanPoint, glm::ivec4> colormap;
    std::map<FloorPlanPoint, float> heightmap;
};
}

#endif
