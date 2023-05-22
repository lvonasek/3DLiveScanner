#include <data/depthmap.h>
#include <data/file3d.h>
#include <exporter/depthmaps.h>

namespace oc {

void ExporterDepthmaps::Process(Dataset* dataset)
{
    //load model
    double temp;
    int count, width, height;
    std::vector<Mesh> geom;
    dataset->ReadState(count, width, height, temp, temp, temp, temp);
    File3d(dataset->GetPath() + "model.obj", false).ReadModel(-1, geom);

    //process poses
    Depthmap depthmap(width, height);
    for (int i = 0; i < count; i++) {
        glm::mat4 world2screen = dataset->ReadPose(i)[SCREEN_CAMERA];
        depthmap.Render(geom, world2screen);
        depthmap.WritePNG16(dataset->GetFileName(i, ".png"));
    }
}
}
