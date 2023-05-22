#include <data/dataset.h>
#include <exporter/csvposes.h>
#include <exporter/depthmaps.h>
#include <exporter/floorpln.h>
#include <exporter/ply.h>
#include <postproc/texturize.h>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        LOGI("Usage: ./dataset_extractor <path to the dataset>");
        return 0;
    }
    oc::Dataset* dataset = new oc::Dataset(std::string(argv[1]));
    //oc::ExporterCSVPoses().Process(dataset, dataset->GetPath() + "/posesOBJ.csv", false);
    //oc::ExporterCSVPoses().Process(dataset, dataset->GetPath() + "/posesPLY.csv", true);
    //oc::ExporterDepthmaps().Process(dataset);
    //oc::ExporterFloorplan().Process(dataset, dataset->GetPath() + "/");
    //oc::ExporterPLY().Process(dataset, dataset->GetPath() + "/output.ply");
    oc::Texturize().Process(dataset, dataset->GetPath() + "model.obj", true);
    std::exit(0);
    return 0;
}
