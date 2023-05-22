#include <exporter/exporter.h>
#include <sstream>

namespace oc {

void(*g_exporter_callback)(int current, int count) = 0;

int Exporter::GetPoseCount(Dataset* dataset)
{
    int poseCount = 0;
    FILE* file = fopen((dataset->GetPath() + "/state.txt").c_str(), "r");
    if (file)
    {
        fscanf(file, "%d", &poseCount);
        fclose(file);
    }
    return poseCount;
}

void Exporter::ConvertFrame(Dataset* dataset, int poseIndex, int poseCount)
{
    if (g_exporter_callback) g_exporter_callback(poseIndex, poseCount);

    //get orientation
    float yaw = glm::radians(dataset->ReadYaw());
    float s = glm::sin(-yaw);
    float c = glm::cos(-yaw);

    //get point cloud
    Tango3DR_PointCloud p = dataset->ReadPointCloud(poseIndex);

    //get matrices and captured photo
    std::vector<glm::mat4> pose = dataset->ReadPose(poseIndex);
    Image* jpg = new oc::Image(dataset->GetFileName(poseIndex, ".jpg"));

    //convert point cloud into world space and colorize it
    Mesh pcl;
    for (unsigned int i = 0; i < p.num_points; i++)
    {
        if ((p.points[i][2] > 0) && (p.points[i][2] < 10)) {
            glm::vec4 v = pose[COLOR_CAMERA] * glm::vec4(p.points[i][0], p.points[i][1], p.points[i][2], 1.0f);
            glm::vec4 t = pose[SCREEN_CAMERA] * glm::vec4(v.x, v.y, v.z, 1.0f);
            t /= fabs(t.w);
            t = t * 0.5f + 0.5f;
            int x = (int)(t.x * jpg->GetWidth());
            int y = (int)(t.y * jpg->GetHeight());
            if ((x >= 0) && (x < jpg->GetWidth()) && (y >= 0) && (y < jpg->GetHeight())) {
                pcl.colors.push_back(jpg->GetColor(x, y));
                pcl.vertices.emplace_back(v.x * s - v.z * c, v.y, v.x * c + v.z * s);
            }
        }
    }
    delete[] p.points;
    delete jpg;

    Process(pcl, pose, poseIndex);

    if (poseIndex + 1 == poseCount) {
        if (g_exporter_callback) g_exporter_callback(-1, -1);
    }
}

void Exporter::SetCallback(void(*callback)(int current, int count)) {
    g_exporter_callback = callback;
}

std::string Exporter::GetId(int poseIndex) {
    std::ostringstream ss;
    ss << poseIndex;
    std::string number = ss.str();
    while(number.size() < 8)
        number = "0" + number;
    return number;
}
}
