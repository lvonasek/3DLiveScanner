#include <data/file3d.h>
#include <exporter/floorpln.h>

namespace oc {

bool operator<(const FloorPlanPoint& lhs, const FloorPlanPoint& rhs)
{
    return lhs.x < rhs.x || (lhs.x == rhs.x && (lhs.z < rhs.z));
}

void ExporterFloorplan::Process(Dataset* dataset, std::string output)
{
    //init AABB
    int bigNumber = 9999999;
    min.x = bigNumber;
    min.z = bigNumber;
    max.x =-bigNumber;
    max.z =-bigNumber;

    //init dataset
    path = dataset->GetPath();
    int poseCount = GetPoseCount(dataset);

    //convert each frame
    for (int i = 0; i < poseCount; i++)
    {
        ConvertFrame(dataset, i, poseCount);
    }
    min.x -= 10;
    min.z -= 10;
    max.x += 10;
    max.z += 10;

    FillHoles();
    Export(output);
}

void ExporterFloorplan::Process(oc::Mesh& pcl, std::vector<glm::mat4>& pose, int poseIndex)
{
    glm::vec3 v;
    glm::mat4 matrix = pose[COLOR_CAMERA];
    std::map<FloorPlanPoint, bool> cache;
    for (unsigned int i = 0; i < pcl.vertices.size(); i++)
    {
        v = pcl.vertices[i];
        if (v.y < matrix[3][1])
        {
            FloorPlanPoint p;
            p.x = v.x * 100;
            p.z = v.z * 100;

            if (cache.find(p) == cache.end()) {
                if (heightmap.find(p) == heightmap.end())
                {
                    colormap[p] = glm::ivec4(File3d::DecodeColor(pcl.colors[i]), 1);
                    heightmap[p] = v.y;

                    if (min.x > p.x) min.x = p.x;
                    if (min.z > p.z) min.z = p.z;
                    if (max.x < p.x) max.x = p.x;
                    if (max.z < p.z) max.z = p.z;
                }
                else
                {
                    if (fabs(heightmap[p] - v.y) < 0.15f)
                    {
                        glm::ivec4 tcolor = glm::ivec4(File3d::DecodeColor(pcl.colors[i]), 1);
                        glm::ivec4 color = colormap[p];
                        int n = glm::min(color.a + 1, 255);
                        color = ((n - 1) * color + tcolor) / n;
                        color.a = n;
                        colormap[p] = color;
                    }
                    else if (heightmap[p] < v.y)
                    {
                        colormap[p] = glm::ivec4(File3d::DecodeColor(pcl.colors[i]), 1);
                    }
                    heightmap[p] = glm::max(heightmap[p], v.y);
                }
                cache[p] = true;
            }
        }
    }
}

void ExporterFloorplan::Export(std::string output)
{
    int w = max.x - min.x + 1;
    int h = max.z - min.z + 1;
    LOGI("Exporting floorplan has dimensions %dcm x %dcm, coverage is %f%%", w, h, 100.0f * heightmap.size() / (float)(w * h));

    Image img(w, h);
    img.Clear();

    glm::ivec4 color;
    FloorPlanPoint p;
    for (p.x = min.x; p.x < max.x; p.x++)
    {
        for (p.z = min.z; p.z < max.z; p.z++)
        {
            if (heightmap.find(p) != heightmap.end())
            {
                int x = p.x - min.x;
                int y = p.z - min.z;
                color = glm::clamp(colormap[p], 0, 255);
                img.GetData()[(y * w + x) * 4 + 0] = color.r;
                img.GetData()[(y * w + x) * 4 + 1] = color.g;
                img.GetData()[(y * w + x) * 4 + 2] = color.b;
                img.GetData()[(y * w + x) * 4 + 3] = 255;
            }
        }
    }
    img.SetName("floorplan.png");
    img.Write(output + img.GetName());

    Mesh mesh;
    float step = 0.02f;
    float xSize = max.x - min.x;
    float zSize = max.z - min.z;
    float xStep = xSize * step;
    float zStep = zSize * step;
    for (float x = 0; x < 1 - step * 0.5f; x += step)
    {
        for (float z = 0; z < 1 - step * 0.5f; z += step)
        {
            mesh.vertices.push_back(glm::vec3(min.x + x * xSize + xStep, 0, min.z + z * zSize) * 0.01f);
            mesh.vertices.push_back(glm::vec3(min.x + x * xSize, 0, min.z + z * zSize) * 0.01f);
            mesh.vertices.push_back(glm::vec3(min.x + x * xSize + xStep, 0, min.z + z * zSize + zStep) * 0.01f);

            mesh.vertices.push_back(glm::vec3(min.x + x * xSize, 0, min.z + z * zSize + zStep) * 0.01f);
            mesh.vertices.push_back(glm::vec3(min.x + x * xSize + xStep, 0, min.z + z * zSize + zStep) * 0.01f);
            mesh.vertices.push_back(glm::vec3(min.x + x * xSize, 0, min.z + z * zSize) * 0.01f);

            mesh.uv.push_back(glm::vec2(x + step, 1.0f - z));
            mesh.uv.push_back(glm::vec2(x, 1.0f - z));
            mesh.uv.push_back(glm::vec2(x + step, 1.0f - z - step));

            mesh.uv.push_back(glm::vec2(x, 1.0f - z - step));
            mesh.uv.push_back(glm::vec2(x + step, 1.0f - z - step));
            mesh.uv.push_back(glm::vec2(x, 1.0f - z));
        }
    }

    mesh.image = &img;

    std::vector<Mesh> model;
    model.push_back(mesh);
    File3d(output + "floorplan.obj", true).WriteModel(model);
}

void ExporterFloorplan::FillHoles()
{
    FloorPlanPoint p;
    std::map<FloorPlanPoint, bool> mask;
    for (p.x = min.x; p.x <= max.x; p.x++)
    {
        for (p.z = min.z; p.z <= max.z; p.z++)
        {
            if (heightmap.find(p) == heightmap.end())
            {
                for (int s = 0; s < 4; s++)
                {
                    int count = 0;
                    FloorPlanPoint q;
                    float value = 0;
                    glm::ivec4 color(0);
                    for (q.x = p.x - s; q.x <= p.x + s; q.x++)
                    {
                        q.z = p.z - s;
                        if (mask.find(q) == mask.end())
                        {
                            if (heightmap.find(q) != heightmap.end())
                            {
                                color += colormap[q];
                                value += heightmap[q];
                                count++;
                            }
                        }
                        q.z = p.z + s;
                        if (mask.find(q) == mask.end())
                        {
                            if (heightmap.find(q) != heightmap.end())
                            {
                                color += colormap[q];
                                value += heightmap[q];
                                count++;
                            }
                        }
                    }
                    for (q.z = p.z - s + 1; q.z <= p.z + s - 1; q.z++)
                    {
                        q.x = p.x - s;
                        if (mask.find(q) == mask.end())
                        {
                            if (heightmap.find(q) != heightmap.end())
                            {
                                color += colormap[q];
                                value += heightmap[q];
                                count++;
                            }
                        }
                        q.x = p.x + s;
                        if (mask.find(q) == mask.end())
                        {
                            if (heightmap.find(q) != heightmap.end())
                            {
                                color += colormap[q];
                                value += heightmap[q];
                                count++;
                            }
                        }
                    }
                    if (count > 0)
                    {
                        colormap[p] = color / count;
                        heightmap[p] = value / (float)count;
                        mask[p] = true;
                        break;
                    }
                }
            }
        }
    }
}
}
