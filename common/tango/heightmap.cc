#include <data/dataset.h>
#include <data/file3d.h>
#include <tango/heightmap.h>
#include <sstream>

//threshold where edges are connected [centimeters]
#define CONNECTIVITY_THRESHOLD 10
//maximal decimation error [centimeters]
#define MAX_DECIMATION_ERROR 1
//maximal heights difference [meters]
#define MAXIMAL_DIFF 1.0f
//minimal accepted heights difference [meters]
#define MINIMAL_DIFF 0.1f
//minimal edge length [centimeters]
#define MINIMAL_EDGE 30

namespace oc {

bool operator<(const HeightmapPoint& lhs, const HeightmapPoint& rhs)
{
    return lhs.x < rhs.x || (lhs.x == rhs.x && (lhs.z < rhs.z));
}

HeightmapPoint centerPoint;

bool Comparator(const HeightmapEdge& a, HeightmapEdge& b){
    return a.GetKey().Distance2(centerPoint) < b.GetKey().Distance2(centerPoint);
}

Heightmap::Heightmap() {
    Clear();
}

Heightmap::Heightmap(std::string filepath) {

    //read header
    FILE* file = fopen(filepath.c_str(), "rb");
    fread(&min.x, sizeof(int), 1, file);
    fread(&min.z, sizeof(int), 1, file);
    fread(&max.x, sizeof(int), 1, file);
    fread(&max.z, sizeof(int), 1, file);

    //read data
    int w = max.x - min.x + 1;
    int h = max.z - min.z + 1;
    float* input = new float[w * h];
    fread(input, sizeof(float), w * h, file);
    int size = 0;
    HeightmapEdge e;
    HeightmapPoint p;
    fread(&size, sizeof(int), 1, file);
    for (unsigned int i = 0; i < size; i++) {
        fread(&p.x, sizeof(int), 1, file);
        fread(&p.z, sizeof(int), 1, file);
        fread(&e.a.x, sizeof(int), 1, file);
        fread(&e.a.z, sizeof(int), 1, file);
        fread(&e.b.x, sizeof(int), 1, file);
        fread(&e.b.z, sizeof(int), 1, file);
        edges[p] = e;
    }
    fclose(file);

    //process data
    std::map<HeightmapPoint, bool> toUpdate;
    for (p.x = min.x; p.x < max.x; p.x++) {
        for (p.z = min.z; p.z < max.z; p.z++) {
            int x = p.x - min.x;
            int y = p.z - min.z;
            float value = input[y * w + x];
            if (value < INT_MAX / 2) {
                heightmap[p] = input[y * w + x];
                toUpdate[p] = true;
            }
        }
    }
    UpdateDiffs(toUpdate);
    delete[] input;
}

void Heightmap::Clear()
{
    //clear data
    diffs.clear();
    edges.clear();
    heightmap.clear();

    //init AABB
    int bigNumber = 9999999;
    min.x = bigNumber;
    min.z = bigNumber;
    max.x =-bigNumber;
    max.z =-bigNumber;
}

void Heightmap::ConvertFrame(Tango3DR_Vector4* points, int size, std::vector<glm::mat4> pose)
{
    //convert point cloud into world space and colorize it
    Mesh pcl;
    for (unsigned int i = 0; i < size; i++)
    {
        glm::vec4 v = pose[COLOR_CAMERA] * glm::vec4(points[i][0], points[i][1], points[i][2], 1.0f);
        pcl.vertices.push_back(glm::vec3(v.x, v.y, v.z));
    }

    //process the data
    HeightmapPoint p, r;
    std::map<HeightmapPoint, bool> toUpdate;
    glm::mat4 matrix = pose[COLOR_CAMERA];
    for (glm::vec3& v : pcl.vertices)
    {
        if (v.y < matrix[3][1])
        {
            p.x = v.x * 100;
            p.z = v.z * 100;

            if (heightmap.find(p) == heightmap.end())
            {
                heightmap[p] = v.y;

                if (min.x > p.x) min.x = p.x;
                if (min.z > p.z) min.z = p.z;
                if (max.x < p.x) max.x = p.x;
                if (max.z < p.z) max.z = p.z;
            }
            else if (heightmap[p] < v.y)
            {
                heightmap[p] = v.y;
            }
            for (r.x = p.x - 1; r.x <= p.x + 1; r.x++) {
                for (r.z = p.z - 1; r.z <= p.z + 1; r.z++) {
                    toUpdate[r] = true;
                }
            }
        }
    }
    UpdateDiffs(toUpdate);
}

float Heightmap::CountScore(std::vector<HeightmapEdge>& areaEdges, glm::vec4& relocalisation) {
    if (areaEdges.empty() || edges.empty()) {
        return 0;
    }

    float score = 0;
    int limit = MINIMAL_EDGE;
    for (std::map<HeightmapPoint, HeightmapEdge>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
        float best = INT_MAX;
        for (HeightmapEdge& edge : areaEdges) {
            best = glm::min(best, it->second.Error(edge, relocalisation));
        }
        if (best < limit) {
            float percentage = (limit - best) / (float)limit;
            score += percentage;
        }
    }

    return score / (float)edges.size();
}

void Heightmap::DrawEdges(Image* img, int cx, int cz, float yaw, std::vector<HeightmapEdge> data, glm::ivec4 color)
{
    float s = sin(yaw);
    float c = cos(yaw);

    int w = img->GetWidth();
    int h = img->GetHeight();
    for (HeightmapEdge& e : data) {
        glm::ivec2 p1 = e.a.ToScreen(w, h, s, c, cx, cz);
        glm::ivec2 p2 = e.b.ToScreen(w, h, s, c, cx, cz);
        img->DrawLine(p1.x, p1.y, p2.x, p2.y, color);
    }
}

void Heightmap::DrawHeightmap(Image* img, int cx, int cz, float yaw)
{
    float s = sin(-yaw);
    float c = cos(-yaw);

    int w = img->GetWidth();
    int h = img->GetHeight();
    glm::ivec4 red(255, 0, 0, 255);
    glm::ivec4 blue(0, 0, 255, 255);
    glm::ivec4 white(255, 255, 255, 255);

    HeightmapPoint p;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            p = HeightmapPoint(x, y, w, h, s, c, cx, cz);
            if (heightmap.find(p) != heightmap.end()) {
                float diff = diffs[p];
                float value = -heightmap[p];
                glm::ivec4 color;
                color.r = glm::clamp((int)(diff * 200.0f), 0, 255);
                color.g = glm::clamp((int)(value * 200.0f), 0, 255);
                color.b = glm::clamp((int)(value * 100.0f), 0, 255);
                color.a = 255;
                img->DrawPixel(x, y, color);
            } else {
                img->DrawPixel(x, y, white);
            }
        }
    }

    //show position indicator
    for (int x = w / 2 - 1; x <= w / 2 + 1; x++) {
        for (int y = h / 2 - 1; y <= h / 2 + 1; y++) {
            img->DrawPixel(x, y, red);
        }
    }
}

void Heightmap::FillHoles()
{
    min.x -= 10;
    min.z -= 10;
    max.x += 10;
    max.z += 10;

    HeightmapPoint p;
    std::map<HeightmapPoint, bool> mask;
    for (p.x = min.x; p.x <= max.x; p.x++)
    {
        for (p.z = min.z; p.z <= max.z; p.z++)
        {
            if (heightmap.find(p) == heightmap.end())
            {
                for (int s = 0; s < 5; s++)
                {
                    int count = 0;
                    HeightmapPoint q;
                    float value = 0;
                    for (q.x = p.x - s; q.x <= p.x + s; q.x++)
                    {
                        q.z = p.z - s;
                        if (mask.find(q) == mask.end())
                        {
                            if (heightmap.find(q) != heightmap.end())
                            {
                                value += heightmap[q];
                                count++;
                            }
                        }
                        q.z = p.z + s;
                        if (mask.find(q) == mask.end())
                        {
                            if (heightmap.find(q) != heightmap.end())
                            {
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
                                value += heightmap[q];
                                count++;
                            }
                        }
                        q.x = p.x + s;
                        if (mask.find(q) == mask.end())
                        {
                            if (heightmap.find(q) != heightmap.end())
                            {
                                value += heightmap[q];
                                count++;
                            }
                        }
                    }
                    if (count > 0)
                    {
                        heightmap[p] = value / (float)count;
                        mask[p] = true;
                        break;
                    }
                }
            }
        }
    }
}


std::vector<HeightmapEdge> Heightmap::GetEdges(int cx, int cz, int maxEdges) {
    std::vector<HeightmapEdge> output;
    for (std::map<HeightmapPoint, HeightmapEdge>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
        output.push_back(it->second);
    }
    if (output.size() > maxEdges) {
        centerPoint.x = cx;
        centerPoint.z = cz;
        std::sort(output.begin(), output.end(), Comparator);
        output.erase(output.begin() + maxEdges - 1, output.begin() + output.size() - 1);
    }
    return output;
}

float Heightmap::GetHeight(HeightmapPoint p, bool minimal, int step) {
    float output = minimal ? INT_MAX : INT_MIN;
    HeightmapPoint q;
    for (q.x = p.x - step; q.x <= p.x + step; q.x++) {
        for (q.z = p.z - step; q.z <= p.z + step; q.z++) {
            if (heightmap.find(q) != heightmap.end()) {
                if (minimal) {
                    output = glm::min(output, heightmap[q]);
                } else {
                    output = glm::max(output, heightmap[q]);
                }
            }
        }
    }
    return output;
}

void Heightmap::Save(std::string filepath) {
    int w = max.x - min.x + 1;
    int h = max.z - min.z + 1;

    float *output = new float[w * h];
    HeightmapPoint p;
    for (p.x = min.x; p.x < max.x; p.x++) {
        for (p.z = min.z; p.z < max.z; p.z++) {
            int x = p.x - min.x;
            int y = p.z - min.z;
            if (heightmap.find(p) != heightmap.end()) {
               output[y * w + x] = heightmap[p];
            } else {
                output[y * w + x] = INT_MAX;
            }
        }
    }

    FILE *file = fopen(filepath.c_str(), "wb");
    fwrite(&min.x, sizeof(int), 1, file);
    fwrite(&min.z, sizeof(int), 1, file);
    fwrite(&max.x, sizeof(int), 1, file);
    fwrite(&max.z, sizeof(int), 1, file);
    fwrite(output, sizeof(float), w * h, file);
    int size = edges.size();
    fwrite(&size, sizeof(int), 1, file);
    for (std::map<HeightmapPoint, HeightmapEdge>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
        int keyx = it->first.x;
        int keyz = it->first.z;
        int ax = it->second.a.x;
        int az = it->second.a.z;
        int bx = it->second.b.x;
        int bz = it->second.b.z;
        fwrite(&keyx, sizeof(int), 1, file);
        fwrite(&keyz, sizeof(int), 1, file);
        fwrite(&ax, sizeof(int), 1, file);
        fwrite(&az, sizeof(int), 1, file);
        fwrite(&bx, sizeof(int), 1, file);
        fwrite(&bz, sizeof(int), 1, file);
    }
    fclose(file);

    delete[] output;
}

void Heightmap::Update(int cx, int cz) {

    std::vector<HeightmapPoint> points = GetNearestPoints(cx, cz);
    std::vector<HeightmapEdge> input = GetValidEdges(points);

    for (int i = input.size() - 1; i >= 1; i--) {

        //decimate edge
        int j = i - 1;
        HeightmapPoint start = input[i].b;
        HeightmapPoint end = input[j].a;
        for (; j >= 0; j--) {
            end = input[j].a;
            if (input[j + 1].a.Distance2(input[j].b) == 0) {

                bool ok = true;
                for (int k = i - 1; k >= j; k--) {
                    HeightmapPoint middle = input[k].b;
                    double a = sqrt(start.Distance2(middle));
                    double b = sqrt(middle.Distance2(end));
                    double c = sqrt(start.Distance2(end));
                    if (a + b > c + MAX_DECIMATION_ERROR) {
                        ok = false;
                        break;
                    }
                }

                if (!ok) {
                    j++;
                    end = input[j].a;
                    break;
                }
            } else {
                j++;
                end = input[j].a;
                break;
            }
        }

        //delete unnecessary vertices
        input[i].a = end;
        for (int k = i - 1; k > j; k--) {
            input.erase(input.begin() + k);
            i--;
        }
    }

    //clear too short edges
    for (int i = input.size() - 1; i >= 0; i--) {
        int len = input[i].a.Distance2(input[i].b);
        if (len < MINIMAL_EDGE * MINIMAL_EDGE) {
            input.erase(input.begin() + i);
            if (input.empty()) {
                break;
            }
        }
    }

    //get keys
    std::vector<HeightmapPoint> keys;
    for (HeightmapEdge& i : input) {
        HeightmapEdge e;
        e.a = i.a;
        e.b = i.b;
        keys.push_back(e.GetKey());
    }

    //clear the area
    std::vector<HeightmapPoint> toDelete;
    for (std::map<HeightmapPoint, HeightmapEdge>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
        float threshold = sqrt(it->second.a.Distance2(it->second.b)) * 0.25f;
        threshold *= threshold;
        for (HeightmapPoint& p : keys) {
            if (it->first.Distance2(p) < threshold) {
                toDelete.push_back(it->first);
                break;
            }
        }
    }
    for (HeightmapPoint& p : toDelete) {
        edges.erase(p);
    }

    //add edges into structure
    for (int i = 0; i < input.size(); i++) {
        HeightmapEdge e;
        e.a = input[i].a;
        e.b = input[i].b;
        edges[e.GetKey()] = e;
    }
}

std::vector<HeightmapPoint> Heightmap::GetNearestPoints(int cx, int cz) {

    HeightmapPoint p, q;
    std::vector<HeightmapPoint> output;
    for (int i = 0; i < 360; i++) {
        float d = 0;
        float s = sin(glm::radians((float)i));
        float c = cos(glm::radians((float)i));
        while (true) {
            p.valid = true;
            p.x = (int)(s * d - c * d) + cx;
            p.z = (int)(c * d + s * d) + cz;
            if (diffs.find(p) != diffs.end()) {
                if (diffs[p] > MINIMAL_DIFF) {
                    break;
                }
            }
            if (heightmap.find(p) == heightmap.end()) {
                p.valid = false;
                break;
            }
            d++;
        }

        if ((p.x != q.x) || (p.z != q.z)) {
            output.push_back(p);
        }
        q.x = p.x;
        q.z = p.z;
    }

    return output;
}

std::vector<HeightmapEdge> Heightmap::GetValidEdges(std::vector<HeightmapPoint>& input) {
    std::vector<HeightmapEdge> output;
    if (!input.empty()) {
        int threshold = CONNECTIVITY_THRESHOLD * CONNECTIVITY_THRESHOLD;
        HeightmapPoint last = input[input.size() - 1];
        for (HeightmapPoint& r : input) {
            bool valid = r.valid && last.valid && (r.Distance2(last) < threshold);
            if (valid) {
                HeightmapEdge e;
                e.a = last;
                e.b = r;
                output.push_back(e);
            }
            last = r;
        }
    }
    return output;
}

void Heightmap::UpdateDiffs(std::map<HeightmapPoint, bool> &toUpdate) {
    HeightmapPoint p, r;
    for (std::map<HeightmapPoint, bool>::const_iterator it = toUpdate.begin(); it != toUpdate.end(); ++it) {
        p = it->first;
        if (heightmap.find(p) != heightmap.end()) {
            float diff = 0;
            float value = heightmap[p];
            for (r.x = p.x - 1; r.x <= p.x + 1; r.x++) {
                for (r.z = p.z - 1; r.z <= p.z + 1; r.z++) {
                    if (heightmap.find(r) != heightmap.end()) {
                        diff = glm::max(diff, fabs(heightmap[r] - value));
                    }
                }
            }
            if (diff > MAXIMAL_DIFF) {
                diff = MAXIMAL_DIFF;
            }
            if (fabs(diff) > MINIMAL_DIFF) {
                diffs[p] = diff;
            } else if (diffs.find(p) != diffs.end()) {
                diffs.erase(p);
            }
        }
    }
}
}
