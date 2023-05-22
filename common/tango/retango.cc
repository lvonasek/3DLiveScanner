#include "tango/retango.h"

#include <ctime>
#include <delaunay.h>

namespace oc {

    Retango::Retango() : finished(0), mask(0), rgb(0) {
        minDff = 0.1f;
        maxDst = 0.25f;
        resolution = 0.04f;

        sorting = 0;
        estimating = 0;
        adding = 0;
        filling = 0;
        masking = 0;
        pairing = 0;
        walling = 0;
    }

    Retango::~Retango() {
        if (finished)
            delete[] finished;
        if (mask)
            delete[] mask;
    }

    void Retango::ADD(std::vector<Component> components, glm::mat4 pose, bool extraEstimators) {

        if (components.empty())
            return;

        clock_t step0 = clock();
        Component longest;
        std::sort(components.begin(), components.end(), Comparator());
        longest = components[0];
        if (!longest.valid)
            return;

        //estimate walls
        clock_t step1 = clock();
        bool inc = false;
        float last = 99999;
        std::vector<glm::vec4> top[2];
        for (int i = 0; i < 2; i++) {
            for (Edge& e : longest.edges) {
                if (last < e.point[0].y) {
                    if (inc)
                        top[0].push_back(e.point[0]);
                    inc = true;
                } else {
                    if (inc)
                        top[1].push_back(e.point[0]);
                    inc = false;
                }
                last = e.point[0].y;
            }
        }

        //add walls
        clock_t step2 = clock();
        glm::mat4 world2camera = glm::inverse(pose);
        if (extraEstimators) {
            for (std::vector<glm::vec4> &k : top) {
                for (unsigned int i = 1; i < k.size(); i++) {
                    bool visible = true;
                    for (int j = 0; j < 2; j++) {
                        glm::vec4 point = k[i - j];
                        point = world2camera * glm::vec4(point.x, point.y, point.z, 1.0f);
                        float x = 0.5f + 0.5f * point.x / fabs(point.z * point.w);
                        float y = 0.5f + 0.5f * point.y / fabs(point.z * point.w);
                        if (fabs(point.z) > 4) visible = false;
                        if (x < 0) visible = false;
                        if (x > 1) visible = false;
                        if (y < 0) visible = false;
                        if (y > 1) visible = false;
                    }
                    if (visible) {
                        AddLine(Vec4ToVec3(k[i - 1]), Vec4ToVec3(k[i]), world2camera);
                    }
                }
            }
        }

        //fill holes
        clock_t step3 = clock();
        for (int i = 1; i < components.size(); i++) {
            if (!components[i].valid || (components[i].edges.size() < 2))
                continue;

            //check if the hole is flat
            bool valid = true;
            Edge previous = components[i].edges[0];
            Edge current = components[i].edges[1];
            glm::vec3 a = current.point[1] - previous.point[0];
            glm::vec3 b = current.point[1] - previous.point[1];
            glm::vec3 normal = glm::cross(a, b);
            for (Edge& c : components[i].edges) {
                a = c.point[1] - previous.point[0];
                b = c.point[1] - previous.point[1];
                glm::vec3 n = glm::cross(a, b);
                if (fabs(n.x * normal.x + n.y * normal.y + n.z * normal.z) < 0.25f) {
                    valid = false;
                    break;
                }
                previous = current;
            }
            if (!valid)
                continue;

            //add voxel in component center
            glm::vec4 center(0);
            for (Edge& e : components[i].edges) {
                center += e.point[0];
            }
            center /= (float)components[i].edges.size();
            AddVoxel(center, world2camera);
            for (Edge& e1 : components[i].edges) {
                AddVoxel((center + e1.point[1]) * 0.5f, world2camera);
            }
        }
        clock_t step4 = clock();

        //profiling
        sorting = int(step1 - step0) / 1000;
        estimating = int(step2 - step1) / 1000;
        adding = int(step3 - step2) / 1000;
        filling = int(step4 - step3) / 1000;
    }

    void Retango::ADD(std::vector<glm::vec4>& p, glm::mat4 pose, Image* img) {
        glm::mat4 world2camera = glm::inverse(pose);
        input.clear();
        merged.clear();
        output.clear();
        for (glm::vec4& v : p) {
            input.emplace_back(v.x, v.y, v.z);
            merged.push_back(v);
            glm::vec4 point = world2camera * glm::vec4(v.x, v.y, v.z, 1.0f);
            point.w = v.w;
            output.push_back(point);
        }
        for (glm::vec4& v : estimated) {
            merged.push_back(v);
            glm::vec4 point = world2camera * glm::vec4(v.x, v.y, v.z, 1.0f);
            point.w = v.w;
            output.push_back(point);
        }
        UpdateCaches(img, pose);
    }

    std::string Retango::DBG() {
        std::string output = "";
        char buffer[4096];
        if (masking > 10) {
            sprintf(buffer, "Masking frame: %dms\n", masking);
            output += buffer;
        }
        if (pairing > 10) {
            sprintf(buffer, "Pairing points: %dms\n", pairing);
            output += buffer;
        }
        if (triangling > 10) {
            sprintf(buffer, "Triangling points: %dms\n", triangling);
            output += buffer;
        }
        if (walling > 10) {
            sprintf(buffer, "Wall estimation: %dms\n", walling);
            output += buffer;
        }
        if (sorting > 10) {
            sprintf(buffer, "Sorting components: %dms\n", sorting);
            output += buffer;
        }
        if (estimating > 10) {
            sprintf(buffer, "Estimating walls: %dms\n", estimating);
            output += buffer;
        }
        if (adding > 10) {
            sprintf(buffer, "Adding walls: %dms\n", adding);
            output += buffer;
        }
        if (filling > 10) {
            sprintf(buffer, "Filling holes: %dms\n", filling);
            output += buffer;
        }
        return output;
    }

    Tango3DR_PointCloud* Retango::PCL(double timestamp) {
#ifdef ANDROID
        Tango3DR_PointCloud* cloud = new Tango3DR_PointCloud();
        Tango3DR_PointCloud_init(output.size(), cloud);
        for (unsigned int i = 0; i < output.size(); i++) {
            cloud->points[i][0] = output[i].x;
            cloud->points[i][1] = output[i].y;
            cloud->points[i][2] = output[i].z;
            cloud->points[i][3] = output[i].w;
        }
        cloud->timestamp = timestamp;
        return cloud;
#else
        return 0;
#endif
    }

    void Retango::RES(float value) {
        resolution = value;
    }

    void Retango::UPD(Image* img, glm::mat4 pose, bool extraEstimators) {
        clock_t step0 = clock();
        UpdateDelaunayEstimation(pose);
        clock_t step1 = clock();
        UpdateMasked(pose);
        clock_t step2 = clock();
        UpdatePairEstimation(pose);
        clock_t step3 = clock();
        if (extraEstimators) UpdateWallEstimation(pose);
        clock_t step4 = clock();

        //profiling
        triangling = int (step1 - step0) / 1000;
        masking = int(step2 - step1) / 1000;
        pairing = int(step3 - step2) / 1000;
        walling = int(step4 - step3) / 1000;
    }

    void Retango::AddLine(glm::vec3 v, glm::vec3 t, glm::mat4& world2camera) {
        float len = glm::length(t - v);
        glm::vec3 dir = glm::normalize(t - v);
        for (float f = resolution; f <= len - resolution; f += resolution) {
            AddVoxel(glm::vec4(v + dir * f, 1.0f), world2camera);
        }
    }

    void Retango::AddTriangle(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        glm::vec3 ab = a - b;
        glm::vec3 bc = b - c;
        glm::vec3 ca = c - a;
        float lab = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
        float lbc = bc.x * bc.x + bc.y * bc.y + bc.z * bc.z;
        float lca = ca.x * ca.x + ca.y * ca.y + ca.z * ca.z;

        if ((lab > minDff) || (lbc > minDff) || (lca > minDff))
            return;

        if ((lab >= lbc) && (lab >= lca))
        {
            if (lab > resolution)
            {
                AddTriangle((a + b) * 0.5f, a, c);
                AddTriangle((a + b) * 0.5f, b, c);
            } else {
                estimated.push_back(glm::vec4((a + b + c) / 3.0f, 1.0f));
            }
            return;
        }
        if ((lbc >= lab) && (lbc >= lca))
        {
            if (lbc > resolution)
            {
                AddTriangle((b + c) * 0.5f, a, b);
                AddTriangle((b + c) * 0.5f, a, c);
            } else {
                estimated.push_back(glm::vec4((a + b + c) / 3.0f, 1.0f));
            }
            return;
        }
        if ((lca >= lab) && (lca >= lbc))
        {
            if (lca > resolution)
            {
                AddTriangle((c + a) * 0.5f, b, a);
                AddTriangle((c + a) * 0.5f, b, c);
            } else {
                estimated.push_back(glm::vec4((a + b + c) / 3.0f, 1.0f));
            }
            return;
        }
    }

    void Retango::AddVoxel(glm::vec4 i, glm::mat4 &world2camera) {

        //convert into mask coordinate system
        glm::vec4 v = glm::vec4(i.x, i.y, i.z, 1.0f);
        v = world2camera * v;
        v /= fabs(v.z * v.w);
        int x = (int) ((0.5f + 0.5f * v.x) * (width - 1));
        int y = (int) ((0.5f + 0.5f * v.y) * (height - 1));
        if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
            return;

        //do not add duplicates
        if (finished[y * width + x])
            return;
        for (int k = glm::max(x - 1, 0); k <= glm::min(x, width - 1); k++)
            for (int l = glm::max(y - 1, 0); l <= glm::min(y, height - 1); l++)
                finished[l * width + k] = true;

        //add voxel into estimated points
        estimated.push_back(i);
    }

    void Retango::GetColor(glm::ivec3 &output, int mem) {
        bool valid = true;
        if (!rgb) valid = false;
        else if (mem < 0) valid = false;
        else if (mem - 3 >= size) valid = false;

        if (valid) {
            output.r = rgb->GetData()[mem + 0];
            output.g = rgb->GetData()[mem + 1];
            output.b = rgb->GetData()[mem + 2];
        } else {
            output.r = 0;
            output.g = 0;
            output.b = 0;
        }
    }

    bool Retango::IsMasked(glm::vec4 v, glm::vec4 t) {

        //RGB frame test
        int x1 = (int) ((0.5f + 0.5f * v.x) * (width - 1));
        int y1 = (int) ((0.5f + 0.5f * v.y) * (height - 1));
        int x2 = (int) ((0.5f + 0.5f * t.x) * (width - 1));
        int y2 = (int) ((0.5f + 0.5f * t.y) * (height - 1));
        if (!LineTest(x1, y1, x2, y2))
            return true;

        //point cloud mask test
        for (int f = 1; f <= 9; f++) {
            glm::vec4 i = glm::lerp(v, t, f * 0.1f);
            int x = (int) ((0.5f + 0.5f * i.x) * (width - 1));
            int y = (int) ((0.5f + 0.5f * i.y) * (height - 1));
            if ((x >= 0) && (y >= 0) && (x < width) && (y < height)) {
                if (mask[y * width + x]) {
                    return true;
                }
            } else {
                return false;
            }
        }
        return false;
    }


    bool Retango::LineTest(int x1, int y1, int x2, int y2) {

        //Liang & Barsky clipping
        int w = x2 - x1;
        int h = y2 - y1;
        double t1 = 0, t2 = 1;
        if (RectTest(-w, x1, t1, t2) && RectTest(w, width - x1, t1, t2)  &
            RectTest(-h, y1, t1, t2) && RectTest(h, height - y1, t1, t2) ) {
            int a0, a1, c0, c1, i, p, mem;

            //clip line
            if (t1 > 0) {
                w = x2 - x1;
                x1 += t1 * w;
                y1 += t1 * h;
            } else
                t1 = 0;
            if (t2 < 1) {
                w = x2 - x1;
                t2 -= t1;
                x2 = (int) (x1 + t2 * w);
                y2 = (int) (y1 + t2 * h);
            }

            //line in x axis nearby
            if (w > h) {
                //count Bresenham's alg. variables
                c0 = 2 * h;
                p = c0 - w;
                c1 = p - w;

                //direction from left to right
                if (x2 >= x1) {
                    a0 = 1;
                } else {
                    a0 = -1;
                }

                //direction from top to bottom
                if (y2 >= y1) {
                    a1 = a0 + width;
                } else {
                    a1 = a0 - width;
                }
            }

            //line in y axis nearby
            else {
                //count Bresenham's alg. variables
                c0 = 2 * w;
                p = c0 - h;
                c1 = p - h;

                //direction from top to bottom
                if (y2 >= y1) {
                    a0 = width;
                } else {
                    a0 = -width;
                }

                //direction from left to right
                if (x2 >= x1) {
                    a1 = a0 + 1;
                } else {
                    a1 = a0 - 1;
                }
            }

            //set color channels
            a0 *= 4;
            a1 *= 4;

            //Bresenham's algorithm
            i = 0;
            mem = (x1 + y1 * width) * 4;
            glm::ivec3 lastColor, newColor;
            GetColor(lastColor, mem);
            for (w--; w >= 0; w--, i++) {

                //interpolate
                if (p < 0) {
                    p += c0;
                    mem += a0;
                } else {
                    p += c1;
                    mem += a1;
                }

                //update color data
                GetColor(newColor, mem);
                if ((i > 1) && (w > 1)) {
                    if (abs(newColor.r - lastColor.r) > 16)
                        return false;
                    if (abs(newColor.g - lastColor.g) > 16)
                        return false;
                    if (abs(newColor.b - lastColor.b) > 16)
                        return false;
                }
                lastColor = newColor;
            }
            return true;
        }
        return false;
    }

    bool Retango::RectTest(double p, double q, double &t1, double &t2) {

        //negative cutting
        if (p < 0) {
            double t = q/p;

            //cut nothing
            if (t > t2)
                return false;
            //cut the first coordinate
            else if (t > t1)
                t1 = t;
        }

        //positive cutting
        else if (p > 0) {
            double t = q/p;

            //cut nothing
            if (t < t1)
                return false;
            //cut the second coordinate
            else if (t < t2)
                t2 = t;
        }

        //line is right to left(or bottom to top)
        else if (q < 0)
            return false;

        //line is at least partly in the viewport
        return true;
    }

    void Retango::UpdateCaches(Image* img, glm::mat4& pose) {

        //set dimension of masks
        int w = img->GetWidth();
        int h = img->GetHeight();
        bool apply = false;
        if (!mask) {
            apply = true;
        } else if ((width != w) || (height != h)) {
            delete[] finished;
            delete[] mask;
            apply = true;
        }

        //apply mask dimensions
        if (apply) {
            width = w;
            height = h;
            finished = new bool[width * height];
            mask = new bool[width * height];
        }

        //clear all caches
        converted.clear();
        estimated.clear();
        for (unsigned int i = 0; i < width * height; i++) {
            finished[i] = false;
            mask[i] = false;
        }
        rgb = img;
        size = width * height * 4;

        //create points in mask coordinates
        glm::mat4 world2camera = glm::inverse(pose);
        for (glm::vec3& i : input) {
            glm::vec4 v = glm::vec4(i, 1.0f);
            v = world2camera * v;
            v.x /= fabs(v.z * v.w);
            v.y /= fabs(v.z * v.w);
            converted.push_back(v);
        }
    }

    void Retango::UpdateDelaunayEstimation(glm::mat4& pose) {
        if (input.size() < 3)
            return;

        glm::mat4 world2camera = glm::inverse(pose);
        std::vector<dln::Vector2<float> > points2d;
        for (glm::vec3& i : input) {
            glm::vec4 v = glm::vec4(i, 1.0f);
            v = world2camera * v;
            v /= fabs(v.z * v.w);
            points2d.push_back(dln::Vector2<float>(v.x, v.y, i));
        }

        std::vector<dln::Triangle<float> > triangles = dln::Delaunay<float>().triangulate(points2d);
        for (unsigned int i = 0; i < triangles.size(); i++) {
            dln::Triangle<float> t = triangles[i];
            if (t.isBad) continue;
            AddTriangle(t.p1.p, t.p2.p, t.p3.p);
        }
    }

    void Retango::UpdateMasked(glm::mat4& pose, int s) {
        glm::mat4 world2camera = glm::inverse(pose);
        for (glm::vec4& i : estimated) {
            glm::vec4 v = glm::vec4(i.x, i.y, i.z, 1.0f);
            v = world2camera * v;
            v /= fabs(v.z * v.w);
            int x = (int) ((0.5f + 0.5f * v.x) * (width - 1));
            int y = (int) ((0.5f + 0.5f * v.y) * (height - 1));
            for (int k = glm::max(x - s, 0); k <= glm::min(x + s, width - 1); k++)
                for (int l = glm::max(y - s, 0); l <= glm::min(y + s, height - 1); l++)
                    mask[l * width + k] = true;
        }
        for (glm::vec3& i : input) {
            glm::vec4 v = glm::vec4(i, 1.0f);
            v = world2camera * v;
            v /= fabs(v.z * v.w);
            int x = (int) ((0.5f + 0.5f * v.x) * (width - 1));
            int y = (int) ((0.5f + 0.5f * v.y) * (height - 1));
            for (int k = glm::max(x - s, 0); k <= glm::min(x + s, width - 1); k++)
                for (int l = glm::max(y - s, 0); l <= glm::min(y + s, height - 1); l++)
                    mask[l * width + k] = true;
        }
    }

    void Retango::UpdatePairEstimation(glm::mat4 &pose) {
        glm::mat4 world2camera = glm::inverse(pose);
        for (unsigned int i = 0; i < input.size(); i++) {
            glm::vec3 v = input[i];
            for (unsigned int j = 0; j < i; j++) {
                glm::vec3 t = input[j];

                //featureless floors
                if (fabs(t.y - v.y) < minDff) {
                    if (!IsMasked(converted[i], converted[j])) {
                        AddLine(v, t, world2camera);
                    }
                }

                //extend wall pointcloud
                if (fabs(t.x - v.x) < minDff) {
                    if (fabs(t.z - v.z) < minDff) {
                        glm::vec3 diff = v - t;
                        diff.y = 0;
                        float dst = glm::length(diff);
                        if ((dst > resolution) && (dst < maxDst)) {
                            if (!IsMasked(converted[i], converted[j])) {
                                AddLine(v, t, world2camera);
                            }
                        }
                    }
                }
            }
        }
    }

    void Retango::UpdateWallEstimation(glm::mat4 &pose) {

        //generate all possibilities
        glm::mat4 world2camera = glm::inverse(pose);
        std::vector<MaskPair> pairs;
        for (unsigned int i = 0; i < input.size(); i++) {
            MaskPair p;
            p.a2d = converted[i];
            p.a3d = input[i];
            p.b3d = glm::vec3(p.a3d.x, pose[3][1] + 0.5f, p.a3d.z);
            p.b2d = world2camera * glm::vec4(p.b3d, 1.0f);
            p.b2d.x /= fabs(p.b2d.z * p.b2d.w);
            p.b2d.y /= fabs(p.b2d.z * p.b2d.w);
            pairs.push_back(p);
        }

        //get extreme points
        float farL = 0; float posL = 0;
        float farR = 0; float posR = 0;
        float farU = 0; float posU = 0;
        float farD = 0; float posD = 0;
        float u = -9999, d = -u;
        for (MaskPair& p : pairs) {
            if ((p.a2d.x < 0) && (farL < p.a2d.z)) { farL = p.a2d.z; posL = p.a2d.x; }
            if ((p.a2d.x > 0) && (farR < p.a2d.z)) { farR = p.a2d.z; posR = p.a2d.x; }
            if ((p.a2d.y < 0) && (farU < p.a2d.z)) { farU = p.a2d.z; posU = p.a2d.y; }
            if ((p.a2d.y > 0) && (farD < p.a2d.z)) { farD = p.a2d.z; posD = p.a2d.y; }
            if (u < p.a3d.y) u = p.a3d.y;
            if (d > p.a3d.y) d = p.a3d.y;
        }

        //check if point cloud is enough good for wall estimation
        if (u < pose[3][1])
            return;
        if (d > pose[3][1])
            return;

        //get the far plane estimation
        bool horizontal = fabs(farL - farR) > fabs(farU - farD);
        float far1 = horizontal ? farL : farU;
        float far2 = horizontal ? farR : farD;

        //process the most far estimations
        for (MaskPair& p : pairs) {
            if (p.b3d.y - p.a3d.y < 2.0f)
                continue;

            float far;
            if (horizontal)
                far = glm::lerp(posL, posR, (posR - p.a2d.x) / (posR - posL));
            else
                far = glm::lerp(posU, posD, (posD - p.a2d.y) / (posD - posU));

            if (far - p.a2d.z < minDff) {
                if (!IsMasked(p.a2d, p.b2d)) {
                    AddLine(p.a3d, p.b3d, world2camera);
                }
            }
        }
    }
}
