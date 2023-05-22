#ifndef TANGO_RETANGO_H
#define TANGO_RETANGO_H

#include <tango_3d_reconstruction_api.h>
#include "data/file3d.h"
#include "gl/opengl.h"

namespace oc {

    struct Edge {
        glm::vec4 point[2];
    };

    struct Component {
        std::vector<Edge> edges;
        bool closed;
        bool valid;
    };

    struct Comparator : public std::binary_function<Component, Component, bool>
    {
        bool operator()(const Component& a, const Component& b) const
        {
            return a.edges.size() > b.edges.size();
        }
    };

    struct MaskPair {
        glm::vec4 a2d;
        glm::vec4 b2d;
        glm::vec3 a3d;
        glm::vec3 b3d;
    };

    class Retango {
    public:
        Retango();
        ~Retango();
        void ADD(std::vector<Component> components, glm::mat4 pose, bool extraEstimators = true);
        void ADD(std::vector<glm::vec4>& p, glm::mat4 pose, Image* img);
        std::string DBG();
        std::vector<glm::vec4> GET() { return merged; }
        Tango3DR_PointCloud* PCL(double timestamp);
        void RES(float value);
        void UPD(Image* img, glm::mat4 pose, bool extraEstimators = true);
        static glm::vec3 Vec4ToVec3(glm::vec4 v) { return glm::vec3(v.x, v.y, v.z); }
    private:
        void AddLine(glm::vec3 v, glm::vec3 t, glm::mat4& world2camera);
        void AddTriangle(glm::vec3 a, glm::vec3 b, glm::vec3 c);
        void AddVoxel(glm::vec4 i, glm::mat4& world2camera);
        void GetColor(glm::ivec3& output, int mem);
        bool IsMasked(glm::vec4 v, glm::vec4 t);
        bool LineTest(int x1, int y1, int x2, int y2);
        bool RectTest(double p, double q, double &t1, double &t2);
        void UpdateCaches(Image* img, glm::mat4& pose);
        void UpdateDelaunayEstimation(glm::mat4& pose);
        void UpdateMasked(glm::mat4& pose, int s = 15);
        void UpdatePairEstimation(glm::mat4 &pose);
        void UpdateWallEstimation(glm::mat4 &pose);

        //points
        std::vector<glm::vec4> converted;
        std::vector<glm::vec4> estimated;
        std::vector<glm::vec3> input;
        std::vector<glm::vec4> merged;
        std::vector<glm::vec4> output;

        //masks
        bool* finished;
        bool* mask;
        Image* rgb;
        int width, height, size;

        //configuration
        float minDff;
        float maxDst;
        float resolution;

        //profiling
        int sorting;
        int estimating;
        int adding;
        int filling;
        int masking;
        int pairing;
        int triangling;
        int walling;
    };
}
#endif