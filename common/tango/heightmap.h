#ifndef TANGO_HEIGHTMAP_H
#define TANGO_HEIGHTMAP_H

#include "gl/opengl.h"
#include <map>
#include <tango_3d_reconstruction_api.h>

namespace oc {

struct HeightmapPoint {
    int x;
    int z;
    bool valid;

    HeightmapPoint() {
        valid = true;
    }

    HeightmapPoint(int x, int z) {
        this->x = x;
        this->z = z;
        valid = true;
    }

    HeightmapPoint(int x, int y, int w, int h, float s, float c, int cx, int cz) {
        int qx = x - w / 2;
        int qz = y - h / 2;
        this->x = s * qx - c * qz + cx;
        this->z = c * qx + s * qz + cz;
        valid = false;
    }

    int Distance2(const HeightmapPoint& p) const {
        int a = p.x - x;
        int b = p.z - z;
        return a * a + b * b;
    }

    glm::ivec2 ToScreen(int w, int h, float s, float c, int cx, int cz) const {
        int qx = x - cx;
        int qz = z - cz;
        int px = s * qx - c * qz;
        int pz = c * qx + s * qz;
        return glm::ivec2(-px + w / 2, -pz + h / 2);
    }

    HeightmapPoint Transform(glm::vec4& relocalisation) {
        HeightmapPoint output;
        float s = sin(-glm::radians(relocalisation.w));
        float c = cos(-glm::radians(relocalisation.w));
        output.x = s * x - c * z + relocalisation.x;
        output.z = c * x + s * z + relocalisation.z;
        return output;
    }
};

struct HeightmapEdge {
    HeightmapPoint a;
    HeightmapPoint b;

    float Angle() const {
        return glm::degrees(atan2(a.x - b.x, b.z - a.z));
    }

    float Distance(HeightmapPoint& e) const {
        //https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/
        HeightmapPoint ab(b.x - a.x, b.z - a.z);
        HeightmapPoint be(e.x - b.x, e.z - b.z);
        HeightmapPoint ae(e.x - a.x, e.z - a.z);

        // Minimum distance from
        // point E to the line segment
        if (ab.x * be.x + ab.z * be.z > 0) {
            int x = e.x - b.x;
            int z = e.z - b.z;
            return sqrt(x * x + z * z);
        } else if (ab.x * ae.x + ab.z * ae.z < 0) {
            int x = e.x - a.x;
            int z = e.z - a.z;
            return sqrt(x * x + z * z);
        } else {
            int x1 = ab.x;
            int z1 = ab.z;
            int x2 = ae.x;
            int z2 = ae.z;
            return abs(x1 * z2 - z1 * x2) / sqrt(x1 * x1 + z1 * z1);
        }
    }

    HeightmapPoint GetKey() const {
        HeightmapPoint p;
        p.x = (a.x + b.x) / 2;
        p.z = (a.z + b.z) / 2;
        return p;
    }

    float Error(HeightmapEdge& e, glm::vec4& relocalisation) const {

        //count angular error (in degrees)
        HeightmapEdge edge = e.Transform(relocalisation);
        float angleError = Angle() - edge.Angle();
        while (angleError > 360) {
            angleError -= 360;
        }
        while (angleError < 0) {
            angleError += 360;
        }

        //count distance error (in centimeters)
        HeightmapPoint center = GetKey();
        float dstError = edge.Distance(center);

        return angleError + dstError;
    }

    HeightmapEdge Offset(int x, int z) {
        HeightmapEdge output;
        output.a = a;
        output.b = b;
        output.a.x += x;
        output.a.z += z;
        output.b.x += x;
        output.b.z += z;
        return output;
    }

    glm::vec4 Relocalisation(HeightmapEdge& to, float angleOffset = 0) {
        float angle = Angle() - to.Angle() - 90 + angleOffset;
        float s = sin(-glm::radians(angle));
        float c = cos(-glm::radians(angle));
        HeightmapPoint current = GetKey();
        HeightmapPoint tcenter = to.GetKey();

        glm::vec4 output(0);
        output.x = current.x - (s * tcenter.x - c * tcenter.z);
        output.z = current.z - (c * tcenter.x + s * tcenter.z);
        output.w = angle;
        return output;
    }

    HeightmapEdge Transform(glm::vec4 relocalisation) {
        HeightmapEdge output;
        output.a = a.Transform(relocalisation);
        output.b = b.Transform(relocalisation);
        return output;
    }
};

bool operator<(const HeightmapPoint& lhs, const HeightmapPoint& rhs);

class Heightmap {
public:
    Heightmap();

    Heightmap(std::string filepath);

    void Clear();

    void ConvertFrame(Tango3DR_Vector4* points, int size, std::vector<glm::mat4> pose);

    float CountScore(std::vector<HeightmapEdge>& areaEdges, glm::vec4& relocalisation);

    void DrawEdges(Image* img, int cx, int cz, float yaw, std::vector<HeightmapEdge> data, glm::ivec4 color);

    void DrawHeightmap(Image* img, int cx, int cz, float yaw);

    void FillHoles();

    std::vector<HeightmapEdge> GetEdges(int cx = 0, int cz = 0, int maxEdges = 10000000);

    float GetHeight(HeightmapPoint p, bool minimal = true, int step = 5);

    void Save(std::string filepath);

    void Update(int cx, int cz);

private:
    std::vector<HeightmapPoint> GetNearestPoints(int cx, int cz);

    std::vector<HeightmapEdge> GetValidEdges(std::vector<HeightmapPoint>& input);

    void UpdateDiffs(std::map<HeightmapPoint, bool>& toUpdate);

    HeightmapPoint min, max;

    std::map<HeightmapPoint, float> diffs;

    std::map<HeightmapPoint, HeightmapEdge> edges;

    std::map<HeightmapPoint, float> heightmap;
};
}

#endif
