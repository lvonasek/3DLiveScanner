#ifndef DATA_DEPTHMAP_H
#define DATA_DEPTHMAP_H

#include <data/dataset.h>
#include <data/mesh.h>
#include <editor/rasterizer.h>

#define DEPTHMAP_SCALE 5000.0f

namespace oc {

class Depthmap : Rasterizer {
public:
    Depthmap(int w, int h);

    virtual ~Depthmap();

    void BevelMask(int mw, int mh);

    bool CanPass(int& x, int& y, double& value);

    bool CanWrite(int& x, int& y, double& value);

    double CountDiff(Depthmap* data);

    void FromPCL(Dataset* dataset, int poseIndex, bool fillHoles = true);

    double GetDepth(int x, int y);

    Image* GetRGB() { return rgb; }

    void MaskEdges(int bevel);

    void Render(std::vector<Mesh>& geom, glm::mat4& world2screen);

    void WritePNG16(std::string filename);

private:

    void BevelShader(int& count, glm::ivec2& p, int& bevel);

    void EdgeShader(double& last, int& count, glm::ivec2& p, int& bevel);

    virtual void Process(unsigned long& index, int &x1, int &x2, int &y, glm::dvec3 &z1, glm::dvec3 &z2);

    double* depth;
    Image* rgb;
    Image* texture;
    std::vector<glm::ivec2> mask;
    int submodel, width, height;
};
}

#endif
