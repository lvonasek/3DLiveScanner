#ifndef POSTPROCESSOR_TEXTURIZE_H
#define POSTPROCESSOR_TEXTURIZE_H

#include <data/dataset.h>
#include <data/depthmap.h>
#include <data/image.h>
#include <editor/rasterizer.h>

#include <map>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace oc {

struct TexelBase {
    unsigned short u, v;
    unsigned char submodel;
    unsigned char r, g, b;
};

struct Texel {
    glm::ivec4 color;
    double depth;
    unsigned char submodel;
    unsigned short u, v;
    int x, y;
};

class Texturize : Rasterizer {
public:
    std::vector<int> GetFrames();
    void Process(Dataset* dataset, std::string model, bool finalRender);
    void SetCallback(void(*callback)(int current, int count));
    void WriteTextures();

private:
    void LoadModel(std::string path);
    void ProjectFrames(bool edges);
    void RemoveBadFrames();

    //postprocessing of textures
    void AddTextureBevel(Image* image);
    void BevelShader(Image* image, glm::ivec4& bevel, glm::ivec4& color, int& x, int& y);

    //projecting frames into textures
    void ColorMapping(glm::ivec4* dx, glm::ivec4* dy);
    void ColorMappingBlur(glm::ivec4* dx, glm::ivec4* dy, glm::ivec4* nx, glm::ivec4* ny);
    void ColorMappingDebug(std::string filename, glm::ivec4* dx, glm::ivec4* dy);
    void DrawTexels(glm::ivec4* dx, glm::ivec4* dy);
    virtual void Process(unsigned long& index, int &x1, int &x2, int &y, glm::dvec3 &z1, glm::dvec3 &z2);

    //texels swapping
    std::vector<TexelBase> ReadTexels(std::string filename);
    void WriteTexels(std::string filename);

private:
    //per frame data
    Depthmap* depthmap;
    Image* frame;
    int* mapping;
    std::vector<Texel> texels;

    //input data
    Dataset* dataset;
    double cx, cy, fx, fy;
    std::map<int, double> frames;
    std::vector<Mesh> geom;
    int width, height, resolution, submodel;
};
}

#endif
