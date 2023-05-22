#ifndef EDITOR_SELECTOR_H
#define EDITOR_SELECTOR_H

#include "data/mesh.h"
#include "editor/rasterizer.h"

#define DESELECT_COLOR 0x00204040

namespace oc {

class Selector : Rasterizer {
public:
    enum Mode { ARRAY, NORMAL, CIRCLE, RECT };

    void CompleteSelection(std::vector<Mesh>& mesh, bool inverse);
    void DecreaseSelection(std::vector<Mesh>& mesh);
    glm::vec3 GetCenter(std::vector<Mesh>& mesh);
    glm::vec4 GetCircle(std::vector<Mesh>& mesh, glm::mat4 world2screen, float x, float y);
    void IncreaseSelection(std::vector<Mesh>& mesh);
    void Init(int w, int h);
    glm::vec3 Raycast(std::vector<Mesh>& mesh, glm::vec3 from, glm::vec3 to);
    void SelectObject(std::vector<Mesh>& mesh, glm::mat4 world2screen, float x, float y, bool convexSearch = true);
    void SelectCircle(std::vector<Mesh>& mesh, glm::mat4 world2screen, float x, float y, float radius, bool invert);
    void SelectRect(std::vector<Mesh>& mesh, glm::mat4 world2screen, float x1, float y1, float x2, float y2, bool invert);
    void SelectTriangle(std::vector<Mesh>& mesh, glm::mat4 world2screen, float x, float y);
    glm::vec3 Transform(std::vector<Mesh>& mesh, glm::mat4 world2screen, float x, float y, bool culling = true);
    std::vector<glm::vec3> Transform(std::vector<Mesh>& mesh, glm::mat4 world2screen, std::vector<glm::vec2>& points, bool culling = true);
    bool ValidResult() { return hit; }
    static std::string VertexToKey(glm::vec3& vec);

private:
    void CreateGraph(std::vector<Mesh>& mesh);

    virtual void Process(unsigned long& index, int &x1, int &x2, int &y, glm::dvec3 &z1, glm::dvec3 &z2);

    std::map<std::string, std::map<std::pair<int, int>, bool> > connections;
    Mesh* currentMesh;
    double depth;
    bool hit;
    float circleRadius;
    int pointX, pointY, pointX2, pointY2;
    std::vector<float> depths;
    std::vector<glm::ivec2> points2D;
    std::vector<std::pair<int,int> > hits;
    unsigned int rangeColor;
    Mode mode;
    int selected;
};
}

#endif
