#ifndef EDITOR_EFFECTOR_H
#define EDITOR_EFFECTOR_H

#include <map>
#include "data/mesh.h"
#include "rasterizer.h"

namespace oc {

class Effector : Rasterizer {
public:
    enum Effect{ CONTRAST, GAMMA, SATURATION, TONE, RESET, CLONE, DELETE, MOVE, ROTATE, SCALE };

    void ApplyEffect(std::vector<Mesh>& mesh, Effect e, float value, int axis);
    void PreviewEffect(std::string& vs, std::string& fs, Effect e, int axis);
    void SetCenter(glm::vec3 value) { center = value; }
    void SetPitch(float value) { pitch = value; }

private:
    void ApplyColorEffect(std::vector<Mesh>& mesh, Effect effect, float value);
    void ApplyGeometryEffect(std::vector<Mesh>& mesh, Effect effect, float value, int axis);
    void PreviewColorEffect(std::string& fs, Effect effect);
    void PreviewGeometryEffect(std::string& vs, Effect effect, int axis);
    virtual void Process(unsigned long& index, int &x1, int &x2, int &y, glm::dvec3 &z1, glm::dvec3 &z2);
    void RotateVertex(glm::vec3& v, glm::vec3& center, int& axis, float& s, float& c);

    glm::vec3 center;
    bool* mask;
    float pitch;
    std::map<long, bool*> texture2mask;
};
}

#endif
