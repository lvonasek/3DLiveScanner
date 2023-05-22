#ifndef POSTPROCESSOR_OPTIMIZER_H
#define POSTPROCESSOR_OPTIMIZER_H

#include <data/mesh.h>
#include <gl/opengl.h>
#include <map>

namespace oc {

    class Optimizer {
    public:
        void Process(std::string filename);
    private:
        glm::mat4 CalculateRotation(std::vector<Mesh>& data);
        void Finish(std::vector<Mesh>& data, std::string filename, glm::mat4 matrix);
    };
}
#endif
