#ifndef POSTPROCESSOR_POISSON_H
#define POSTPROCESSOR_POISSON_H

#include <gl/opengl.h>
#include <map>
#include <string>

namespace oc {

    class Poisson {
    public:
        void Process(std::string filename);
    };
}
#endif
