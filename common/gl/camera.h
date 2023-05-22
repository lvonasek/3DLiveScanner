#ifndef GL_CAMERA_H
#define GL_CAMERA_H

#include "gl/opengl.h"

namespace oc {
    struct EulerAngles {
        double roll, pitch, yaw;
    };

    class GLCamera {
    public:
        static void DecomposeMatrix(const glm::mat4& matrix, glm::vec3* translation,
                                    glm::quat* rotation, glm::vec3* scale);
        static float Diff(glm::vec3& pa, glm::vec3& pb, glm::quat &a, glm::quat &b);

        EulerAngles GetRotation();
        glm::mat4 GetTransformation() const;
        glm::mat4 GetView() const;
        void SetRotation(EulerAngles euler);
        void SetTransformation(const glm::mat4& transform);
        void Translate(float x, float y, float z);

        glm::mat4 projection;
        glm::vec3 position;
        glm::quat rotation;
        glm::vec3 scale;
    };
}
#endif
