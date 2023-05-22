#include "gl/camera.h"

namespace oc {

    void GLCamera::DecomposeMatrix(const glm::mat4& matrix, glm::vec3* translation,
                                   glm::quat* rotation, glm::vec3* scale) {
        translation->x = matrix[3][0];
        translation->y = matrix[3][1];
        translation->z = matrix[3][2];
        *rotation = glm::quat_cast(matrix);
        scale->x = glm::length(glm::vec3(matrix[0][0], matrix[1][0], matrix[2][0]));
        scale->y = glm::length(glm::vec3(matrix[0][1], matrix[1][1], matrix[2][1]));
        scale->z = glm::length(glm::vec3(matrix[0][2], matrix[1][2], matrix[2][2]));
        if (glm::determinant(matrix) < 0.0)
            scale->x = -scale->x;
    }

    float GLCamera::Diff(glm::vec3& pa, glm::vec3& pb, glm::quat &a, glm::quat &b)
    {
        if (glm::abs(b.x) < 0.005)
            if (glm::abs(b.y) < 0.005)
                if (glm::abs(b.z) < 0.005)
                    if (glm::abs(b.w) - 1 < 0.005)
                        return 1;
        glm::vec3 diff = glm::eulerAngles(glm::inverse(a) * b);
        diff = glm::abs(diff);
        if (diff.x > M_PI)
            diff.x -= M_PI;
        if (diff.y > M_PI)
            diff.y -= M_PI;
        if (diff.z > M_PI)
            diff.z -= M_PI;
        float pos = glm::length(pa - pb) * 100.0f;
        float rot = glm::degrees(glm::max(glm::max(diff.x, diff.y), diff.z));
        return glm::max(pos, rot);
    }

    EulerAngles GLCamera::GetRotation() {
        glm::quat q = rotation;
        EulerAngles angles;

        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles.roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.pitch = std::asin(sinp);

        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles.yaw = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }

    glm::mat4 GLCamera::GetTransformation() const {
        glm::mat4 transform = glm::scale(glm::mat4_cast(rotation), scale);
        transform[3][0] = position.x;
        transform[3][1] = position.y;
        transform[3][2] = position.z;
        return transform;
    }

    glm::mat4 GLCamera::GetView() const {
        return glm::inverse(GetTransformation());
    }

    void GLCamera::SetRotation(EulerAngles euler) {
        double cy = cos(euler.yaw * 0.5);
        double sy = sin(euler.yaw * 0.5);
        double cp = cos(euler.pitch * 0.5);
        double sp = sin(euler.pitch * 0.5);
        double cr = cos(euler.roll * 0.5);
        double sr = sin(euler.roll * 0.5);

        rotation.w = cy * cp * cr + sy * sp * sr;
        rotation.x = cy * cp * sr - sy * sp * cr;
        rotation.y = sy * cp * sr + cy * sp * cr;
        rotation.z = sy * cp * cr - cy * sp * sr;
    }

    void GLCamera::SetTransformation(const glm::mat4& transform) {
        DecomposeMatrix(transform, &position, &rotation, &scale);
    }

    void GLCamera::Translate(float x, float y, float z) {
        position.x += x;
        position.y += y;
        position.z += z;
    }
}
