#include <glm/glm.hpp>
#include <map>
#include <string>
#include "data/mesh.h"

namespace oc {

    struct Triangle {
        glm::vec3 a, b, c;
    };

    Mesh::Mesh() : aabbUpdate(0), image(NULL), imageOwner(true), visible(true) {}

    void Mesh::Destroy() {
        if (image) {
            image->DelInstance();
            if (image->CanBeDeleted()) {
                delete image;
                image = 0;
            }
        }
    }

    void Mesh::GenerateFaceNormals() {
        normals.clear();
        glm::vec3 a, b, c, n;
        for (unsigned int i = 0; i < vertices.size(); i += 3) {
            a = vertices[i + 0];
            b = vertices[i + 1];
            c = vertices[i + 2];
            n = glm::normalize(glm::cross(a - b, a - c));
            normals.push_back(n);
            normals.push_back(n);
            normals.push_back(n);
        }
    }

    void Mesh::GenerateNormals() {
        //init variables
        std::string key;
        std::map<std::string, glm::vec3> vertexNormal;
        glm::vec3 a, b, c, n;

        //calculate vertex normals
        for (unsigned int i = 0; i < vertices.size(); i += 3) {
            a = vertices[i + 0];
            b = vertices[i + 1];
            c = vertices[i + 2];
            n = glm::normalize(glm::cross(a - b, a - c));
            key = Vector2key(a);
            if (vertexNormal.find(key) == vertexNormal.end())
                vertexNormal[key] = n;
            else
                vertexNormal[key] += n;
            key = Vector2key(b);
            if (vertexNormal.find(key) == vertexNormal.end())
                vertexNormal[key] = n;
            else
                vertexNormal[key] += n;
            key = Vector2key(c);
            if (vertexNormal.find(key) == vertexNormal.end())
                vertexNormal[key] = n;
            else
                vertexNormal[key] += n;
        }

        //apply vertex normals
        normals.clear();
        for (unsigned int i = 0; i < vertices.size(); i++) {
            a = vertices[i];
            key = Vector2key(a);
            n = glm::normalize(vertexNormal[key]);
            if (std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z))
                normals.push_back(glm::vec3(0));
            else
                normals.push_back(n);
        }
        std::map<std::string, glm::vec3>().swap(vertexNormal);
    }

    float Mesh::GetFloorLevel(glm::vec3 pos) {
        //detect collision with mesh boundary
        if (aabbUpdate != vertices.size()) {
            aabbMin = glm::vec3(INT_MAX, INT_MAX, INT_MAX);
            aabbMax = glm::vec3(INT_MIN, INT_MIN, INT_MIN);
            aabbUpdate = vertices.size();
            for (glm::vec3& v : vertices)
                UpdateAABB(v, aabbMin, aabbMax);
        }
        if (!IsInAABB(pos, aabbMin, aabbMax))
            return INT_MIN;

        //detect collision with triangle boundary
        glm::vec3 min, max;
        std::vector<Triangle> colliding;
        for (unsigned int i = 0; i < vertices.size(); i += 3) {
            min = vertices[i];
            max = vertices[i];
            UpdateAABB(vertices[i + 1], min, max);
            UpdateAABB(vertices[i + 2], min, max);
            if (IsInAABB(pos, min, max)) {
                Triangle t;
                t.a = vertices[i + 0];
                t.b = vertices[i + 1];
                t.c = vertices[i + 2];
                colliding.push_back(t);
            }
        }
        if (colliding.empty())
            return INT_MIN;

        //return some approximated value
        double output = INT_MAX;
        double value;
        for (Triangle t : colliding) {
            value = t.a.y + t.b.y + t.c.y;
            if (output > value)
                output = value;
        }
        return (float) (output / 3.0);
    }

    void Mesh::MirrorZ() {
        for (glm::vec3& v : vertices) {
            v.z *= -1.0f;
        }
        for (glm::vec3& v : normals) {
            v.z *= -1.0f;
        }
    }

    void Mesh::Normals2Color() {
        colors.resize(normals.size());
        for (unsigned int i = 0; i < normals.size(); i++)
        {
            int color = 0;
            color += glm::clamp((int)(normals[i].x * 127 + 128), 0, 255);
            color += glm::clamp((int)(normals[i].y * 127 + 128), 0, 255) << 8;
            color += glm::clamp((int)(normals[i].z * 127 + 128), 0, 255) << 16;
            colors[i] = color;
        }
    }

    void Mesh::Reindex() {
        Mesh temp;
        temp.vertices = vertices;
        temp.normals = normals;
        temp.colors = colors;
        temp.indices = indices;
        temp.uv = uv;

        vertices.clear();
        normals.clear();
        colors.clear();
        indices.clear();
        uv.clear();

        for (unsigned int& i : temp.indices) {
            if (!temp.vertices.empty())
                vertices.push_back(temp.vertices[i]);
            if (!temp.normals.empty())
                normals.push_back(temp.normals[i]);
            if (!temp.colors.empty())
                colors.push_back(temp.colors[i]);
            if (!temp.uv.empty())
                uv.push_back(temp.uv[i]);
        }
    }

    bool Mesh::IsInAABB(glm::vec3 &p, glm::vec3 &min, glm::vec3 &max) {
        return !((p.x < min.x) || (p.z < min.z) || (p.x > max.x) || (p.z > max.z));
    }

    void Mesh::UpdateAABB(glm::vec3& p, glm::vec3& min, glm::vec3& max) {
        if (min.x > p.x)
            min.x = p.x;
        if (min.z > p.z)
            min.z = p.z;
        if (max.x < p.x)
            max.x = p.x;
        if (max.z < p.z)
            max.z = p.z;
    }

    void Mesh::SwapYZ() {
        for (glm::vec3& v : vertices) {
            float y = v.y;
            float z = v.z;
            v.y = z;
            v.z = y;
        }
        for (glm::vec3& v : normals) {
            float y = v.y;
            float z = v.z;
            v.y = z;
            v.z = y;
        }
    }

    glm::vec3 Mesh::Key2Vector(std::string a) {
        glm::vec3 output;
        sscanf(a.c_str(), "%f,%f,%f", &output.x, &output.y, &output.z);
        return output;
    }

    std::string Mesh::Vector2key(glm::vec3 a) {
        char buffer[32];
        sprintf(buffer, "%.2f,%.2f,%.2f", a.x, a.y, a.z);
        return std::string(buffer);
    }
}
