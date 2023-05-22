#include <data/file3d.h>
#include <postproc/optimizer.h>

namespace oc {
    void Optimizer::Process(std::string filename) {

        //load model
        std::vector<Mesh> data;
        File3d(filename, false).ReadModel(INT_MAX, data);

        //rotate the model
        glm::mat4 matrix = CalculateRotation(data);
        glm::vec3 scale;
        glm::quat rotation;
        glm::vec3 translation;
        glm::vec3 skew;
        glm::vec4 perspective;
        glm::decompose(matrix, scale, rotation, translation, skew, perspective);
        if (glm::abs(glm::length(scale) - glm::length(glm::vec3(1))) < 0.05f) {
            for (Mesh& mesh : data) {
                for (glm::vec3& v : mesh.vertices) {
                    glm::vec4 r = matrix * glm::vec4(v, 1.0f);
                    v.x = r.x;
                    v.y = r.z;
                    v.z =-r.y;
                }
                mesh.GenerateFaceNormals();
            }
        } else {
            LOGI("Matrix scale is not correct %f", glm::length(scale) - glm::length(glm::vec3(1)));
            matrix = glm::mat4(1);
        }

        Finish(data, filename, matrix);
    }

    glm::mat4 Optimizer::CalculateRotation(std::vector<Mesh>& data) {

        //get the biggest face normal
        float best = 0;
        glm::vec3 normal(0);
        for (Mesh& m : data) {
            m.GenerateFaceNormals();
            for (unsigned int i = 0; i < m.normals.size(); i += 3) {
                glm::vec3 a = m.vertices[i + 0];
                glm::vec3 b = m.vertices[i + 1];
                glm::vec3 c = m.vertices[i + 2];
                float ab = glm::distance(a, b);
                float ac = glm::distance(a, c);
                float bc = glm::distance(b, c);
                float perimeter = ab + ac + bc;

                if (best < perimeter) {
                    best = perimeter;
                    normal = -m.normals[i];
                }
            }
        }

        //generate matrix
        glm::vec3 xaxis = glm::normalize(glm::cross(glm::vec3(0, 1, 0), normal));
        glm::vec3 yaxis = glm::normalize(glm::cross(normal, xaxis));
        glm::mat4 output(1);
        output[0][0] = xaxis.x;
        output[0][1] = yaxis.x;
        output[0][2] = normal.x;
        output[1][0] = xaxis.y;
        output[1][1] = yaxis.y;
        output[1][2] = normal.y;
        output[2][0] = xaxis.z;
        output[2][1] = yaxis.z;
        output[2][2] = normal.z;
        return output;
    }

    void Optimizer::Finish(std::vector<Mesh>& data, std::string filename, glm::mat4 matrix) {
#ifndef ANDROID
        filename += ".obj";
#endif

        //get dimensions
        glm::vec3 min(INT_MAX);
        glm::vec3 max(INT_MIN);
        for (Mesh& mesh : data) {
            for (glm::vec3& v : mesh.vertices) {
                if (min.x > v.x) min.x = v.x;
                if (min.y > v.y) min.y = v.y;
                if (min.z > v.z) min.z = v.z;
                if (max.x < v.x) max.x = v.x;
                if (max.y < v.y) max.y = v.y;
                if (max.z < v.z) max.z = v.z;
            }
        }

        //recenter
        glm::vec3 center = (min + max) / 2.0f;
        center.y = min.y;
        for (Mesh& mesh : data) {
            for (glm::vec3& v : mesh.vertices) {
                v -= center;

            }
        }

        //parse model
        float w;
        glm::vec3 p, v;
        char buffer[1024];
        std::vector<std::string> model;
        FILE* file = fopen(filename.c_str(), "r");
        while (true) {
            if (!fgets(buffer, 1024, file))
                break;
            std::string sbuf = buffer;
            while(!sbuf.empty() && isspace(sbuf[0])) {
                sbuf = sbuf.substr(1);
            }
            if ((sbuf[0] == 'v') && (sbuf[1] != 't')) {
                if (sbuf[1] == ' ') {
                    sscanf(sbuf.c_str(), "v %f %f %f", &p.x, &p.y, &p.z);
                    w = 1;
                } else {
                    sscanf(sbuf.c_str(), "vn %f %f %f", &p.x, &p.y, &p.z);
                    w = 0;
                }
                glm::vec4 r = matrix * glm::vec4(p, w);
                v.x = r.x;
                v.y = r.z;
                v.z =-r.y;
                v -= center * w;
                if (sbuf[1] == ' ') {
                    sprintf(buffer, "v %f %f %f\n", v.x, v.y, v.z);
                } else {
                    sprintf(buffer, "vn %f %f %f\n", v.x, v.y, v.z);
                }
                model.emplace_back(buffer);
            } else {
                model.push_back(sbuf);
            }
        }
        fclose(file);

        //save model
        file = fopen(filename.c_str(), "w");
        for (std::string& s : model) {
            fprintf(file, "%s", s.c_str());
        }
        fclose(file);
    }
}
