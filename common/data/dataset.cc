#include "data/dataset.h"
#include <sstream>

namespace oc {
    Dataset::Dataset(std::string path) {
        dataset = path;
    }

    std::string Dataset::GetFileName(int index, std::string extension) {
        std::ostringstream ss;
        ss << index;
        std::string number = ss.str();
        while(number.size() < 8)
            number = "0" + number;
        return dataset + "/" + number + extension;
    }

    std::vector<float> Dataset::ReadDistortion() {
        std::vector<float> output;

        int size = 0;
        FILE* file = fopen((dataset + "/distortion.txt").c_str(), "r");
        if (file) {
            fscanf(file, "%d\n", &size);
            for (int i = 0; i < size; i++) {
                float value = 0;
                fscanf(file, "%f\n", &value);
                output.push_back(value);
            }
            fclose(file);
        } else {
            for (int i = 0; i < 3; i++) {
                output.push_back(0);
            }
        }

        return output;
    }

    std::vector<glm::mat4> Dataset::ReadPose(int index) {
        std::vector<glm::mat4> output;
        FILE* file = fopen(GetFileName(index, ".mat").c_str(), "r");
        for (int i = 0; i < MAX_CAMERA; i++) {
            glm::mat4 mat(0);
            for (int j = 0; j < 4; j++) {
                if (feof(file)) {
                    break;
                }
                fscanf(file, "%f %f %f %f\n", &mat[j][0], &mat[j][1], &mat[j][2], &mat[j][3]);
            }
            output.push_back(mat);
        }
        fclose(file);
        return output;
    }

    void Dataset::ReadState(int &count, int &width, int &height, double& cx, double& cy, double& fx, double& fy) {
        FILE* file = fopen((dataset + "/state.txt").c_str(), "r");
        if (file) {
            fscanf(file, "%d %d %d %lf %lf %lf %lf\n", &count, &width, &height, &cx, &cy, &fx, &fy);
            fclose(file);
        }
    }

    float Dataset::ReadYaw() {
        float yaw = -90;
        FILE* file = fopen((dataset + "/rotation.txt").c_str(), "r");
        if (file) {
            fscanf(file, "%f\n", &yaw);
            fclose(file);
        }
        return yaw;
    }

    void Dataset::WriteDistortion(std::vector<float> data) {
        FILE* file = fopen((dataset + "/distortion.txt").c_str(), "w");
        fprintf(file, "%d\n", (int)data.size());
        for (int i = 0; i < data.size(); i++) {
            fprintf(file, "%f\n", data[i]);
        }
        fclose(file);
    }

    void Dataset::WritePose(int index, std::vector<glm::mat4> pose) {
        FILE* file = fopen(GetFileName(index, ".mat").c_str(), "w");
        for (int k = 0; k < MAX_CAMERA; k++)
            for (int i = 0; i < 4; i++)
                fprintf(file, "%f %f %f %f\n", pose[k][i][0], pose[k][i][1],
                                               pose[k][i][2], pose[k][i][3]);
        fclose(file);
    }

    void Dataset::WriteState(int count, int width, int height, double cx, double cy, double fx, double fy) {
        FILE* file = fopen((dataset + "/state.txt").c_str(), "w");
        fprintf(file, "%d %d %d %lf %lf %lf %lf\n", count, width, height, cx, cy, fx, fy);
        fclose(file);
    }

    void Dataset::WriteYaw(float yaw) {
        FILE* file = fopen((dataset + "/rotation.txt").c_str(), "w");
        fprintf(file, "%f\n", yaw);
        fclose(file);
    }

    Tango3DR_PointCloud Dataset::ReadPointCloud(int index) {

        Tango3DR_PointCloud t3dr_depth;
        FILE* file = fopen(GetFileName(index, ".pcl").c_str(), "rb");
        fread(&t3dr_depth.num_points, sizeof(int), 1, file);

#ifdef ANDROID
        Tango3DR_PointCloud_init(t3dr_depth.num_points, &t3dr_depth);
#else
        t3dr_depth.points = new Tango3DR_Vector4[t3dr_depth.num_points];
#endif
        fread(t3dr_depth.points, sizeof(Tango3DR_Vector4), t3dr_depth.num_points, file);
        fclose(file);
        return t3dr_depth;
    }

    void Dataset::WritePointCloud(int index, Tango3DR_PointCloud t3dr_depth) {
        FILE* file = fopen(GetFileName(index, ".pcl").c_str(), "wb");
        fwrite(&t3dr_depth.num_points, sizeof(uint32_t), 1, file);
        fwrite(t3dr_depth.points, sizeof(Tango3DR_Vector4), t3dr_depth.num_points, file);
        fclose(file);
    }

    std::vector<std::pair<GridIndex, Tango3DR_Mesh *> > Dataset::ReadPreview(int index, bool empty) {
        FILE* file = fopen(GetFileName(index, ".bin").c_str(), "rb");
        std::vector<std::pair<GridIndex, Tango3DR_Mesh *>> output;
        int count = 0;
        fread(&count, sizeof(int), 1, file);
        for (int i = 0; i < count; i++) {
            std::pair<GridIndex, Tango3DR_Mesh *> p;
            fread(p.first.indices, sizeof(Tango3DR_GridIndex), 1, file);
            output.push_back(p);
        }
        if (!empty) {
            for (int i = 0; i < count; i++) {
                uint32_t num_faces;
                uint32_t num_vertices;
                fread(&num_faces, sizeof(uint32_t), 1, file);
                fread(&num_vertices, sizeof(uint32_t), 1, file);

                Tango3DR_Mesh* mesh = new Tango3DR_Mesh();
                mesh->num_faces = mesh->max_num_faces = num_faces;
                mesh->num_vertices = mesh->max_num_vertices = num_vertices;
                mesh->vertices = new Tango3DR_Vector3[num_vertices];
                mesh->normals = new Tango3DR_Vector3[num_vertices];
                mesh->colors = new Tango3DR_Color[num_vertices];
                mesh->faces = new Tango3DR_Face[num_faces];

                fread(mesh->vertices, sizeof(Tango3DR_Vector3), num_vertices, file);
                fread(mesh->normals, sizeof(Tango3DR_Vector3), num_vertices, file);
                fread(mesh->colors, sizeof(Tango3DR_Color), num_vertices, file);
                fread(mesh->faces, sizeof(Tango3DR_Face), num_faces, file);
                output[i].second = mesh;
            }
        }
        return output;
    }

    void Dataset::WritePreview(int index, std::vector<std::pair<GridIndex, Tango3DR_Mesh *>> preview) {
        FILE* file = fopen(GetFileName(index, ".bin").c_str(), "wb");
        int count = preview.size();
        fwrite(&count, sizeof(int), 1, file);
        for (std::pair<GridIndex, Tango3DR_Mesh *>& p : preview) {
            fwrite(p.first.indices, sizeof(Tango3DR_GridIndex), 1, file);
        }
        for (std::pair<GridIndex, Tango3DR_Mesh *>& p : preview) {
            fwrite(&p.second->num_faces, sizeof(uint32_t), 1, file);
            fwrite(&p.second->num_vertices, sizeof(uint32_t), 1, file);
            fwrite(p.second->vertices, sizeof(Tango3DR_Vector3), p.second->num_vertices, file);
            fwrite(p.second->normals, sizeof(Tango3DR_Vector3), p.second->num_vertices, file);
            fwrite(p.second->colors, sizeof(Tango3DR_Color), p.second->num_vertices, file);
            fwrite(p.second->faces, sizeof(Tango3DR_Face), p.second->num_faces, file);
        }
        fclose(file);
    }
}
