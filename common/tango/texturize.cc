#include <sstream>
#include "data/file3d.h"
#include "data/image.h"
#include "gl/camera.h"
#include "tango/texturize.h"
namespace oc {

    TangoTexturize::TangoTexturize() : poses(0), width(0), height(0),
                                       meshSimplification(10),
                                       scale(1),
                                       textureCount(4),
                                       textureResolution(2048),
                                       useDistortion(false) {}

    void TangoTexturize::Add(Image *image, double timestamp, Tango3DR_CameraCalibration *camera,
                             std::vector<glm::mat4> matrix, Dataset* dataset) {
        if (poses == 0)
            UpdatePoses(dataset);

        //save frame
        width = image->GetWidth();
        height = image->GetHeight();
        image->Write(dataset->GetFileName(poses, ".jpg"));

        //save transform
        dataset->WritePose(poses, matrix);

        //save timestamp
        FILE* file = fopen(dataset->GetFileName(poses, ".tms").c_str(), "w");
        fprintf(file, "%lf", timestamp);
        fclose(file);

        //save state
        poses++;
        dataset->WriteState(poses, width, height, camera->cx, camera->cy, camera->fx, camera->fy);
    }

    void TangoTexturize::ApplyDistortion(Tango3DR_CameraCalibration& camera, std::vector<float>& distortion) {
        if (useDistortion) {
            switch (distortion.size()) {
                case 3:
                    camera.calibration_type = TANGO_3DR_CALIBRATION_POLYNOMIAL_3_PARAMETERS;
                    camera.distortion[0] = distortion[0];
                    camera.distortion[1] = distortion[1];
                    camera.distortion[2] = distortion[2];
                    break;
                case 5:
                    camera.calibration_type = TANGO_3DR_CALIBRATION_POLYNOMIAL_5_PARAMETERS;
                    camera.distortion[0] = distortion[0];
                    camera.distortion[1] = distortion[1];
                    camera.distortion[2] = distortion[2];
                    camera.distortion[3] = distortion[3];
                    camera.distortion[4] = distortion[4];
                    break;
            }
        } else {
            camera.calibration_type = TANGO_3DR_CALIBRATION_POLYNOMIAL_3_PARAMETERS;
            camera.distortion[0] = 0;
            camera.distortion[1] = 0;
            camera.distortion[2] = 0;
        }
    }

    void TangoTexturize::ApplyFrames(Dataset* dataset) {
        UpdatePoses(dataset);
        std::vector<int> frames;
        for (unsigned int i = 0; i < poses; i++) {
            frames.push_back(i);
        }
        ApplyFrames(dataset, frames);
    }

    void TangoTexturize::ApplyFrames(Dataset *dataset, std::vector<int> frames) {
        UpdatePoses(dataset);
        Tango3DR_ImageBuffer image;
        image.width = (uint32_t) width;
        image.height = (uint32_t) height;
        image.stride = (uint32_t) width;
        image.format = TANGO_3DR_HAL_PIXEL_FORMAT_YCrCb_420_SP;
        image.data = new unsigned char[width * height * 3];

        for (int i = 0; i < frames.size(); i++) {
            std::ostringstream ss;
            ss << "IMAGE ";
            ss << i + 1;
            ss << "/";
            ss << frames.size();
            event = ss.str();

            image.timestamp = 0;
            Image::JPG2YUV(dataset->GetFileName(frames[i], ".jpg"), image.data, width, height);
            Tango3DR_Pose t3dr_image_pose = Extract3DRPose(dataset->ReadPose(frames[i])[COLOR_CAMERA]);
            t3dr_image_pose.translation[0] *= scale;
            t3dr_image_pose.translation[1] *= scale;
            t3dr_image_pose.translation[2] *= scale;
            Tango3DR_Status ret = Tango3DR_updateTexture(context, &image, &t3dr_image_pose);
            if (ret != TANGO_3DR_SUCCESS)
                exit(EXIT_SUCCESS);
        }
        delete[] image.data;
    }

    void TangoTexturize::Clear(Dataset* dataset) {
        poses = 0;
        dataset->WriteState(poses, width, height, camera.cx, camera.cy, camera.fx, camera.fy);
    }

    void TangoTexturize::DeleteLast(Dataset* dataset) {
        UpdatePoses(dataset);
        if (poses > 0) {
            poses--;
            dataset->WriteState(poses, width, height, camera.cx, camera.cy, camera.fx, camera.fy);
        }
    }

    Tango3DR_Pose TangoTexturize::Extract3DRPose(glm::mat4 matrix) {
        Tango3DR_Pose pose;
        glm::quat rotation = glm::quat_cast(matrix);
        pose.translation[0] = matrix[3][0];
        pose.translation[1] = matrix[3][1];
        pose.translation[2] = matrix[3][2];
        pose.orientation[0] = rotation[0];
        pose.orientation[1] = rotation[1];
        pose.orientation[2] = rotation[2];
        pose.orientation[3] = rotation[3];
        return pose;
    }

    int TangoTexturize::GetLatestIndex(Dataset* dataset) {
        UpdatePoses(dataset);
        return poses - 1;
    }

    double TangoTexturize::GetTimestamp(Dataset* dataset, int index) {
        double timestamp = 0;
        FILE* file = fopen(dataset->GetFileName(index, ".tms").c_str(), "r");
        fscanf(file, "%lf", &timestamp);
        fclose(file);
        return timestamp;
    }

    bool TangoTexturize::Init(std::string filename, bool verbose, bool ignoreConfig) {
        if (verbose)
            event = "MERGE";
        Tango3DR_Mesh mesh;
        Tango3DR_Status ret;
        ret = Tango3DR_Mesh_loadFromObj(filename.c_str(), &mesh);
        if (ret != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        //prevent crash on saving empty model
        if (mesh.num_faces == 0) {
            ret = Tango3DR_Mesh_destroy(&mesh);
            if (ret != TANGO_3DR_SUCCESS)
                exit(EXIT_SUCCESS);
            if (verbose)
                event = "";
            return false;
        }

        //create texturing context
        ScaleMesh(&mesh, scale);
        CreateContext(&mesh, verbose, ignoreConfig);
        ret = Tango3DR_Mesh_destroy(&mesh);
        if (ret != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);
        return true;
    }

    void TangoTexturize::Process(std::string filename, bool verbose, bool poisson) {
        //texturize mesh
        if (verbose)
            event = "UNWRAP";
        Tango3DR_Mesh mesh;
        Tango3DR_Status ret;
        ret = Tango3DR_getTexturedMesh(context, &mesh);
        if (ret != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        //save
        if (verbose)
            event = "CONVERT";
        ScaleMesh(&mesh, 1.0f / scale);
        ret = Tango3DR_Mesh_saveToObj(&mesh, filename.c_str());
        if (ret != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        //cleanup
        ret = Tango3DR_Mesh_destroy(&mesh);
        if (ret != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);
        ret = Tango3DR_TexturingContext_destroy(context);
        if (ret != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);
        context = 0;

        //add alpha channel to textures
        if (poisson) {
            std::vector<Mesh> data;
            File3d(filename, false).ReadModel(INT_MAX, data);
            for (Mesh& m : data) {
                if (m.image && m.imageOwner && m.vertices.size()) {
                    //get color statistics
                    std::map<int, int> colors;
                    for (unsigned int x = 0; x < m.image->GetWidth(); x++) {
                        for (unsigned int y = 0; y < m.image->GetHeight(); y++) {
                            int c = m.image->GetColor(x, y);
                            if (colors.find(c) == colors.end()) {
                                colors[c] = 0;
                            }
                            colors[c]++;
                        }
                    }

                    //get the most used color
                    int best = 0;
                    int count = 0;
                    for (std::map<int, int>::const_iterator it = colors.begin(); it != colors.end(); ++it) {
                        if (count < it->second) {
                            count = it->second;
                            best = it->first;
                        }
                    }

                    //replace color with transparency
                    glm::ivec4 transparent(128, 128, 128, 0);
                    for (unsigned int x = 0; x < m.image->GetWidth(); x++) {
                        for (unsigned int y = 0; y < m.image->GetHeight(); y++) {
                            int c = m.image->GetColor(x, y);
                            if (c == best) {
                                m.image->DrawPixel(x, y, transparent);
                            }
                        }
                    }

                    //save texture
                    m.image->Write(m.image->GetName());
                }
            }
        }
        event = "";
    }

    void TangoTexturize::CreateContext(Tango3DR_Mesh* mesh, bool verbose, bool ignoreConfig) {
        if (verbose)
            event = "SIMPLIFY";
        Tango3DR_Config textureConfig = Tango3DR_Config_create(TANGO_3DR_CONFIG_TEXTURING);
        if (Tango3DR_Config_setInt32(textureConfig, "texturing_backend", TANGO_3DR_CPU_TEXTURING) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
        if (ignoreConfig) {
            if (Tango3DR_Config_setInt32(textureConfig, "max_num_textures", 1) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
            if (Tango3DR_Config_setInt32(textureConfig, "mesh_simplification_factor", 5) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
            if (Tango3DR_Config_setInt32(textureConfig, "texture_size", 4096) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
        } else {
            if (Tango3DR_Config_setInt32(textureConfig, "max_num_textures", textureCount) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
            if (Tango3DR_Config_setInt32(textureConfig, "mesh_simplification_factor", meshSimplification) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
            if (Tango3DR_Config_setInt32(textureConfig, "texture_size", textureResolution) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
            if (Tango3DR_Config_setDouble(textureConfig, "min_resolution", 0.001 * scale) != TANGO_3DR_SUCCESS) exit(EXIT_SUCCESS);
        }
        context = Tango3DR_TexturingContext_create(textureConfig, mesh);
        if (context == nullptr) exit(EXIT_SUCCESS);
        Tango3DR_Config_destroy(textureConfig);
    }

    void TangoTexturize::UpdatePoses(Dataset* dataset) {
        dataset->ReadState(poses, width, height, cx, cy, fx, fy);

        camera.width = (uint32_t) width;
        camera.height = (uint32_t) height;
        camera.cx = cx;
        camera.cy = cy;
        camera.fx = fx;
        camera.fy = fy;

        std::vector<float> distortion = dataset->ReadDistortion();
        ApplyDistortion(camera, distortion);

        if (context) {
            Tango3DR_TexturingContext_setColorCalibration(context, &camera);
        }
    }

    void TangoTexturize::SetCalibration(Tango3DR_ReconstructionContext context, Dataset* dataset, int scale) {
        Tango3DR_CameraCalibration camera;

        std::vector<float> distortion = dataset->ReadDistortion();
        ApplyDistortion(camera, distortion);

        int t, w, h;
        dataset->ReadState(t, w, h, camera.cx, camera.cy, camera.fx, camera.fy);
        camera.width = w / scale;
        camera.height = h / scale;
        camera.cx /= (float)scale;
        camera.cy /= (float)scale;
        camera.fx /= (float)scale;
        camera.fy /= (float)scale;
        Tango3DR_ReconstructionContext_setColorCalibration(context, &camera);
    }

    void TangoTexturize::ScaleMesh(Tango3DR_Mesh *mesh, float s) {
        for (unsigned int i = 0; i < mesh->num_vertices; i++) {
            mesh->vertices[i][0] *= s;
            mesh->vertices[i][1] *= s;
            mesh->vertices[i][2] *= s;
        }
    }
}
