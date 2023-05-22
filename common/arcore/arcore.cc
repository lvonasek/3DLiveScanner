#include <arcore/arcore.h>
#include <mutex>
#include "service.h"

namespace oc {

    ARCore::ARCore(void *env, void *context, bool faceMode, bool depthCamera) {
        useDepthRaw = false;
#ifndef ARCORE_BACKPORT
        if (env && context) {
            if (faceMode) {
                ArSessionFeature features[2] = {AR_SESSION_FEATURE_FRONT_CAMERA, AR_SESSION_FEATURE_END_OF_LIST};
                ArSession_createWithFeatures(env, context, features, &ar_session_);
            } else if (!depthCamera) {
                ArSessionFeature features[2] = {AR_SESSION_FEATURE_DO_NOT_USE_ACTIVE_DEPTH_SENSOR, AR_SESSION_FEATURE_END_OF_LIST};
                ArSession_createWithFeatures(env, context, features, &ar_session_);
            } else {
                ArSession_create(env, context, &ar_session_);
            }

            ArConfig *ar_config = nullptr;
            ArConfig_create(ar_session_, &ar_config);
            ArConfig_setFocusMode(ar_session_, ar_config, AR_FOCUS_MODE_FIXED);
            ArConfig_setPlaneFindingMode(ar_session_, ar_config, AR_PLANE_FINDING_MODE_DISABLED);
            if (faceMode)
                ArConfig_setAugmentedFaceMode(ar_session_, ar_config, AR_AUGMENTED_FACE_MODE_MESH3D);
            else {
                useDepthRaw = true;
                if (depthCamera) {
                    ArConfig_setDepthMode(ar_session_, ar_config, AR_DEPTH_MODE_RAW_DEPTH_ONLY);
                } else {
                    ArConfig_setDepthMode(ar_session_, ar_config, AR_DEPTH_MODE_ALWAYS_ENABLED);
                }
            }
            ArConfig_setUpdateMode(ar_session_, ar_config, AR_UPDATE_MODE_BLOCKING);
            ArSession_configure(ar_session_, ar_config);
            ArConfig_destroy(ar_config);
            ArFrame_create(ar_session_, &ar_frame_);

            int out_is_supported = 0;
            ArSession_isDepthModeSupported(ar_session_, AR_DEPTH_MODE_ALWAYS_ENABLED, &out_is_supported);
            useDepth = (out_is_supported != 0) || depthCamera;
        } else
#endif
        {
            useDepth = depthCamera;
            useDepthRaw = depthCamera;
        }

        ar_zero_.second = 0;
        has_coordinate_system_ = false;
        has_depth_sensor = depthCamera;
        lastDepthTimestamp = 0;
        offset = 0;
        resolution = 0;
        face_mode_ = faceMode;
    }

    ARCore::~ARCore() {
        ArSession_destroy(ar_session_);
        ArFrame_destroy(ar_frame_);
    }

    void ARCore::Clear(bool detach) {
        if (detach) {
            for (auto& anchor : ar_anchor_list) {
                ArAnchor_detach(ar_session_, ar_anchor_list[anchor.first]);
                ArAnchor_release(ar_anchor_list[anchor.first]);
            }
            ArAnchor_detach(ar_session_, ar_zero_.second);
            ArAnchor_release(ar_zero_.second);
        }
        ar_anchor_list.clear();
        ar_zero_.second = 0;
        has_coordinate_system_ = false;
    }

    void ARCore::OnPause() {
        ArSession_pause(ar_session_);
    }

    void ARCore::OnResume() {
        ArSession_resume(ar_session_);
        camera.InitializeGlContent();
        texture_initialized_ = false;
    }

    void ARCore::OnDisplayGeometryChanged(int display_rotation, int width, int height) {
        viewportWidth = width;
        viewportHeight = height;
        ArSession_setDisplayGeometry(ar_session_, display_rotation, width, height);
    }

    void ARCore::Configure(void *session, void *frame) {
        ar_session_ = static_cast<ArSession *>(session);
        ar_frame_ = static_cast<ArFrame *>(frame);

        int out_is_supported = 0;
        ArSession_isDepthModeSupported(ar_session_, AR_DEPTH_MODE_AUTOMATIC, &out_is_supported);
        useDepth = (out_is_supported != 0) || useDepth;
    }

    float ARCore::CountFrameError() {
        int size = 0;
        float error = 10000;
        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        ArPose *ar_pose;
        ArPose_create(ar_session_, data, &ar_pose);
        glm::mat4 matrix = projection_mat * view_mat;

        ArHitResult* hit = 0;
        ArHitResultList* hits = 0;
        ArHitResult_create(ar_session_, &hit);
        ArHitResultList_create(ar_session_, &hits);
        for (glm::vec3& v : GetActiveAnchors()) {
            glm::vec4 point = matrix * glm::vec4(v, 1.0);
            point /= fabs(point.z * point.w);
            point = 0.5f * point + 0.5f;

            ArFrame_hitTest(ar_session_, ar_frame_, viewportWidth * point.x, viewportHeight * point.y, hits);
            ArHitResultList_getSize(ar_session_, hits, &size);
            for (int i = 0; i < size; i++) {
                ArTrackable *trackable = 0;
                ArHitResultList_getItem(ar_session_, hits, i, hit);
                ArHitResult_acquireTrackable(ar_session_, hit, &trackable);
                ArPoint_getPose(ar_session_, ArAsPoint(trackable), ar_pose);
                ArPose_getPoseRaw(ar_session_, ar_pose, data);
                glm::vec3 position = glm::vec3(data[4], data[5], data[6]);
                float dst = glm::distance(v, position);
                if (dst > 0) {
                    error = glm::min(error, dst);
                }
                ArTrackable_release(trackable);
            }
        }

        ArHitResultList_destroy(hits);
        ArHitResult_destroy(hit);
        ArPose_destroy(ar_pose);
        return error;
    }

    bool ARCore::Process(bool update) {
        if (update) {
            if (!texture_initialized_) {
                ArSession_setCameraTextureName(ar_session_, camera.GetTextureName());
                texture_initialized_ = true;
            }
            if (ArSession_update(ar_session_, ar_frame_) != AR_SUCCESS)
                return false;
        }

        ArCamera *ar_camera;
        ArFrame_acquireCamera(ar_session_, ar_frame_, &ar_camera);
        ArCamera_getViewMatrix(ar_session_, ar_camera, glm::value_ptr(view_mat));
        ArCamera_getProjectionMatrix(ar_session_, ar_camera, 0.001f, 100.f,
                                       glm::value_ptr(projection_mat));
        view_mat = view_mat * GetZeroTransform();
        ArCamera_release(ar_camera);

        if (face_mode_) {
            return true;
        } else if (ar_anchor_list.empty()) {
            return UpdateAnchor();
        } else {
            return true;
        }
    }

    void ARCore::RenderCamera(ARCoreCamera::Effect effect, int scale) {
        if (effect >= ARCoreCamera::DEPTH) {
            Image* img = 0;
            if (effect == ARCoreCamera::NIGHTVISION)
                img = GetDepthMap(true, false, scale);
            else if (effect == ARCoreCamera::DEPTH_INV)
                img = GetDepthMap(false, false, scale);
            else
                img = GetDepthMap(false, true, scale);
            if (img) {
                GLuint texture = GLSL::Image2GLTexture(img);
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, texture);
                camera.GetShader()->Bind();
                camera.GetShader()->UniformInt("depth", 0);
                camera.DrawARCore(ar_session_, ar_frame_, effect, viewportWidth, viewportHeight);
                glDeleteTextures(1, &texture);
                delete img;
            }
        } else {
            camera.DrawARCore(ar_session_, ar_frame_, effect, viewportWidth, viewportHeight);
        }
    }

    std::vector<glm::vec3> ARCore::GetActiveAnchors() {
        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        ArPose *ar_pose;
        ArPose_create(ar_session_, data, &ar_pose);
        std::vector<glm::vec3> output;

        glm::mat4 zero = glm::inverse(GetZeroTransform());
        for (auto& anchor : ar_anchor_list) {
            ArTrackingState state = AR_TRACKING_STATE_STOPPED;
            ArAnchor_getTrackingState(ar_session_, anchor.second, &state);
            if (state == AR_TRACKING_STATE_TRACKING) {
                ArAnchor_getPose(ar_session_, anchor.second, ar_pose);
                ArPose_getPoseRaw(ar_session_, ar_pose, data);
                glm::vec3 v = glm::vec3(data[4], data[5], data[6]);
                glm::vec4 p = zero * glm::vec4(v, 1.0f);
                v = p / fabs(p.w);
                output.push_back(v);
            }
        }
        ArPose_destroy(ar_pose);
        return output;
    }

    std::vector<float> ARCore::GetDistortion() {
        std::vector<float> output;
        for (int i = 0; i < 3; i++) {
            output.push_back(0);
        }
        return output;
    }

    glm::vec3 ARCore::HitTest(int x, int y) {
        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        ArPose *ar_pose;
        ArPose_create(ar_session_, data, &ar_pose);

        int size = 0;
        ArHitResult* hit = 0;
        ArHitResultList* hits = 0;
        ArHitResult_create(ar_session_, &hit);
        ArHitResultList_create(ar_session_, &hits);
        ArFrame_hitTest(ar_session_, ar_frame_, x, y, hits);
        ArHitResultList_getSize(ar_session_, hits, &size);
        if (size > 0) {
            ArTrackable* trackable = 0;
            ArHitResultList_getItem(ar_session_, hits, 0, hit);
            ArHitResult_acquireTrackable(ar_session_, hit, &trackable);
            ArPoint_getPose(ar_session_, ArAsPoint(trackable), ar_pose);
            ArPose_getPoseRaw(ar_session_, ar_pose, data);
            ArTrackable_release(trackable);
            ArHitResultList_destroy(hits);
            ArHitResult_destroy(hit);
            ArPose_destroy(ar_pose);
            return glm::vec3(data[4], data[5], data[6]);
        }
        ArHitResultList_destroy(hits);
        ArHitResult_destroy(hit);
        ArPose_destroy(ar_pose);
        return glm::vec3(INT_MAX);
    }

    Image* ARCore::GetDepthMap(bool confidence, bool increasing, int s) {

        if (useDepth) {
            ArImage* image = 0;
            int result = 0;
            if (useDepthRaw) {
                result = ArFrame_acquireRawDepthImage16Bits(ar_session_, ar_frame_, &image);
            } else {
                result = ArFrame_acquireDepthImage16Bits(ar_session_, ar_frame_, &image);
            }

            if (result == AR_SUCCESS) {

                uint8_t* confidenceData;
                ArImage* confidenceImage = 0;
                bool hasConfidence = false;
                if (useDepthRaw) {
                    result = ArFrame_acquireRawDepthConfidenceImage(ar_session_, ar_frame_, &confidenceImage);;
                    if (result == AR_SUCCESS) {
                        int32_t dataLength;
                        ArImage_getPlaneData(ar_session_, confidenceImage, 0, (const uint8_t**)&confidenceData, &dataLength);
                        hasConfidence = true;
                    }
                }

                //get depth data
                uint16_t* imgData;
                int32_t dataLength;
                int32_t depthWidth, depthHeight, stride;
                ArImage_getPlaneRowStride(ar_session_, image, 0, &stride);
                ArImage_getPlaneData(ar_session_, image, 0, (const uint8_t**)&imgData, &dataLength);
                ArImage_getWidth(ar_session_, image, &depthWidth);
                ArImage_getHeight(ar_session_, image, &depthHeight);

                if (depthWidth > 240) s *= depthWidth / 240;
                depthWidth /= s;
                depthHeight /= s;
                Image* output = new Image(depthWidth, depthHeight);
                for (int y = 0; y < depthHeight; y++) {
                    for (int x = 0; x < depthWidth; x++) {
                        int depth = static_cast<int>((imgData[s * y * stride / 2 + s * x] & 0xFFFF) * 0.001 * 255);
                        if (!increasing && depth > 0) depth = 768 - depth;
                        output->GetData()[(y * depthWidth + x) * 4 + 0] = camera.Convert(depth, 0);
                        output->GetData()[(y * depthWidth + x) * 4 + 1] = camera.Convert(depth, 1);
                        output->GetData()[(y * depthWidth + x) * 4 + 2] = camera.Convert(depth, 2);
                        output->GetData()[(y * depthWidth + x) * 4 + 3] = 255;
                        if (confidence && hasConfidence) {
                            output->GetData()[(y * depthWidth + x) * 4 + 3] = 128 + confidenceData[s * y * depthWidth + s * x] / 2;
                        }
                    }
                }

                if (hasConfidence) {
                    ArImage_release(confidenceImage);
                }
                ArImage_release(image);
                return output;
            }
        }
        return 0;
    }

    glm::mat4 ARCore::GetMatrix(ArPose* ar_pose) {
        float* matrix = new float[16];
        ArPose_getMatrix(ar_session_, ar_pose, matrix);

        glm::mat4 output(1);
        output[0][0] = matrix[0];
        output[0][1] = matrix[1];
        output[0][2] = matrix[2];
        output[0][3] = matrix[3];
        output[1][0] = matrix[4];
        output[1][1] = matrix[5];
        output[1][2] = matrix[6];
        output[1][3] = matrix[7];
        output[2][0] = matrix[8];
        output[2][1] = matrix[9];
        output[2][2] = matrix[10];
        output[2][3] = matrix[11];
        output[3][0] = matrix[12];
        output[3][1] = matrix[13];
        output[3][2] = matrix[14];
        output[3][3] = matrix[15];
        delete[] matrix;
        return output;
    }

    glm::mat4 ARCore::GetZeroTransform() {
#ifndef ARCORE_BACKPORT
        if (ar_zero_.second) {
            float data[7] = {0, 0, 0, 1, 0, 0, 0};

            ArPose *ar_pose;
            ArPose_create(ar_session_, data, &ar_pose);
            ArAnchor_getPose(ar_session_, ar_zero_.second, ar_pose);
            glm::mat4 matrix = GetMatrix(ar_pose);
            ArPose_destroy(ar_pose);

            return matrix * glm::inverse(ar_zero_.first.matrix);
        }
#endif
        return glm::mat4(1);
    }

    glm::vec4 ARCore::ToPoint(glm::dmat4& screen2world, double& len,
                      int32_t& depthWidth, int32_t& depthHeight, int& x, int& y, double& depth) {

        //convert sensor coordinates to screen coordinates
        glm::dvec2 T = camera.Transform(x, y, depthWidth, depthHeight);

        //create a ray from screen space to world space
        glm::dvec4 point0 = screen2world * glm::vec4(T, 0, 1);
        point0 /= glm::abs(point0.w);
        point0.w = 1;
        glm::dvec4 point1 = screen2world * glm::vec4(T, 1, 1);
        point1 /= glm::abs(point1.w);
        point1.w = 1;

        //get a point on ray that match the depth
        return point0 + (point1 - point0) / len * depth;
    }

    bool ARCore::UpdateAnchor() {
        if (!ar_anchor_list.empty() && GetActiveAnchors().empty()) {
            return false;
        }

        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        ArPose *ar_pose;
        ArPose_create(ar_session_, data, &ar_pose);

        bool valid = false;
        ArAnchor* ar_anchor_ = 0;
        int size = 0;
        ArHitResult* hit = 0;
        ArHitResultList* hits = 0;
        ArHitResult_create(ar_session_, &hit);
        ArHitResultList_create(ar_session_, &hits);
        for (float x = 0.25f; x <= 0.75f; x += 0.5f) {
            for (float y = 0.25f; y <= 0.75f; y += 0.5f) {
                ArFrame_hitTest(ar_session_, ar_frame_, viewportWidth * x, viewportHeight * y, hits);
                ArHitResultList_getSize(ar_session_, hits, &size);
                for (int i = 0; i < size; i++) {
                    ArTrackable* trackable = 0;
                    ArHitResultList_getItem(ar_session_, hits, i, hit);
                    ArHitResult_acquireTrackable(ar_session_, hit, &trackable);
                    ArTrackableType type = AR_TRACKABLE_NOT_VALID;
                    ArTrackable_getType(ar_session_, trackable, &type);
                    if (type == AR_TRACKABLE_POINT) {

                        ArPoint_getPose(ar_session_, ArAsPoint(trackable), ar_pose);
                        ArPose_getPoseRaw(ar_session_, ar_pose, data);
                        glm::vec3 position(data[4], data[5], data[6]);

                        id3d pos;
                        pos.matrix = GetMatrix(ar_pose);
                        float density = ANCHOR_DENSITY_BASE;
                        for (pos.layer = 0; pos.layer < ANCHOR_LAYERS; pos.layer++) {
                            pos.x = static_cast<int>(position.x / density);
                            pos.y = static_cast<int>(position.y / density);
                            pos.z = static_cast<int>(position.z / density);
                            valid = true;
                            if (ar_anchor_list.find(pos) == ar_anchor_list.end()) {
                                ArStatus ret = ArTrackable_acquireNewAnchor(ar_session_, trackable, ar_pose, &ar_anchor_);
                                if (ret == AR_SUCCESS) {
                                    if (ar_zero_.second == 0) {
                                        ar_zero_.first = pos;
                                        ar_zero_.second = ar_anchor_;
                                        break;
                                    }

                                    ar_anchor_list[pos] = ar_anchor_;
                                    while (true) {
                                        int count = 0;
                                        id3d far = pos;
                                        for (auto& anchor : ar_anchor_list) {
                                            if (anchor.first.layer == pos.layer) {
                                                if (Diff(anchor.first, pos) > Diff(far, pos)) {
                                                    far = anchor.first;
                                                }
                                                count++;
                                            }
                                        }
                                        if (count > ANCHOR_CACHE) {
                                            ArAnchor_detach(ar_session_, ar_anchor_list[far]);
                                            ArAnchor_release(ar_anchor_list[far]);
                                            ar_anchor_list.erase(far);
                                        } else {
                                            break;
                                        }
                                    }
                                }
                            }
                            density *= ANCHOR_DENSITY_SCALE;
                        }
                    }
                    ArTrackable_release(trackable);
                }
            }
        }
        ArHitResultList_destroy(hits);
        ArHitResult_destroy(hit);
        ArPose_destroy(ar_pose);

        if (!valid && !GetActiveAnchors().empty()) {
            valid = true;
        }
        return valid;
    }

    void ARCore::UpdateFace(glm::mat4 matrix) {
#ifndef ARCORE_BACKPORT
        face_mesh.vertices.clear();
        face_mesh.normals.clear();
        face_mesh.uv.clear();
        face_mesh.indices.clear();
        points.clear();
        int32_t size = 0;
        ArTrackableList* faces = 0;
        ArTrackableList_create(ar_session_, &faces);
        ArSession_getAllTrackables(ar_session_, AR_TRACKABLE_FACE, faces);
        ArTrackableList_getSize(ar_session_, faces, &size);
        for (int32_t i = 0; i < size; i++) {
            int32_t count = 0;
            const float* vertices = 0;
            const float* normals = 0;
            ArTrackable* face = 0;
            ArTrackableList_acquireItem(ar_session_, faces, i, &face);
            ArAugmentedFace_getMeshVertices(ar_session_, ArAsFace(face), &vertices, &count);
            ArAugmentedFace_getMeshNormals(ar_session_, ArAsFace(face), &normals, &count);

            float data[7] = {0, 0, 0, 1, 0, 0, 0};
            ArPose *ar_pose;
            ArPose_create(ar_session_, data, &ar_pose);
            ArAugmentedFace_getCenterPose(ar_session_, ArAsFace(face), ar_pose);
            ArPose_getPoseRaw(ar_session_, ar_pose, data);

            GLCamera pose;
            pose.position = glm::vec3(data[4], data[5], data[6]);
            pose.rotation = glm::quat(data[3], data[0], data[1], data[2]);
            pose.scale = glm::vec3(1);
            glm::mat4 transform = pose.GetTransformation();
            for (int j = 0; j < count; j++) {
                glm::vec4 point = glm::vec4(vertices[j * 3 + 0],
                                            vertices[j * 3 + 1],
                                            vertices[j * 3 + 2],
                                            1.0f);
                point = transform * point;
                point /= fabs(point.w);
                point.w = 1.0f;
                face_mesh.vertices.push_back(point);
                face_mesh.normals.push_back(glm::vec3(normals[j * 3 + 0],
                                                      normals[j * 3 + 1],
                                                      normals[j * 3 + 2]));

                point = matrix * point;
                point /= fabs(point.w);
                face_mesh.uv.push_back(0.5f * glm::vec2(point.x, point.y) + 0.5f);
            }

            const uint16_t* indices = 0;
            int32_t triangles = 0;
            ArAugmentedFace_getMeshTriangleIndices(ar_session_, ArAsFace(face), &indices, &triangles);
            for (int j = 0; j < triangles; j++) {
                if (face_not_all) {
                    bool ok = true;
                    for (int l = j * 3 + 0; l < j * 3 + 3; l++)
                    {
                        if ((indices[l] == 13) || (indices[l] == 14))
                            ok = false;
                        if ((indices[l] == 78) || (indices[l] == 95))
                            ok = false;
                        if ((indices[l] >= 80) && (indices[l] <= 82))
                            ok = false;
                        if ((indices[l] == 87) || (indices[l] == 88))
                            ok = false;
                        if ((indices[l] == 178) || (indices[l] == 191))
                            ok = false;
                        if ((indices[l] == 308) || (indices[l] == 324))
                            ok = false;
                        if ((indices[l] >= 310) && (indices[l] <= 312))
                            ok = false;
                        if ((indices[l] == 317) || (indices[l] == 318))
                            ok = false;
                        if ((indices[l] == 402) || (indices[l] == 415))
                            ok = false;
                    }
                    if (!ok)
                        continue;
                }
                face_mesh.indices.push_back(indices[j * 3 + 0]);
                face_mesh.indices.push_back(indices[j * 3 + 1]);
                face_mesh.indices.push_back(indices[j * 3 + 2]);
                points.push_back(glm::vec4(face_mesh.vertices[indices[j * 3 + 0]], 1.0f));
                points.push_back(glm::vec4(face_mesh.vertices[indices[j * 3 + 1]], 1.0f));
                points.push_back(glm::vec4(face_mesh.vertices[indices[j * 3 + 1]], 1.0f));
                points.push_back(glm::vec4(face_mesh.vertices[indices[j * 3 + 2]], 1.0f));
                points.push_back(glm::vec4(face_mesh.vertices[indices[j * 3 + 2]], 1.0f));
                points.push_back(glm::vec4(face_mesh.vertices[indices[j * 3 + 0]], 1.0f));
            }

            ArPose_destroy(ar_pose);
            ArTrackable_release(face);
        }
        ArTrackableList_destroy(faces);
#endif
    }

    void ARCore::UpdateFeaturePoints() {
        if (!UpdateAnchor()) {
            points.clear();
            return;
        }

        ArPointCloud *ar_point_cloud = nullptr;
        ArStatus point_cloud_status = ArFrame_acquirePointCloud(ar_session_, ar_frame_, &ar_point_cloud);
        int32_t number_of_points = 0;
        if (point_cloud_status == AR_SUCCESS) {
            //get point cloud
            ArPointCloud_getNumberOfPoints(ar_session_, ar_point_cloud, &number_of_points);
            const float *point_cloud_data;
            ArPointCloud_getData(ar_session_, ar_point_cloud, &point_cloud_data);

            if (number_of_points > 0)
                points.clear();
            for (int i = 0; i < number_of_points * 4; i += 4) {
                points.push_back(glm::vec4(point_cloud_data[i + 0], point_cloud_data[i + 1],
                                           point_cloud_data[i + 2], point_cloud_data[i + 3]));
            }
            ArPointCloud_release(ar_point_cloud);
        }

        if (useDepth) {
            points.clear();
            camera.InitARCore(ar_session_, ar_frame_);
            if (number_of_points > 0) {

                ArImage* image = 0;
                int result = 0;
                if (useDepthRaw) {
                    result = ArFrame_acquireRawDepthImage(ar_session_, ar_frame_, &image);
                } else {
                    result = ArFrame_acquireDepthImage(ar_session_, ar_frame_, &image);
                }
                if (result == AR_SUCCESS) {

                    uint8_t* confidenceData;
                    ArImage* confidenceImage = 0;
                    bool hasConfidence = false;
                    if (useDepthRaw) {
                        result = ArFrame_acquireRawDepthConfidenceImage(ar_session_, ar_frame_, &confidenceImage);;
                        if (result == AR_SUCCESS) {
                            int32_t dataLength;
                            ArImage_getPlaneData(ar_session_, confidenceImage, 0, (const uint8_t**)&confidenceData, &dataLength);
                            hasConfidence = true;
                        }
                    }

                    //get depth data
                    uint16_t* imgData;
                    int64_t timestamp;
                    int32_t dataLength;
                    int32_t depthWidth, depthHeight, stride;
                    ArImage_getPlaneRowStride(ar_session_, image, 0, &stride);
                    ArImage_getPlaneData(ar_session_, image, 0, (const uint8_t**)&imgData, &dataLength);
                    ArImage_getWidth(ar_session_, image, &depthWidth);
                    ArImage_getHeight(ar_session_, image, &depthHeight);
                    ArImage_getTimestamp(ar_session_, image, &timestamp);

                    ArImage* image2 = 0;
                    uint16_t* imgData2;
                    bool hasSecondary = false;
                    if (useDepthRaw && !has_depth_sensor) {
                        result = ArFrame_acquireDepthImage(ar_session_, ar_frame_, &image2);
                        if (result == AR_SUCCESS) {
                            hasSecondary = true;
                            ArImage_getPlaneData(ar_session_, image2, 0, (const uint8_t**)&imgData2, &dataLength);
                        }
                    }

                    //convert depthmap to pointcloud
                    int minConfidence = has_depth_sensor ? 32 : 128;
                    float maxErrorFilter = resolution * 3.0f;
                    float maxErrorHoles = resolution * 3.0f;
                    float maxErrorWalls = resolution * 3.0f;
                    double len = 100 - 0.001f; //far - near
                    std::vector<glm::vec3> refused;
                    std::map<std::pair<int, int>, double> edges2d;
                    glm::vec3 cam = glm::inverse(view_mat)[3];
                    glm::dmat4 screen2world = glm::inverse(projection_mat * view_mat);
                    if (!has_depth_sensor || (lastDepthTimestamp != timestamp)) {

                        float maxY = INT_MIN;
                        std::map<int, float> distances;
                        std::map<std::pair<int, int>, float> distancesLocal;
                        float s = 1;
                        int m = has_depth_sensor ? (depthWidth - depthHeight) / 2 : 0;
                        if (depthWidth > 240) s = depthWidth / 240.0f;
                        for (float fy = 0; fy < depthHeight; fy += s) {
                            for (float fx = m; fx < depthWidth - m; fx += s) {
                                int x = (int)fx;
                                int y = (int)fy;
                                if ((x < 4) && (y == 0))
                                    continue;

                                //check point validity
                                double depth = (imgData[y * stride / 2 + x] & 0xFFFF) * 0.001f;
                                if (hasConfidence) {
                                    if (confidenceData[y * depthWidth + x] <= minConfidence) {
                                        if (hasSecondary) {

                                            //get nearest point with high confidence in 4 directions
                                            bool left = false, right = false, up = false, down = false;
                                            glm::vec3 c, l, r, u, d;
                                            for (int tx = x; tx >= 0; tx--) {
                                                if (confidenceData[y * depthWidth + tx] > minConfidence) {
                                                    l = glm::vec3(tx, y, (imgData[y * stride / 2 + tx] & 0xFFFF) * 0.001f);
                                                    left = true;
                                                    break;
                                                }
                                            }
                                            for (int tx = x; tx < depthWidth; tx++) {
                                                if (confidenceData[y * depthWidth + tx] > minConfidence) {
                                                    r = glm::vec3(tx, y, (imgData[y * stride / 2 + tx] & 0xFFFF) * 0.001f);
                                                    right = true;
                                                    break;
                                                }
                                            }
                                            for (int ty = y; ty >= 0; ty--) {
                                                if (confidenceData[ty * depthWidth + x] > minConfidence) {
                                                    u = glm::vec3(x, ty, (imgData[ty * stride / 2 + x] & 0xFFFF) * 0.001f);
                                                    up = true;
                                                    break;
                                                }
                                            }
                                            for (int ty = y; ty < depthHeight; ty++) {
                                                if (confidenceData[ty * depthWidth + x] > minConfidence) {
                                                    d = glm::vec3(x, ty, (imgData[ty * stride / 2 + x] & 0xFFFF) * 0.001f);
                                                    down = true;
                                                    break;
                                                }
                                            }
                                            c = glm::vec3(x, y, (imgData2[y * stride / 2 + x] & 0xFFFF) * 0.001f);

                                            //"closed" holes filling
                                            bool horizontal = false, vertical = false;
                                            if (left && right) {
                                                glm::vec3 v = glm::lerp(l, r, (c.x - l.x) / (r.x - l.x));
                                                if (fabs(v.z - c.z) < maxErrorHoles) {
                                                    horizontal = true;
                                                }
                                            }
                                            if (up && down) {
                                                glm::vec3 v = glm::lerp(u, d, (c.x - u.x) / (d.x - u.x));
                                                if (fabs(v.z - c.z) < maxErrorHoles) {
                                                    vertical = true;
                                                }
                                            }
                                            if (horizontal || vertical) {
                                                depth = c.z;
                                            } else {

                                                //store the refused point to process later
                                                depth = c.z;
                                                refused.emplace_back(ToPoint(screen2world, len, depthWidth, depthHeight, x, y, depth));

                                                //mark edge points for wall validation
                                                if (up && !down) edges2d[std::pair<int, int>(u.x, u.y)] = u.z;
                                                if (!up && down) edges2d[std::pair<int, int>(d.x, d.y)] = d.z;
                                                if (left && !right) edges2d[std::pair<int, int>(l.x, l.y)] = l.z;
                                                if (!left && right) edges2d[std::pair<int, int>(r.x, r.y)] = r.z;
                                                continue;
                                            }
                                        } else {
                                            continue;
                                        }
                                    }
                                }

                                //filter depth noise
                                if (hasSecondary) {
                                    double filtered = (imgData2[y * stride / 2 + x] & 0xFFFF) * 0.001f;
                                    if (fabs(depth - filtered) < maxErrorFilter) {
                                        depth = filtered;
                                    }
                                }

                                //add point into output
                                if (depth > 0.05) {
                                    depth -= offset;
                                    glm::vec4 p = ToPoint(screen2world, len, depthWidth, depthHeight, x, y, depth);
                                    if (maxY < p.y) maxY = p.y;
                                    points.emplace_back(p);

                                    if (!has_depth_sensor) {
                                        int dir = (int)glm::degrees(atan2(p.y - cam.y, p.x - cam.x));
                                        float dst = glm::distance(cam, glm::vec3(p));
                                        if (distances.find(dir) == distances.end()) {
                                            distances[dir] = dst;
                                        } else if (distances[dir] > dst) {
                                            distances[dir] = dst;
                                        }
                                        std::pair<int, int> key;
                                        key.first = dir;
                                        key.second = (int)(p.y * 10);
                                        if (distancesLocal.find(key) == distancesLocal.end()) {
                                            distancesLocal[key] = dst;
                                        } else if (distancesLocal[key] > dst) {
                                            distancesLocal[key] = dst;
                                        }
                                    }
                                }
                            }
                        }


                        //convert edge points into 3D space
                        std::vector<glm::vec3> edges3d;
                        for (std::pair<const std::pair<int, int>, double>& e : edges2d) {
                            int x = e.first.first;
                            int y = e.first.second;
                            double depth = e.second;
                            edges3d.emplace_back(ToPoint(screen2world, len, depthWidth, depthHeight, x, y, depth));
                        }

                        //add wall points
                        for (glm::vec3& r : refused) {
                            bool ok = false;
                            glm::vec3 v(INT_MIN);
                            for (glm::vec3& p : edges3d) {
                                if ((r.y > p.y) && (r.y < maxY)) {
                                    float x = fabs(p.x - r.x);
                                    float z = fabs(p.z - r.z);
                                    if (x * x + z * z < maxErrorWalls * maxErrorWalls) {
                                        if (v.y < p.y) {
                                            ok = true;
                                            v = p;
                                        }
                                    }
                                }
                            }
                            if (ok) {
                                int dir = (int)glm::degrees(atan2(v.y - cam.y, v.x - cam.x));
                                if (distances.find(dir) != distances.end()) {
                                    float dst = glm::distance(cam, v);
                                    if ((distances[dir] - maxErrorWalls < dst)) {
                                        std::pair<int, int> key;
                                        key.first = dir;
                                        key.second = (int)(r.y * 10);
                                        if (distancesLocal.find(key) == distancesLocal.end()) {
                                            points.emplace_back(r, 1.0f);
                                        } else if (distancesLocal[key] + maxErrorWalls > dst) {
                                            points.emplace_back(r, 1.0f);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    //cleanup
                    if (hasConfidence) {
                        ArImage_release(confidenceImage);
                    }
                    if (hasSecondary) {
                        ArImage_release(image2);
                    }
                    ArImage_release(image);
                    lastDepthTimestamp = timestamp;
                }
            }
        }
        has_coordinate_system_ = true;
    }
}
