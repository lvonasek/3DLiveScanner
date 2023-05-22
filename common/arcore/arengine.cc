#include <arcore/arengine.h>
#include <media/NdkImageReader.h>
#include <delaunay.h>
#include "service.h"

namespace oc {

    AREngine::AREngine(void *env, void *context, bool depthCamera, bool faceMode, bool flashlight) {
        offset = 0;
        resolution = 0;
        useDepth = depthCamera;
        face_mode_ = faceMode;
        has_coordinate_system_ = false;

        if (env && context) {
            HwArEnginesSelector_checkAllAvailableEngines(env, context);
            HwArEnginesSelector_setAREngine(HWAR_ENGINE);
            HwArInstallStatus install_status;
            if (HwArEnginesApk_requestInstall(env, context, true, &install_status) == HWAR_SUCCESS) {
                switch (install_status) {
                    case HWAR_INSTALL_STATUS_INSTALLED:
                        break;
                    case HWAR_INSTALL_STATUS_INSTALL_REQUESTED:
                        return;
                }
            }

            HwArSession_create(env, context, &ar_session_);
            HwArConfig *ar_config = nullptr;
            HwArConfig_create(ar_session_, &ar_config);
            HwArConfig_setCameraLensFacing(ar_session_, ar_config, faceMode ? HWAR_CAMERA_FACING_FRONT : HWAR_CAMERA_FACING_REAR);
            HwArConfig_setFocusMode(ar_session_, ar_config, HWAR_FOCUS_MODE_AUTO);
            HwArConfig_setPlaneFindingMode(ar_session_, ar_config, HWAR_PLANE_FINDING_MODE_DISABLED);
            HwArConfig_setPowerMode(ar_session_, ar_config, HWAR_POWER_MODE_PERFORMANCE_FIRST);
            HwArConfig_setUpdateMode(ar_session_, ar_config, HWAR_UPDATE_MODE_BLOCKING);
            if (faceMode) {
                HwArConfig_setArType(ar_session_, ar_config, HWAR_TYPE_FACE);
            } else {
                if (flashlight) {
                    if (depthCamera)
                        HwArConfig_setEnableItem(ar_session_, ar_config, HWAR_ENABLE_DEPTH | HWAR_ENABLE_FLASH_MODE_TORCH);
                    else
                        HwArConfig_setEnableItem(ar_session_, ar_config, HWAR_ENABLE_FLASH_MODE_TORCH);
                } else {
                    if (depthCamera)
                        HwArConfig_setEnableItem(ar_session_, ar_config, HWAR_ENABLE_DEPTH);
                    else
                        HwArConfig_setEnableItem(ar_session_, ar_config, HWAR_ENABLE_NULL);
                }
            }

            HwArSession_configure(ar_session_, ar_config);
            HwArConfig_destroy(ar_config);
            HwArFrame_create(ar_session_, &ar_frame_);
        }
    }

    AREngine::~AREngine() {
        HwArSession_destroy(ar_session_);
        HwArFrame_destroy(ar_frame_);
    }

    void AREngine::Clear(bool detach) {
        if (detach) {
            for (auto& anchor : ar_anchor_list) {
                HwArAnchor_detach(ar_session_, ar_anchor_list[anchor.first]);
                HwArAnchor_release(ar_anchor_list[anchor.first]);
            }
        }
        ar_anchor_list.clear();
        has_coordinate_system_ = false;
    }

    void AREngine::OnPause() {
        HwArSession_pause(ar_session_);
    }

    void AREngine::OnResume() {
        HwArSession_resume(ar_session_);
        camera.InitializeGlContent();
        texture_initialized_ = false;
    }

    void AREngine::OnDisplayGeometryChanged(int display_rotation, int width, int height) {
        viewportWidth = width;
        viewportHeight = height;
        HwArSession_setDisplayGeometry(ar_session_, display_rotation, width, height);
    }

    void AREngine::Configure(void *session, void *frame) {
        ar_session_ = static_cast<HwArSession *>(session);
        ar_frame_ = static_cast<HwArFrame *>(frame);
    }

    float AREngine::CountFrameError() {
        int size = 0;
        float error = 10000;
        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        HwArPose *ar_pose;
        HwArPose_create(ar_session_, data, &ar_pose);
        glm::mat4 matrix = projection_mat * view_mat;

        HwArHitResult* hit = 0;
        HwArHitResultList* hits = 0;
        HwArHitResult_create(ar_session_, &hit);
        HwArHitResultList_create(ar_session_, &hits);
        for (glm::vec3& v : GetActiveAnchors()) {
            glm::vec4 point = matrix * glm::vec4(v, 1.0);
            point /= fabs(point.z * point.w);
            point = 0.5f * point + 0.5f;

            HwArFrame_hitTest(ar_session_, ar_frame_, viewportWidth * point.x, viewportHeight * point.y, hits);
            HwArHitResultList_getSize(ar_session_, hits, &size);
            for (int i = 0; i < size; i++) {
                HwArTrackable *trackable = 0;
                HwArHitResultList_getItem(ar_session_, hits, i, hit);
                HwArHitResult_acquireTrackable(ar_session_, hit, &trackable);
                HwArPoint_getPose(ar_session_, HwArAsPoint(trackable), ar_pose);
                HwArPose_getPoseRaw(ar_session_, ar_pose, data);
                glm::vec3 position = glm::vec3(data[4], data[5], data[6]);
                float dst = glm::distance(v, position);
                if (dst > 0) {
                    error = glm::min(error, dst);
                }
                HwArTrackable_release(trackable);
            }
        }

        HwArHitResultList_destroy(hits);
        HwArHitResult_destroy(hit);
        HwArPose_destroy(ar_pose);
        return error;
    }

    bool AREngine::Process(bool update) {
        if (update) {
            if (!texture_initialized_) {
                HwArSession_setCameraTextureName(ar_session_, camera.GetTextureName());
                texture_initialized_ = true;
            }
            if (HwArSession_update(ar_session_, ar_frame_) != HWAR_SUCCESS)
                return false;
        }

        HwArCamera *ar_camera;
        HwArFrame_acquireCamera(ar_session_, ar_frame_, &ar_camera);
        HwArCamera_getViewMatrix(ar_session_, ar_camera, glm::value_ptr(view_mat));
        HwArCamera_getProjectionMatrix(ar_session_, ar_camera, 0.001f, 100.f,
                                           glm::value_ptr(projection_mat));

        if (!face_mode_) {
            HwArTrackingState state = HWAR_TRACKING_STATE_STOPPED;
            HwArCamera_getTrackingState(ar_session_, ar_camera, &state);
            if (state != HWAR_TRACKING_STATE_TRACKING) {
                HwArCamera_release(ar_camera);
                return false;
            }
        }

        HwArCamera_release(ar_camera);
        return !GetActiveAnchors().empty() || ar_anchor_list.empty();
    }

    void AREngine::RenderCamera(ARCoreCamera::Effect effect, int scale) {
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
                camera.DrawAREngine(ar_session_, ar_frame_, effect, viewportWidth, viewportHeight);
                glDeleteTextures(1, &texture);
                delete img;
            }
        } else {
            camera.DrawAREngine(ar_session_, ar_frame_, effect, viewportWidth, viewportHeight);
        }
    }

    std::vector<glm::vec3> AREngine::GetActiveAnchors() {
        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        HwArPose *ar_pose;
        HwArPose_create(ar_session_, data, &ar_pose);
        std::vector<glm::vec3> output;

        for (auto& anchor : ar_anchor_list) {
            HwArTrackingState state = HWAR_TRACKING_STATE_STOPPED;
            HwArAnchor_getTrackingState(ar_session_, anchor.second, &state);
            if (state == HWAR_TRACKING_STATE_TRACKING) {
                HwArAnchor_getPose(ar_session_, anchor.second, ar_pose);
                HwArPose_getPoseRaw(ar_session_, ar_pose, data);
                glm::vec3 v = glm::vec3(data[4], data[5], data[6]);
                output.push_back(v);
            }
        }
        HwArPose_destroy(ar_pose);
        return output;
    }

    std::vector<float> AREngine::GetDistortion() {
        HwArCamera *ar_camera;
        HwArCameraIntrinsics* intrinsics;
        float distortion[DISTORTION_COUNT];
        HwArFrame_acquireCamera(ar_session_, ar_frame_, &ar_camera);
        HwArCameraIntrinsics_create(ar_session_, &intrinsics);
        HwArCamera_getImageIntrinsics(ar_session_, ar_camera, intrinsics);
        HwArCameraIntrinsics_getDistortion(ar_session_, intrinsics, distortion);
        HwArCameraIntrinsics_destroy(ar_session_, intrinsics);
        HwArCamera_release(ar_camera);

        std::vector<float> output;
        output.push_back(distortion[0]);
        output.push_back(distortion[1]);
        output.push_back(distortion[3]);
        output.push_back(distortion[4]);
        output.push_back(distortion[2]);
        return output;
    }

    glm::vec3 AREngine::HitTest(int x, int y) {
        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        HwArPose *ar_pose;
        HwArPose_create(ar_session_, data, &ar_pose);

        int size = 0;
        HwArHitResult* hit = 0;
        HwArHitResultList* hits = 0;
        HwArHitResult_create(ar_session_, &hit);
        HwArHitResultList_create(ar_session_, &hits);
        HwArFrame_hitTest(ar_session_, ar_frame_, x, y, hits);
        HwArHitResultList_getSize(ar_session_, hits, &size);
        if (size > 0) {
            HwArTrackable* trackable = 0;
            HwArHitResultList_getItem(ar_session_, hits, 0, hit);
            HwArHitResult_acquireTrackable(ar_session_, hit, &trackable);
            HwArPoint_getPose(ar_session_, HwArAsPoint(trackable), ar_pose);
            HwArPose_getPoseRaw(ar_session_, ar_pose, data);
            HwArTrackable_release(trackable);
            HwArHitResultList_destroy(hits);
            HwArHitResult_destroy(hit);
            HwArPose_destroy(ar_pose);
            return glm::vec3(data[4], data[5], data[6]);
        }
        HwArHitResultList_destroy(hits);
        HwArHitResult_destroy(hit);
        HwArPose_destroy(ar_pose);
        return glm::vec3(INT_MAX);
    }

    Image* AREngine::GetDepthMap(bool confidence, bool increasing, int s) {

        if (useDepth) {
            HwArImage* image = 0;
            if (HwArFrame_acquireDepthImage(ar_session_, ar_frame_, &image) == HWAR_SUCCESS) {

                //get depth data
                const AImage *depthMap = 0;
                HwArImage_getNdkImage(image, &depthMap);
                uint16_t *imgData;
                int dataLength;
                int32_t depthWidth = 0, depthHeight = 0, stride = 0;
                if (AImage_getWidth(depthMap, &depthWidth) != AMEDIA_OK) { HwArImage_release(image); return 0; }
                if (AImage_getHeight(depthMap, &depthHeight) != AMEDIA_OK) { HwArImage_release(image); return 0; }
                if (AImage_getPlaneRowStride(depthMap, 0, &stride) != AMEDIA_OK) { HwArImage_release(image); return 0; }
                if (AImage_getPlaneData(depthMap, 0, (uint8_t **) &imgData, &dataLength) != AMEDIA_OK) { HwArImage_release(image); return 0; }

                depthWidth /= s;
                depthHeight /= s;
                Image* output = new Image(depthWidth, depthHeight);
                for (int y = 0; y < depthHeight; y++) {
                    for (int x = 0; x < depthWidth; x++) {
                        int depth = static_cast<int>((imgData[s * y * stride / 2 + s * x] & 0x1FFF) * 0.001 * 255);
                        if (!increasing && depth > 0) depth = 768 - depth;
                        output->GetData()[(y * depthWidth + x) * 4 + 0] = camera.Convert(depth, 0);
                        output->GetData()[(y * depthWidth + x) * 4 + 1] = camera.Convert(depth, 1);
                        output->GetData()[(y * depthWidth + x) * 4 + 2] = camera.Convert(depth, 2);
                        output->GetData()[(y * depthWidth + x) * 4 + 3] = 255;
                        if (confidence) {
                            int depthConfidence = ((imgData[s * y * stride / 2 + s * x] >> 13) & 0x7);
                            float depthPercentage = depthConfidence == 0 ? 1.f : (depthConfidence - 1) / 7.f;
                            output->GetData()[(y * depthWidth + x) * 4 + 3] *= 0.5f + depthPercentage * 0.5f;
                        }
                    }
                }
                HwArImage_release(image);
                return output;
            }
        }
        return 0;
    }

    bool AREngine::UpdateAnchor() {
        if (!ar_anchor_list.empty() && GetActiveAnchors().empty()) {
            return false;
        }

        float data[7] = {0, 0, 0, 1, 0, 0, 0};
        HwArPose *ar_pose;
        HwArPose_create(ar_session_, data, &ar_pose);

        bool valid = false;
        HwArAnchor* ar_anchor_ = 0;
        int size = 0;
        HwArHitResult* hit = 0;
        HwArHitResultList* hits = 0;
        HwArHitResult_create(ar_session_, &hit);
        HwArHitResultList_create(ar_session_, &hits);
        for (float x = 0.25f; x <= 0.75f; x += 0.5f) {
            for (float y = 0.25f; y <= 0.75f; y += 0.5f) {
                HwArFrame_hitTest(ar_session_, ar_frame_, viewportWidth * x, viewportHeight * y, hits);
                HwArHitResultList_getSize(ar_session_, hits, &size);
                for (int i = 0; i < size; i++) {
                    HwArTrackable* trackable = 0;
                    HwArHitResultList_getItem(ar_session_, hits, i, hit);
                    HwArHitResult_acquireTrackable(ar_session_, hit, &trackable);
                    HwArTrackableType type = HWAR_TRACKABLE_NOT_VALID;
                    HwArTrackable_getType(ar_session_, trackable, &type);
                    if (type == HWAR_TRACKABLE_POINT) {

                        HwArPoint_getPose(ar_session_, HwArAsPoint(trackable), ar_pose);
                        HwArPose_getPoseRaw(ar_session_, ar_pose, data);
                        glm::vec3 v = glm::vec3(data[4], data[5], data[6]);

                        id3d pos;
                        float density = ANCHOR_DENSITY_BASE;
                        for (pos.layer = 0; pos.layer < ANCHOR_LAYERS; pos.layer++) {
                            pos.x = static_cast<int>(v.x / density);
                            pos.y = static_cast<int>(v.y / density);
                            pos.z = static_cast<int>(v.z / density);
                            valid = true;
                            if (ar_anchor_list.find(pos) == ar_anchor_list.end()) {
                                HwArStatus ret = HwArTrackable_acquireNewAnchor(ar_session_, trackable, ar_pose, &ar_anchor_);
                                if (ret == HWAR_SUCCESS) {
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
                                            HwArAnchor_detach(ar_session_, ar_anchor_list[far]);
                                            HwArAnchor_release(ar_anchor_list[far]);
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
                    HwArTrackable_release(trackable);
                }
            }
        }
        HwArHitResultList_destroy(hits);
        HwArHitResult_destroy(hit);
        HwArPose_destroy(ar_pose);
        return valid;
    }

    void AREngine::UpdateFace(glm::mat4 matrix) {
        face_mesh.vertices.clear();
        face_mesh.normals.clear();
        face_mesh.uv.clear();
        face_mesh.indices.clear();
        points.clear();
        int32_t size = 0;
        HwArTrackableList* faces = 0;
        HwArTrackableList_create(ar_session_, &faces);
        HwArSession_getAllTrackables(ar_session_, HWAR_TRACKABLE_FACE, faces);
        HwArTrackableList_getSize(ar_session_, faces, &size);
        for (int32_t i = 0; i < size; i++) {
            int32_t count = 0;
            const float* vertices = 0;
            HwArTrackable* face = 0;
            HwArFaceGeometry* geometry = 0;
            HwArTrackableList_acquireItem(ar_session_, faces, i, &face);
            HwArFace_acquireGeometry(ar_session_, (HwArFace*)face, &geometry);
            HwArFaceGeometry_acquireVertices(ar_session_, geometry, &vertices);
            HwArFaceGeometry_getVerticesSize(ar_session_, geometry, &count);

            float data[7] = {0, 0, 0, 1, 0, 0, 0};
            HwArPose *ar_pose;
            HwArPose_create(ar_session_, data, &ar_pose);
            HwArFace_getPose(ar_session_, (HwArFace*)face, ar_pose);
            HwArPose_getPoseRaw(ar_session_, ar_pose, data);

            GLCamera pose;
            pose.position = glm::vec3(data[4], data[5], data[6]);
            pose.rotation = glm::quat(data[3], data[0], data[1], data[2]);
            pose.scale = glm::vec3(1);
            glm::mat4 transform = pose.GetTransformation();
            const int32_t* indices = 0;
            int32_t triangles = 0;
            HwArFaceGeometry_acquireTriangleIndices(ar_session_, geometry, &indices);
            HwArFaceGeometry_getTriangleIndicesSize(ar_session_, geometry, &size);

#if USE_ARENGINE_FACE_INDICES
            for (int j = 0; j < size / 3; j++) {
                for (int k = 0; k < 3; k++) {
                    int index = indices[j * 3 + k];
                    glm::vec4 point = glm::vec4(vertices[index * 3 + 0],
                                                vertices[index * 3 + 1],
                                                vertices[index * 3 + 2],
                                                1.0f);

                    point = transform * point;
                    point /= fabs(point.w);
                    point.w = 1.0f;
                    points2d.emplace_back(vertices[index * 3 + 0], vertices[index * 3 + 1], point);

                    point = transform * point;
                    point /= fabs(point.w);
                    point.w = 1.0f;
                    face_mesh.vertices.emplace_back(point);
                    point = matrix * point;
                    point /= fabs(point.w);
                    face_mesh.uv.push_back(0.5f * glm::vec2(point.x, point.y) + 0.5f);
                    face_mesh.normals.emplace_back(0);
                }
            }
#else
            std::vector<dln::Vector2<float> > points2d;
            for (int j = 0; j < count / 3; j++) {
                glm::vec4 point = glm::vec4(vertices[j * 3 + 0],
                                            vertices[j * 3 + 1],
                                            vertices[j * 3 + 2],
                                            1.0f);

                point = transform * point;
                point /= fabs(point.w);
                point.w = 1.0f;
                float z = 1.0f + vertices[j * 3 + 2];
                points2d.emplace_back(vertices[j * 3 + 0] * z, vertices[j * 3 + 1] * z, point);
            }

            if (!points2d.empty()) {
                auto result = dln::Delaunay<float>().triangulate(points2d);
                for (const dln::Triangle<float>& t : result) {
                    if (t.isBad) continue;
                    glm::vec3 p[] = {t.p1.p, t.p2.p, t.p3.p};
                    for (int k = 0; k < 3; k++) {
                        face_mesh.vertices.emplace_back(p[k]);
                        glm::vec4 point = matrix * glm::vec4(p[k], 1.0f);
                        point /= fabs(point.w);
                        face_mesh.uv.push_back(0.5f * glm::vec2(point.x, point.y) + 0.5f);
                        face_mesh.normals.emplace_back(0);
                    }
                }
            }
#endif

            HwArFaceGeometry_release(geometry);
            HwArPose_destroy(ar_pose);
            HwArTrackable_release(face);
        }
        HwArTrackableList_destroy(faces);
    }

    void AREngine::UpdateFeaturePoints() {
        if (!UpdateAnchor()) {
            points.clear();
            return;
        }

        HwArPointCloud *ar_point_cloud = nullptr;
        HwArStatus point_cloud_status = HwArFrame_acquirePointCloud(ar_session_, ar_frame_, &ar_point_cloud);
        int32_t number_of_points = 0;
        if (point_cloud_status == HWAR_SUCCESS) {
            //get point cloud
            HwArPointCloud_getNumberOfPoints(ar_session_, ar_point_cloud, &number_of_points);
            const float *point_cloud_data;
            HwArPointCloud_getData(ar_session_, ar_point_cloud, &point_cloud_data);

            if (number_of_points > 0)
                points.clear();
            for (int i = 0; i < number_of_points * 4; i += 4) {
                points.push_back(glm::vec4(point_cloud_data[i + 0], point_cloud_data[i + 1],
                                           point_cloud_data[i + 2], point_cloud_data[i + 3]));
            }
            HwArPointCloud_release(ar_point_cloud);
        }

        if (useDepth) {
            camera.InitAREngine(ar_session_, ar_frame_);
            if (number_of_points > 0) {
                HwArImage *image = 0;
                if (HwArFrame_acquireDepthImage(ar_session_, ar_frame_, &image) == HWAR_SUCCESS) {

                    //get depth data
                    const AImage *depthMap = 0;
                    HwArImage_getNdkImage(image, &depthMap);
                    uint16_t *imgData;
                    int dataLength;
                    int32_t depthWidth = 0, depthHeight = 0, stride = 0;
                    if (AImage_getWidth(depthMap, &depthWidth) != AMEDIA_OK) { HwArImage_release(image); return; }
                    if (AImage_getHeight(depthMap, &depthHeight) != AMEDIA_OK) { HwArImage_release(image); return; }
                    if (AImage_getPlaneRowStride(depthMap, 0, &stride) != AMEDIA_OK) { HwArImage_release(image); return; }
                    if (AImage_getPlaneData(depthMap, 0, (uint8_t **) &imgData, &dataLength) != AMEDIA_OK) { HwArImage_release(image); return; }
                    points.clear();

                    //convert depthmap to pointcloud
                    double len = 100 - 0.001f; //far - near
                    glm::dmat4 screen2world = glm::inverse(projection_mat * view_mat);
                    for (int y = 0; y < depthHeight; y++) {
                        for (int x = 0; x < depthWidth; x++) {
                            if ((x < 4) && (y == 0))
                                continue;

                            double depth = (imgData[y * stride / 2 + x] & 0x1FFF) * 0.001f;
                            if ((depth > 0.05) && (depth < 15)) {
                                depth -= offset;

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
                                points.emplace_back(point0 + (point1 - point0) / len * depth);
                            }
                        }
                    }
                    HwArImage_release(image);
                }
            }
        }
        has_coordinate_system_ = true;
    }
}
