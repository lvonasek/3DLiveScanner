#include <sstream>
#include <unistd.h>

#include <arcore/service.h>
#include <thread/reconstr.h>

namespace oc {

    cv::Ptr< cv::ORB > detector = cv::ORB::create();
    cv::Ptr< cv::ORB > extractor = cv::ORB::create();
    cv::BFMatcher matcher( cv::NORM_HAMMING2, true );

    Reconstruction* g_reconstruction = nullptr;

    void* ProcessDummy(void*) {
        usleep(100000);
        pthread_detach(g_reconstruction->threadId);
        g_reconstruction->binder_mutex_.unlock();
        return 0;
    }

    void* ProcessPoseCorrection(void*) {

        double best_accuracy = INT_MAX;
        glm::mat4 orig_mat = g_reconstruction->frame_viewmat;
        glm::mat4 best_mat = g_reconstruction->frame_viewmat;
        Reconstruction::CVDescription camera = g_reconstruction->DetectFeatures(g_reconstruction->request_image);
        Reconstruction::CVDescription render = g_reconstruction->DetectFeatures(g_reconstruction->rendered_image);
        g_reconstruction->request_newprojection = false;

        //validate continuity
        if (camera.keypoints.empty() || render.keypoints.empty()) {
            g_reconstruction->Start(Reconstruction::RECONSTRUCTION);
            pthread_detach(g_reconstruction->threadId);
            return 0;
        }

        //convert points into 3D
        std::vector<glm::vec3> points3d;
        float w = g_reconstruction->request_image->GetWidth();
        float h = g_reconstruction->request_image->GetHeight();
        if (g_reconstruction->scene.static_meshes_.empty()) {
            glm::mat4 screen2world = glm::inverse(g_reconstruction->frame_pose[SCREEN_CAMERA]);
            for (cv::KeyPoint& p : render.keypoints) {
                glm::vec4 depth = g_reconstruction->rendered_depth->GetColorRGBA(p.pt.x, p.pt.y);
                float x = 2.0f * p.pt.x / w - 1.0f;
                float y = 2.0f * p.pt.y / h - 1.0f;
                float z = (depth.r + depth.g + depth.b) / 255.0f * 2.0f;
                glm::vec4 v = screen2world * glm::vec4(x, y, z, 1.0f);
                points3d.emplace_back(v / fabs(v.w));
            }
        } else {
            std::vector<glm::vec2> points2d;
            for (cv::KeyPoint& p : render.keypoints) {
                points2d.emplace_back(p.pt.x, h - p.pt.y);
            }
            points3d = g_reconstruction->selector.Transform(g_reconstruction->scene.static_meshes_,
                                                            g_reconstruction->frame_pose[SCREEN_CAMERA],
                                                            points2d);
        }

        std::vector<cv::DMatch> allMatches, matches;
        matcher.match(camera.descriptors, render.descriptors, allMatches);
        for (cv::DMatch& m : allMatches) {
            cv::Point2f p1 = camera.keypoints[m.queryIdx].pt;
            cv::Point2f p2 = render.keypoints[m.trainIdx].pt;
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            float diff = sqrt(dx * dx + dy * dy);
            if (diff < 10) {
                matches.push_back(m);
            }
        }

        while (true) {
            while (true) {

                //get next matrix
                if (g_reconstruction->request_matrix.empty()) {
                    break;
                }
                glm::mat4 matrix = g_reconstruction->request_matrix[0];
                g_reconstruction->request_matrix.erase(g_reconstruction->request_matrix.begin());

                //reproject CV descriptors
                glm::mat4 m = g_reconstruction->scene.renderer->camera.projection * matrix;
                for (int i = 0; i < points3d.size(); i++) {
                    glm::vec4 v = m * glm::vec4(points3d[i], 1);
                    v /= fabs(v.w);
                    v = 0.5f * v + 0.5f;
                    render.keypoints[i].pt.x = v.x * w;
                    render.keypoints[i].pt.y = v.y * h;
                }

                //determine pose accuracy
                double accuracy = g_reconstruction->GetAccuracy(matches, camera, render);

                //decision on better pose
                if (best_accuracy > accuracy) {
                    best_accuracy = accuracy;
                    best_mat = matrix;
                }
            }

            //apply value
            g_reconstruction->frame_pose = ARCoreService::GetPose(g_reconstruction->frame_calibration, best_mat);
            g_reconstruction->frame_viewmat = best_mat;

            //next steps
            if ((g_reconstruction->request_distance > 0.0005f) && (best_accuracy != INT_MAX)) {
                g_reconstruction->request_distance *= 0.25f;
                g_reconstruction->AddPoses();
            } else {
                break;
            }
        }

        /*
        g_reconstruction->event_mutex_.lock();
        sprintf(g_reconstruction->pose_feedback, "%f", best_accuracy);
        g_reconstruction->event_mutex_.unlock();
        usleep(1000000);
         */


        //start reconstruction
        if (best_accuracy > 3.0f) {
            g_reconstruction->frame_pose = ARCoreService::GetPose(g_reconstruction->frame_calibration, orig_mat);
            g_reconstruction->frame_viewmat = orig_mat;
        }
        g_reconstruction->Start(Reconstruction::RECONSTRUCTION);
        pthread_detach(g_reconstruction->threadId);
        return 0;
    }

    void* ProcessReconstruction(void*) {

        //process camera calibration
        int scale = g_reconstruction->frame_image->GetWidth() > 1000 ? 3 : 1;
        Tango3DR_CameraCalibration camera = g_reconstruction->GetCalibration(scale);
        Tango3DR_ReconstructionContext_setColorCalibration(g_reconstruction->scan.Context(), &camera);

        //quit if there is nothing to be processed
        if (g_reconstruction->frame_points.empty()) {
            pthread_detach(g_reconstruction->threadId);
            g_reconstruction->binder_mutex_.unlock();
            return 0;
        }

        g_reconstruction->render_mutex_.lock();
        g_reconstruction->depth.ADD(g_reconstruction->frame_points, g_reconstruction->frame_pose[COLOR_CAMERA], g_reconstruction->frame_image);
        g_reconstruction->render_mutex_.unlock();

        //get data in Tango3DR format
        Tango3DR_ImageBuffer t3dr_image;
        t3dr_image.width = (uint32_t) g_reconstruction->frame_image->GetWidth() / scale;
        t3dr_image.height = (uint32_t) g_reconstruction->frame_image->GetHeight() / scale;
        t3dr_image.stride = (uint32_t) (g_reconstruction->frame_image->GetWidth()) / scale;
        t3dr_image.timestamp = g_reconstruction->frame_timestamp;
        t3dr_image.format = TANGO_3DR_HAL_PIXEL_FORMAT_YCrCb_420_SP;
        t3dr_image.data = g_reconstruction->frame_image->ExtractYUVDownscaled(scale);
        Tango3DR_Pose image_pose = g_reconstruction->texturize.Extract3DRPose(g_reconstruction->frame_pose[COLOR_CAMERA]);
        g_reconstruction->frame_pose[OPENGL_CAMERA] = g_reconstruction->frame_viewmat;

        //process pointcloud
        if (!g_reconstruction->scan.Update(&g_reconstruction->depth, g_reconstruction->frame_timestamp,
                &image_pose, &t3dr_image, &image_pose, g_reconstruction->holes_filling)) {
            pthread_detach(g_reconstruction->threadId);
            g_reconstruction->binder_mutex_.unlock();
            return 0;
        }

        //confirm that frame is in dataset
        camera = g_reconstruction->GetCalibration(1);
        g_reconstruction->dataset->WriteDistortion(g_reconstruction->frame_distortion);
        g_reconstruction->texturize.Add(g_reconstruction->frame_image, g_reconstruction->frame_timestamp,
                                        &camera, g_reconstruction->frame_pose, g_reconstruction->dataset);

        //process reconstructed geometry
        int index = g_reconstruction->texturize.GetLatestIndex(g_reconstruction->dataset);
        g_reconstruction->dataset->WritePreview(index, g_reconstruction->scan.Added());
        g_reconstruction->render_mutex_.lock();
        g_reconstruction->scan.Merge();
        g_reconstruction->frame_index = index;
        g_reconstruction->render_mutex_.unlock();

        //save point cloud into dataset
        Tango3DR_PointCloud* pcl = g_reconstruction->depth.PCL(g_reconstruction->frame_timestamp);
        g_reconstruction->dataset->WritePointCloud(index, *pcl);
        Tango3DR_PointCloud_destroy(pcl);

        //postprocess pointcloud
        g_reconstruction->depth.RES(g_reconstruction->scan.Resolution());
        if (g_reconstruction->frame_sparse) g_reconstruction->depth.UPD(g_reconstruction->frame_image, g_reconstruction->frame_pose[COLOR_CAMERA], true);
        if (g_reconstruction->holes_filling) g_reconstruction->depth.ADD(g_reconstruction->scan.Components(), g_reconstruction->frame_pose[COLOR_CAMERA], false);

        //photo mode
        if (g_reconstruction->photo_mode) {
            g_reconstruction->t3dr_is_running_ = false;
        }

        g_reconstruction->request_newprojection = true;
        g_reconstruction->binder_mutex_.unlock();
        pthread_detach(g_reconstruction->threadId);
        return 0;
    }

    Reconstruction::Reconstruction() {
        dataset = nullptr;
        jumped = false;
        paused = true;
        tracked = false;
        lost = false;
        photo_mode = false;
        request_newprojection = true;
        t3dr_is_running_ = false;

        sprintf(pose_feedback, "");
        g_reconstruction = this;
        selector.Init(360, 640);
    }

    void Reconstruction::AddPoses() {
        float m = request_distance;
        float step = request_distance * 0.25f;
        for (float x = -m; x <= m; x += step) {
            for (float y = -m; y <= m; y += step) {
                for (float z = -m; z <= m; z += step) {
                    if ((fabs(x) > 0) || (fabs(y) > 0) || (fabs(z) > 0)) {
                        if (fabs(x) + fabs(y) + fabs(z) < request_distance * 1.5f) {
                            glm::mat4 mat = frame_viewmat;
                            mat[3][0] += x;
                            mat[3][1] += y;
                            mat[3][2] += z;
                            request_matrix.push_back(mat);
                        }
                    }
                }
            }
        }
    }

    Reconstruction::CVDescription Reconstruction::DetectFeatures(Image* frame) {
        cv::Mat mat(frame->GetHeight(), frame->GetWidth(), CV_8UC1);
        for (int x = 0; x < frame->GetWidth(); x++) {
            for (int y = 0; y < frame->GetHeight(); y++) {
                glm::ivec4 color = frame->GetColorRGBA(x, y);
                mat.at<uchar>(y, x) = (color.r + color.g + color.b) / 3;
            }
        }
        CVDescription output;
        detector->detect(mat, output.keypoints);
        if (!output.keypoints.empty()) {
            extractor->compute(mat, output.keypoints, output.descriptors);
        }
        return output;
    }

    double Reconstruction::GetAccuracy(std::vector<cv::DMatch>& matches, Reconstruction::CVDescription& a, Reconstruction::CVDescription& b) {
        std::vector<float> errors;
        for (cv::DMatch& m : matches) {
            cv::Point2f p1 = a.keypoints[m.queryIdx].pt;
            cv::Point2f p2 = b.keypoints[m.trainIdx].pt;
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            float diff = sqrt(dx * dx + dy * dy);
            errors.push_back(diff);
        }
        double error = INT_MAX;
        if (!errors.empty()) {
            error = 0;
            int size = errors.size() / 2;
            std::sort(errors.begin(), errors.end());
            for (int i = 0; i < size; i++) {
                error += errors[i] / (float)size;
            }
        }
        return error;
    }

    Tango3DR_CameraCalibration Reconstruction::GetCalibration(int scale) {
        float w = frame_image->GetWidth() / scale;
        float h = frame_image->GetHeight() / scale;
        Tango3DR_CameraCalibration camera;
        texturize.ApplyDistortion(camera, frame_distortion);
        camera.width = (uint32_t) w;
        camera.height = (uint32_t) h;
        camera.cx = fabs(w * (1.0f - frame_calibration[2][0]) / 2.0f);
        camera.cy = fabs(h * (1.0f - frame_calibration[2][1]) / 2.0f);
        camera.fx = fabs(w * frame_calibration[0][0] / 2.0f);
        camera.fy = fabs(h * frame_calibration[1][1] / 2.0f);
        return camera;
    }

    std::string Reconstruction::GetEvent() {
        event_mutex_.lock();
        std::string output = event_;
        if (strlen(pose_feedback) > 0)
            output = pose_feedback;

        if (lost)
            output = tracked ? "MT_LOST" : "MT_INIT";
        if (!texturize.GetEvent().empty())
            output = texturize.GetEvent();
        if (!GLSL::GetError().empty())
            output = GLSL::GetError();
        if (output.empty()) {
            //output = depth.DBG() + scan.DebugInfo();
        }
        event_ = "";
        event_mutex_.unlock();
        return output;
    }

    bool Reconstruction::InitTexturing(std::string input, bool fast) {
        if (!texturize.Init(input, true, fast)) {
            render_mutex_.unlock();
            binder_mutex_.unlock();
            return false;
        }
        return true;
    }

    void Reconstruction::PreviewChange(int frames) {
        binder_mutex_.lock();

        //update cache
        int last = texturize.GetLatestIndex(dataset);
        if (preview_start.size() != last) {
            preview_start.clear();
            for (int i = 0; i <= last; i++) {
                for (std::pair<GridIndex, Tango3DR_Mesh *>& p : dataset->ReadPreview(i, true)) {
                    if (preview_start.find(p.first) == preview_start.end()) {
                        preview_start[p.first] = i;
                    }
                }
            }
        }

        //select frame
        std::vector<int> dataIndex;
        long frame_previous = frame_index;
        frame_index += frames;
        if (frame_index <= -1) frame_index = -1;
        if (frame_index >= last) frame_index = last;
        if (frame_index >= 0) {
            if (frames > 0) {
                for (int i = frame_previous + 1; i <= frame_index; i++) {
                    dataIndex.push_back(i);
                }
            } else {
                for (int i = frame_previous - 1; i >= frame_index; i--) {
                    dataIndex.push_back(i);
                }
            }
        }

        //prepare frames
        std::vector<std::pair<GridIndex, Tango3DR_Mesh *> > data;
        for (int i : dataIndex) {
            for (std::pair<GridIndex, Tango3DR_Mesh *>& d : dataset->ReadPreview(i, false)) {
                for (int j = data.size() - 1; j >= 0; j--) {
                    if (data[j].first == d.first) {
                        Tango3DR_Status ret = Tango3DR_Mesh_destroy(data[j].second);
                        if (ret != TANGO_3DR_SUCCESS)
                            exit(EXIT_SUCCESS);
                        data.erase(data.begin() + j);
                        if (data.empty()) {
                            break;
                        }
                    }
                }
                data.push_back(d);
            }
        }

        //apply frames
        std::vector<GridIndex> toDelete;
        for (std::pair<const GridIndex, int>& startIndex : preview_start) {
            if (startIndex.second > frame_index) {
                toDelete.push_back(startIndex.first);
            }
        }
        render_mutex_.lock();
        scan.Merge(data);
        scan.Delete(toDelete);
        render_mutex_.unlock();
        binder_mutex_.unlock();
    }

    void Reconstruction::RenderGL(glm::mat4 view) {
        int w = request_image->GetWidth();
        int h = request_image->GetHeight();
        if (!renderer) {
            renderer = new GLRenderer();
            renderer->Init(scene.renderer->width, scene.renderer->height, w, h);
        }
        glClearColor(1, 1, 1, 1);
        renderer->Rtt(true);
        glViewport(0, 0, w, h);
        scene.CustomRender(frame_calibration * view);
        renderer->Rtt(false);

        if (rendered_image) {
            delete rendered_image;
        }
        rendered_image = renderer->ReadRtt(0, 0, w, h);
    }

    void Reconstruction::Setup(double res, double dmin, double dmax, int noise, bool holesFilling,
                               bool poseCorrection, bool distortion, bool clearing, std::string dataset_path) {
        holes_filling = holesFilling;
        pose_correction = poseCorrection;
        lost = true;
        texturize.SetDistortion(distortion);

        binder_mutex_.lock();
        dataset = new Dataset(dataset_path);
        scan.Setup3DR(fabs(res), dmin, dmax, noise, clearing);
        binder_mutex_.unlock();
    }


    void Reconstruction::SetPhotoMode(bool on) {
        binder_mutex_.lock();
        photo_mode = on;
        binder_mutex_.unlock();
    }

    void Reconstruction::Start(ReconstructionThread thread) {
        struct thread_info *tinfo = 0;
        switch (thread) {
            case DUMMY:
                pthread_create(&threadId, NULL, ProcessDummy, tinfo);
                break;
            case POSE_CORRECTION:
                pthread_create(&threadId, NULL, ProcessPoseCorrection, tinfo);
                break;
            case RECONSTRUCTION:
                pthread_create(&threadId, NULL, ProcessReconstruction, tinfo);
                break;
        }
    }

    void Reconstruction::Undo(bool fromUser, bool applyTextures) {
        binder_mutex_.lock();
        render_mutex_.lock();
        request_newprojection = true;
        if (!scan.Data().empty() || !fromUser) {
            scan.ClearContext();
            render_mutex_.unlock();

            if (fromUser) {
                while (true) {
                    int index = texturize.GetLatestIndex(dataset);
                    if (index < 0)
                        break;
                    if (frame_index < index) {
                        texturize.DeleteLast(dataset);
                    } else {
                        break;
                    }
                }
            }

            //mesh reconstruction
            double temp;
            Tango3DR_Status ret;
            int count, width, height;
            dataset->ReadState(count, width, height, temp, temp, temp, temp);
            int scale = width > 1000 ? 3 : 1;
            texturize.SetCalibration(scan.Context(), dataset, scale);
            Tango3DR_ImageBuffer image;
            image.width = (uint32_t) width / scale;
            image.height = (uint32_t) height / scale;
            image.stride = (uint32_t) width / scale;
            image.format = TANGO_3DR_HAL_PIXEL_FORMAT_YCrCb_420_SP;
            if (!applyTextures) image.data = new unsigned char[width / scale * height / scale * 2];
            std::unordered_map<GridIndex, bool, GridIndexHasher> added;
            for (int i = 0; i < count; i++) {
                std::ostringstream ss;
                ss << "CONVERT ";
                ss << i + 1;
                ss << "/";
                ss << count + 1;
                texturize.SetEvent(ss.str());

                Tango3DR_Pose image_pose = texturize.Extract3DRPose(dataset->ReadPose(i)[COLOR_CAMERA]);
                Tango3DR_PointCloud t3dr_depth = dataset->ReadPointCloud(i);

                Tango3DR_GridIndexArray t3dr_updated;
                if (applyTextures) image.data = Image(dataset->GetFileName(i, ".jpg")).ExtractYUVDownscaled(scale);
                ret = Tango3DR_updateFromPointCloud(scan.Context(), &t3dr_depth, &image_pose,
                                                    &image, &image_pose, &t3dr_updated);
                if (ret == TANGO_3DR_SUCCESS) {
                    GridIndex index;
                    unsigned long size = t3dr_updated.num_indices;
                    for (unsigned long it = 0; it < size; ++it) {
                        index.indices[0] = t3dr_updated.indices[it][0];
                        index.indices[1] = t3dr_updated.indices[it][1];
                        index.indices[2] = t3dr_updated.indices[it][2];
                        added[index] = true;
                    }
                    Tango3DR_GridIndexArray_destroy(&t3dr_updated);
                }
                Tango3DR_PointCloud_destroy(&t3dr_depth);
                if (applyTextures) delete[] image.data;
            }
            if (!applyTextures) delete[] image.data;
            texturize.SetEvent("");

            //unpack results
            std::vector<std::pair<GridIndex, Tango3DR_Mesh*> > toAdd;
            for (auto &p : added) {
                std::pair<GridIndex, Tango3DR_Mesh*> pair;
                pair.first = p.first;
                pair.second = new Tango3DR_Mesh();
                ret = Tango3DR_extractMeshSegment(scan.Context(), pair.first.indices, pair.second);
                if (ret != TANGO_3DR_SUCCESS)
                    exit(EXIT_SUCCESS);
                toAdd.push_back(pair);
            }

            //merge results
            render_mutex_.lock();
            scan.ClearGeometry();
            for (auto &p : toAdd) {
                scan.Add(p.first, p.second);
            }
            frame_index = count;
        }
        event_mutex_.lock();
        sprintf(pose_feedback, "");
        event_mutex_.unlock();

        render_mutex_.unlock();
        binder_mutex_.unlock();
    }
}
