#ifndef THREAD_RECONSTRUCTION_H
#define THREAD_RECONSTRUCTION_H

#include <mutex>

#include <data/dataset.h>
#include <editor/selector.h>
#include <tango/retango.h>
#include <tango/scan.h>
#include <tango/texturize.h>
#include <thread/scene.h>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

namespace oc {

    class Reconstruction {
    public:
        enum ReconstructionThread { DUMMY, POSE_CORRECTION, RECONSTRUCTION };

        struct CVDescription {
            std::vector< cv::KeyPoint > keypoints;
            cv::Mat descriptors;
        };

        Reconstruction();
        void AddPoses();
        CVDescription DetectFeatures(Image* frame);
        double GetAccuracy(std::vector<cv::DMatch>& matches, Reconstruction::CVDescription& a, Reconstruction::CVDescription& b);
        Tango3DR_CameraCalibration GetCalibration(int scale);
        std::string GetEvent();
        bool InitTexturing(std::string input, bool fast);
        void PreviewChange(int frames);
        void RenderGL(glm::mat4 matrix);
        void Setup(double res, double dmin, double dmax, int noise, bool holesFilling,
                   bool poseCorrection, bool distortion, bool clearing, std::string dataset_path);
        void SetPhotoMode(bool on);
        void Start(ReconstructionThread thread);
        void Undo(bool fromUser, bool applyTextures);

    public:
        //objects
        Dataset* dataset;
        Retango depth;
        Scene scene;
        TangoScan scan;
        TangoTexturize texturize;
        //mutexes
        std::mutex binder_mutex_;
        std::mutex render_mutex_;
        std::mutex event_mutex_;
        pthread_t threadId;
        //frame data
        clock_t start;
        glm::mat4 frame_calibration;
        std::vector<float> frame_distortion;
        glm::mat4 frame_viewmat;
        Image* frame_image = 0;
        std::vector<glm::vec4> frame_points;
        std::vector<glm::mat4> frame_pose;
        double frame_timestamp;
        bool frame_sparse;
        bool holes_filling;
        bool photo_mode;
        bool pose_correction;
        bool t3dr_is_running_;
        //tracking
        std::string event_;
        bool jumped;
        bool paused;
        bool tracked;
        bool lost;
        char pose_feedback[1024];
        //pose correction
        Selector selector;
        GLRenderer* renderer;
        Image* rendered_depth = 0;
        Image* rendered_image = 0;
        Image* request_image = 0;
        float request_distance;
        bool request_newprojection;
        std::vector<glm::mat4> request_matrix;
        //scan preview
        long frame_index;
        std::unordered_map<GridIndex, int, GridIndexHasher> preview_start;
    };
}
#endif