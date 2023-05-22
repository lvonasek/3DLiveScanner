#include <exporter/csvposes.h>
#include <gl/camera.h>

namespace oc {

void ExporterCSVPoses::Process(Dataset* dataset, std::string output, bool rotated)
{    
    //init dataset
    int poseCount = GetPoseCount(dataset);
    float yaw = glm::radians(dataset->ReadYaw());
    float s = glm::sin(-yaw);
    float c = glm::cos(-yaw);

    //create CSV file
    FILE* csv = fopen(output.c_str(), "w");
    fprintf(csv, "%s\n", "frame_id,x,y,z,yaw,pitch,roll");

    //convert each frame
    for (int i = 0; i < poseCount; i++)
    {
        //get values
        GLCamera camera;
        glm::mat4 pose = dataset->ReadPose(i)[OPENGL_CAMERA];
        camera.SetTransformation(glm::inverse(pose));

        //apply compass rotation to position
        float x = camera.position.x;
        float z = camera.position.z;
        camera.position.x = x * s - z * c;
        camera.position.z = x * c + z * s;

        //apply compass rotation to rotation
        EulerAngles angles = camera.GetRotation();
        angles.yaw -= yaw;
        camera.SetRotation(angles);

        pose = camera.GetTransformation();
        if (rotated)
        {
            pose = glm::rotate(glm::mat4(1), glm::radians(90.0f), glm::vec3(0, 0, 1)) * pose;
            pose = glm::rotate(glm::mat4(1), glm::radians(90.0f), glm::vec3(0, 1, 0)) * pose;
            pose = glm::rotate(glm::mat4(1), glm::radians(90.0f), glm::vec3(0, 0, -1)) * pose;
        }
        glm::quat rotation = glm::normalize(glm::quat_cast(pose));

        //write data
        fprintf(csv, "%s,", GetId(i).c_str());
        fprintf(csv, "%f,%f,%f,", pose[3][0], pose[3][1], pose[3][2]);
        fprintf(csv, "%f,", glm::degrees(glm::yaw(rotation)));
        fprintf(csv, "%f,", glm::degrees(glm::pitch(rotation)));
        fprintf(csv, "%f\n", glm::degrees(glm::roll(rotation)));
    }

    fclose(csv);
}

void ExporterCSVPoses::Process(Mesh& pcl, std::vector<glm::mat4>& pose, int poseIndex)
{
    //unused - ConvertFrame was not called
}
}
