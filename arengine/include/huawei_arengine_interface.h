/**
 * Copyright 2022. Huawei Technologies Co., Ltd. All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef ARNDK_HWAR_INTERFACE_H
#define ARNDK_HWAR_INTERFACE_H

#include <string>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

using HwArConfig = struct HwArConfig_;
using HwArSession = struct HwArSession_;
using HwArPose = struct HwArPose_;
using HwArCamera = struct HwArCamera_;
using HwArCameraIntrinsics = struct HwArCameraIntrinsics_;
using HwArFrame = struct HwArFrame_;
using HwArLightEstimate = struct HwArLightEstimate_;
using HwArPointCloud = struct HwArPointCloud_;
using HwArImageMetadata = struct HwArImageMetadata_;
using HwArImage = struct HwArImage_;
using AImage = struct AImage;
using HwArTrackable = struct HwArTrackable_;
using HwArTrackableList = struct HwArTrackableList_;
using HwArPlane = struct HwArPlane_;
using HwArPoint = struct HwArPoint_;
using HwArAnchor = struct HwArAnchor_;
using HwArAnchorList = struct HwArAnchorList_;
using HwArHitResult = struct HwArHitResult_;
using HwArHitResultList = struct HwArHitResultList_;
using ACameraMetadata = struct ACameraMetadata;
using HwArCameraConfig = struct HwArCameraConfig;

using HwArHand = struct HwArHand_;
using HwArBody = struct HwArBody_;
using HwArFace = struct HwArFace_;
using HwArFacePose = struct HwArPose_;
using HwArFaceGeometry = struct HwArFaceGeometry_;
using HwArFaceBlendShapes = struct HwArFaceBlendShapes_;
using HwArObject = struct HwArObject_;
using HwArCameraInfo = struct HwArCameraInfo_;
using HwArAugmentedImage = struct HwArAugmentedImage_;
using HwArAugmentedImageDatabase = struct HWArAugmentedImageDatabase_;
using HwArSceneMesh = struct HwArSceneMesh_;
using HwArTarget = struct HwArTarget_;

#ifdef __cplusplus

/*
 * Upcasts to ArTrackable
 */
inline HwArTrackable *HwArAsTrackable(HwArPlane *plane)
{
    return reinterpret_cast<HwArTrackable *>(plane);
}

inline HwArTrackable *HwArAsTrackable(HwArPoint *point)
{
    return reinterpret_cast<HwArTrackable *>(point);
}

inline HwArTrackable *ArAsTrackable(HwArAugmentedImage *augmentedImage)
{
    return reinterpret_cast<HwArTrackable *>(augmentedImage);
}

/*
 * downcasts to ArXXX
 */
inline HwArPlane *HwArAsPlane(HwArTrackable *trackable)
{
    return reinterpret_cast<HwArPlane *>(trackable);
}

inline HwArPoint *HwArAsPoint(HwArTrackable *trackable)
{
    return reinterpret_cast<HwArPoint *>(trackable);
}

inline HwArAugmentedImage *ArAsAugmentedImage(HwArTrackable *trackable)
{
    return reinterpret_cast<HwArAugmentedImage *>(trackable);
}

#endif

#if __cplusplus >= 201100
#define HWAR_DEFINE_ENUM(_type) enum _type : int32_t
#else
#define HWAR_DEFINE_ENUM(_type) \
    typedef int32_t _type; \
    enum
#endif

HWAR_DEFINE_ENUM(HwArEnginesType) {
    HWAR_NONE = 0,
    HWAR_ENGINE = 1,
};

HWAR_DEFINE_ENUM(HwArEnginesAvaliblity) {
    HWAR_NONE_ENGINES_SUPPORTED = 0,
    HWAR_ENGINE_SUPPORTED = 1,
};

HWAR_DEFINE_ENUM(HwArTrackableType) {
    HWAR_TRACKABLE_BASE_TRACKABLE = 0x41520100,
    HWAR_TRACKABLE_PLANE = 0x41520101,
    HWAR_TRACKABLE_POINT = 0x41520102,
    HWAR_TRACKABLE_AUGMENTED_IMAGE = 0x41520104,
    HWAR_TRACKABLE_HAND = 0x50000000,
    HWAR_TRACKABLE_BODY = 0x50000001,
    HWAR_TRACKABLE_FACE = 0x50000002,
    HWAR_TRACKABLE_TARGET_DETECTION = 0x50000003,
    HWAR_TRACKABLE_OBJECT = 0x50000005,
    HWAR_TRACKABLE_TARGET = 0x50000008,
    HWAR_TRACKABLE_NOT_VALID = 0
};

HWAR_DEFINE_ENUM(HwArStatus) {
    HWAR_SUCCESS = 0,
    HWAR_ERROR_INVALID_ARGUMENT = -1,
    HWAR_ERROR_FATAL = -2,
    HWAR_ERROR_SESSION_PAUSED = -3,
    HWAR_ERROR_SESSION_NOT_PAUSED = -4,
    HWAR_ERROR_NOT_TRACKING = -5,
    HWAR_ERROR_TEXTURE_NOT_SET = -6,
    HWAR_ERROR_MISSING_GL_CONTEXT = -7,
    HWAR_ERROR_UNSUPPORTED_CONFIGURATION = -8,
    HWAR_ERROR_CAMERA_PERMISSION_NOT_GRANTED = -9,
    HWAR_ERROR_DEADLINE_EXCEEDED = -10,
    HWAR_ERROR_RESOURCE_EXHAUSTED = -11,
    HWAR_ERROR_NOT_YET_AVAILABLE = -12,
    HWAR_ERROR_CAMERA_NOT_AVAILABLE = -13,
    HWAR_UNAVAILABLE_ARSERVICE_NOT_INSTALLED = -100,
    HWAR_UNAVAILABLE_DEVICE_NOT_COMPATIBLE = -101,
    HWAR_UNAVAILABLE_APK_TOO_OLD = -103,
    HWAR_UNAVAILABLE_SDK_TOO_OLD = -104,
    HWAR_UNAVAILABLE_USER_DECLINED_INSTALLATION = -105,
    HWAR_UNAVAILABLE_EMUI_NOT_COMPATIBLE = -1000,
    HWAR_UNAVAILABLE_CONNECT_SERVER_TIME_OUT = -1001
};

HWAR_DEFINE_ENUM(HwArTrackingState) {
    HWAR_TRACKING_STATE_TRACKING = 0,
    HWAR_TRACKING_STATE_PAUSED = 1,
    HWAR_TRACKING_STATE_STOPPED = 2
};

HWAR_DEFINE_ENUM(HwArAvailability) {
    HWAR_AVAILABILITY_UNKNOWN_ERROR = 0,
    HWAR_AVAILABILITY_UNKNOWN_CHECKING = 1,
    HWAR_AVAILABILITY_UNKNOWN_TIMED_OUT = 2,
    HWAR_AVAILABILITY_UNSUPPORTED_DEVICE_NOT_CAPABLE = 100,
    HWAR_AVAILABILITY_SUPPORTED_NOT_INSTALLED = 201,
    HWAR_AVAILABILITY_SUPPORTED_APK_TOO_OLD = 202,
    HWAR_AVAILABILITY_SUPPORTED_INSTALLED = 203,
    HWAR_AVAILABILITY_UNSUPPORTED_EMUI_NOT_CAPABLE = 5000
};

HWAR_DEFINE_ENUM(HwArInstallStatus) {
    HWAR_INSTALL_STATUS_INSTALLED = 0,
    HWAR_INSTALL_STATUS_INSTALL_REQUESTED = 1
};

HWAR_DEFINE_ENUM(HwArInstallBehavior) {
    HWAR_INSTALL_BEHAVIOR_REQUIRED = 0,
    HWAR_INSTALL_BEHAVIOR_OPTIONAL = 1
};

HWAR_DEFINE_ENUM(HwArInstallUserMessageType) {
    HWAR_INSTALL_USER_MESSAGE_TYPE_APPLICATION = 0,
    HWAR_INSTALL_USER_MESSAGE_TYPE_FEATURE = 1,
    HWAR_INSTALL_USER_MESSAGE_TYPE_USER_ALREADY_INFORMED = 2
};

HWAR_DEFINE_ENUM(HwArPlaneFindingMode) {
    HWAR_PLANE_FINDING_MODE_DISABLED = 0,
    HWAR_PLANE_FINDING_MODE_HORIZONTAL = 1,
    HWAR_PLANE_FINDING_MODE_VERTICAL = 2,
    HWAR_PLANE_FINDING_MODE_HORIZONTAL_AND_VERTICAL = 3
};

HWAR_DEFINE_ENUM(HwArUpdateMode) {
    HWAR_UPDATE_MODE_BLOCKING = 0,
    HWAR_UPDATE_MODE_LATEST_CAMERA_IMAGE = 1
};

HWAR_DEFINE_ENUM(HwArPowerMode) {
    HWAR_POWER_MODE_NORMAL = 0,
    HWAR_POWER_MODE_POWER_SAVING = 1,
    HWAR_POWER_MODE_ULTRA_POWER_SAVING = 2,
    HWAR_POWER_MODE_PERFORMANCE_FIRST = 3
};

HWAR_DEFINE_ENUM(HwArFocusMode) {
    HWAR_FOCUS_MODE_FIXED = 0,
    HWAR_FOCUS_MODE_AUTO = 1
};

HWAR_DEFINE_ENUM(HwArEnableItem) {
    HWAR_ENABLE_NULL = 0,
    HWAR_ENABLE_DEPTH = 1 << 0,
    HWAR_ENABLE_MASK = 1 << 1,
    HWAR_ENABLE_SCENE_MESH = 1 << 2,
    HWAR_ENABLE_CLOUD_AUGMENTED_IMAGE = 1 << 5,
    HWAR_ENABLE_HEALTH_DEVICE = 1 << 6,
    HWAR_ENABLE_FLASH_MODE_TORCH = 1 << 7,
    HWAR_ENABLE_CLOUD_OBJECT_RECOGNITION = 1 << 10,
};

HWAR_DEFINE_ENUM(HwArPlaneType) {
    HWAR_PLANE_HORIZONTAL_UPWARD_FACING = 0,
    HWAR_PLANE_HORIZONTAL_DOWNWARD_FACING = 1,
    HWAR_PLANE_VERTICAL_FACING = 2,
    HWAR_PLANE_UNKNOWN_FACING = 3
};

HWAR_DEFINE_ENUM(HwArLightEstimateState) {
    HWAR_LIGHT_ESTIMATE_STATE_NOT_VALID = 0,
    HWAR_LIGHT_ESTIMATE_STATE_VALID = 1
};

HWAR_DEFINE_ENUM(HwArType) {
    HWAR_TYPE_WORLD = 0x1,
    HWAR_TYPE_BODY = 0x2,
    HWAR_TYPE_HAND = 0x4,
    HWAR_TYPE_FACE = 0x10,
    HWAR_TYPE_IMAGE = 0x80,
    HWAR_TYPE_NONE = 0x8000
};

HWAR_DEFINE_ENUM(HwArCameraLensFacing) {
    HWAR_CAMERA_FACING_REAR = 0,
    HWAR_CAMERA_FACING_FRONT = 1
};

HWAR_DEFINE_ENUM(HwArHandFindingMode) {
    HwAr_HAND_FINDING_MODE_DISABLED = 0x0,
    HwAr_HAND_FINDING_MODE_2D_ENABLED = 0x1,
    HwAr_HAND_FINDING_MODE_3D_ENABLED = 0x2
};

HWAR_DEFINE_ENUM(HwArSemanticMode) {
    HwAr_SEMANTIC_NONE = 0,
    HwAr_SEMANTIC_PLANE = 1,
    HwAr_SEMANTIC_TARGET = 2,
};

HWAR_DEFINE_ENUM(HwArLightEstimationMode) {
    HwAr_LIGHT_ESTIMATION_MODE_DISABLED = 0,
    HwAr_LIGHT_ESTIMATION_MODE_AMBIENT_INTENSITY = 1,
    HwAr_LIGHT_ESTIMATION_MODE_ENVIRONMENT_LIGHTING = 2,
    HwAr_LIGHT_ESTIMATION_MODE_ENVIRONMENT_TEXTURE = 4,
};

HWAR_DEFINE_ENUM(HwArFaceDetectMode) {
    HwAr_HEALTH_ENABLE_HEART_RATE = 0x01,
    HwAr_HEALTH_ENABLE_BREATH_RATE = 0x02,
    HwAr_HEALTH_ENABLE_LIVE_DETECTION = 0x04,
    HwAr_HEALTH_ENABLE_DEFAULT = HwAr_HEALTH_ENABLE_HEART_RATE |
        HwAr_HEALTH_ENABLE_BREATH_RATE |
        HwAr_HEALTH_ENABLE_LIVE_DETECTION,
    HwAr_HEALTH_ENABLE_SP02 = 0x08,
    HwAr_HEALTH_ENABLE_STRESS = 0x10,
    HwAr_FACE_ENABLE_MESH = 0x100,
    HwAr_FACE_ENABLE_BLEND_SHAPE = 0x200,
    HwAr_FACE_ENABLE_DEFAULT = HwAr_FACE_ENABLE_MESH | HwAr_FACE_ENABLE_BLEND_SHAPE,
    HwAr_FACE_ENABLE_MULTIFACE = 0x800,
};

HWAR_DEFINE_ENUM(HwArPointOrientationMode) {
    HWAR_POINT_ORIENTATION_INITIALIZED_TO_IDENTITY = 0,
    HWAR_POINT_ORIENTATION_ESTIMATED_SURFACE_NORMAL = 1
};

HWAR_DEFINE_ENUM(HwAr_Pose_Type) {
    HWAR_POSE_TYPE_IDENTITY = 0,
    HWAR_POSE_TYPE_ROTATE_Z_90,
    HWAR_POSE_TYPE_ROTATE_Z_180,
    HWAR_POSE_TYPE_ROTATE_Z_270,
    HWAR_POSE_TYPE_TYPE_MAX
};

HWAR_DEFINE_ENUM(HwArNativeWindowType) {
    HWAR_PREVIEW = 0,
    HWAR_VGA = 1,
    HWAR_METADATA = 2,
    HWAR_DEPTH = 4
};

HWAR_DEFINE_ENUM(HwArImageInputMode) {
    NON_INPUT = 0,
    EXTERNAL_INPUT_ALL = -1
};

HWAR_DEFINE_ENUM(HwArWorldMappingState) {
    HWAR_WORLD_MAPPING_NOT_AVAILABLE = -1,
    HWAR_WORLD_MAPPING_LIMITED = 0,
    HWAR_WORLD_MAPPING_EXTENDING,
    HWAR_WORLD_MAPPING_MAPPED
};

HWAR_DEFINE_ENUM(HwArAlignState) {
    HWAR_ALIGN_NONE = 0,
    HWAR_ALIGN_FAILED,
    HWAR_ALIGN_PROCESSING,
    HWAR_ALIGN_SUCCESS
};

HWAR_DEFINE_ENUM(HwArAnimojiBlendShape) {
    HwAr_Animoji_Eye_Blink_Left = 0,
    HwAr_Animoji_Eye_Look_Down_Left = 1,
    HwAr_Animoji_Eye_Look_In_Left = 2,
    HwAr_Animoji_Eye_Look_Out_Left = 3,
    HwAr_Animoji_Eye_Look_Up_Left = 4,
    HwAr_Animoji_Eye_Squint_Left = 5,
    HwAr_Animoji_Eye_Wide_Left = 6,
    HwAr_Animoji_Eye_Blink_Right = 7,
    HwAr_Animoji_Eye_Look_Down_Right = 8,
    HwAr_Animoji_Eye_Look_In_Right = 9,
    HwAr_Animoji_Eye_Look_Out_Right = 10,
    HwAr_Animoji_Eye_Look_Up_Right = 11,
    HwAr_Animoji_Eye_Squint_Right = 12,
    HwAr_Animoji_Eye_Wide_Right = 13,
    HwAr_Animoji_Jaw_Forward = 14,
    HwAr_Animoji_Jaw_Left = 15,
    HwAr_Animoji_Jaw_Right = 16,
    HwAr_Animoji_Jaw_Open = 17,
    HwAr_Animoji_Mouth_Funnel = 18,
    HwAr_Animoji_Mouth_Pucker = 19,
    HwAr_Animoji_Mouth_Left = 20,
    HwAr_Animoji_Mouth_Right = 21,
    HwAr_Animoji_Mouth_Smile_Left = 22,
    HwAr_Animoji_Mouth_Smile_Right = 23,
    HwAr_Animoji_Mouth_Frown_Left = 24,
    HwAr_Animoji_Mouth_Frown_Right = 25,
    HwAr_Animoji_Mouth_Dimple_Left = 26,
    HwAr_Animoji_Mouth_Dimple_Right = 27,
    HwAr_Animoji_Mouth_Stretch_Left = 28,
    HwAr_Animoji_Mouth_Stretch_Right = 29,
    HwAr_Animoji_Mouth_Roll_Lower = 30,
    HwAr_Animoji_Mouth_Roll_Upper = 31,
    HwAr_Animoji_Mouth_Shrug_Lower = 32,
    HwAr_Animoji_Mouth_Shrug_Upper = 33,
    HwAr_Animoji_Mouth_Upper_Up = 34,
    HwAr_Animoji_Mouth_Lower_Down = 35,
    HwAr_Animoji_Mouth_Lower_Out = 36,
    HwAr_Animoji_Brow_Down_Left = 37,
    HwAr_Animoji_Brow_Down_Right = 38,
    HwAr_Animoji_Brow_Inner_Up = 39,
    HwAr_Animoji_Brow_Outer_Up_Left = 40,
    HwAr_Animoji_Brow_Outer_Up_Right = 41,
    HwAr_Animoji_Cheek_Puff = 42,
    HwAr_Animoji_Cheek_Squint_Left = 43,
    HwAr_Animoji_Cheek_Squint_Right = 44,
    HwAr_Animoji_Frown_Nose_Mouth_Up = 45,
    HwAr_Animoji_Tongue_In = 46,
    HwAr_Animoji_Tongue_Out_Slight = 47,
    HwAr_Animoji_Tongue_Left = 48,
    HwAr_Animoji_Tongue_Right = 49,
    HwAr_Animoji_Tongue_Up = 50,
    HwAr_Animoji_Tongue_Down = 51,
    HwAr_Animoji_Tongue_Left_Up = 52,
    HwAr_Animoji_Tongue_Left_Down = 53,
    HwAr_Animoji_Tongue_Right_Up = 54,
    HwAr_Animoji_Tongue_Right_Down = 55,
    HwAr_Animoji_Left_Eyeball_Left = 56,
    HwAr_Animoji_Left_Eyeball_Right = 57,
    HwAr_Animoji_Left_Eyeball_Up = 58,
    HwAr_Animoji_Left_Eyeball_Down = 59,
    HwAr_Animoji_Right_Eyeball_Left = 60,
    HwAr_Animoji_Right_Eyeball_Right = 61,
    HwAr_Animoji_Right_Eyeball_Up = 62,
    HwAr_Animoji_Right_Eyeball_Down = 63,
    HwAr_ANIMOJI_BLENDSHAPE_LENGTH = 64
};

/**
 * label: -1=non_face, 0=face_other, 1=lower_lip, 2=upper_lip, 3=left_eye, 4=right_eye,
 * 5=left_brow, 6=right_brow,7=brow_center, 8=nose
 */
HWAR_DEFINE_ENUM(HwArAnimojiTriangleLabel) {
    HwAr_Animoji_Label_non_face = -1,
    HwAr_Animoji_Label_face_other = 0,
    HwAr_Animoji_Label_lower_lip = 1,
    HwAr_Animoji_Label_upper_lip = 2,
    HwAr_Animoji_Label_left_eye = 3,
    HwAr_Animoji_Label_right_eye = 4,
    HwAr_Animoji_Label_left_brow = 5,
    HwAr_Animoji_Label_right_brow = 6,
    HwAr_Animoji_Label_brow_center = 7,
    HwAr_Animoji_Label_nose = 8,
    HwAr_ANIMOJI_TRIANGLE_LABELS_LENGTH
};

HWAR_DEFINE_ENUM(HwArCoordinateSystemType) {
    HwAr_Coordinate_System_Type_Unknown = -1,
    HwAr_Coordinate_System_Type_3D_World, // world coordinate system
    HwAr_Coordinate_System_Type_3D_Self, // 3d image coordinate system for bodypose
    HwAr_Coordinate_System_Type_2D_Image, // 2D image coordinate system for gesture (openGL)
    HwAr_Coordinate_System_Type_3D_Camera
};

HWAR_DEFINE_ENUM(HwArBodySkeletonType) {
    HwAr_BodySkeleton_Head = 0,
    HwAr_BodySkeleton_Neck = 1,
    HwAr_BodySkeleton_r_Sho = 2,
    HwAr_BodySkeleton_r_Elbow = 3,
    HwAr_BodySkeleton_r_Wrist = 4,
    HwAr_BodySkeleton_l_Sho = 5,
    HwAr_BodySkeleton_l_Elbow = 6,
    HwAr_BodySkeleton_l_Wrist = 7,
    HwAr_BodySkeleton_r_Hip = 8,
    HwAr_BodySkeleton_r_Knee = 9,
    HwAr_BodySkeleton_r_Ankle = 10,
    HwAr_BodySkeleton_l_Hip = 11,
    HwAr_BodySkeleton_l_Knee = 12,
    HwAr_BodySkeleton_l_Ankle = 13,
    HwAr_BodySkeleton_Hip_mid = 14,
    HwAr_BodySkeleton_r_ear = 15,
    HwAr_BodySkeleton_r_eye = 16,
    HwAr_BodySkeleton_nose = 17,
    HwAr_BodySkeleton_l_eye = 18,
    HwAr_BodySkeleton_l_ear = 19,
    HwAr_BodySkeleton_spine = 20,
    HwAr_BodySkeleton_r_toe = 21,
    HwAr_BodySkeleton_l_toe = 22,
    HwAr_BodySkeleton_Length
};

HWAR_DEFINE_ENUM(HwArBodyAction) {
    HwAr_Body_Action_None = 0,
    HwAr_Body_Action_1,
    HwAr_Body_Action_2,
    HwAr_Body_Action_3,
    HwAr_Body_Action_4,
    HwAr_Body_Action_5,
    HwAr_Body_Action_6
};

HWAR_DEFINE_ENUM(HwArHandType) {
    HwAr_Hand_Unknown = -1,
    HwAr_Hand_Right,
    HwAr_Hand_Left
};

HWAR_DEFINE_ENUM(HwArHandGesutreType) {
    HwAr_GESTURE_HAND_UNKNOWN = -1,
    HwAr_GESTURE_HAND_0 = 0, // in use
    HwAr_GESTURE_HAND_1, // in use
    HwAr_GESTURE_HAND_2,
    HwAr_GESTURE_HAND_3,
    HwAr_GESTURE_HAND_4,
    HwAr_GESTURE_HAND_5, // in use
    HwAr_GESTURE_HAND_6, // in use
    HwAr_GESTURE_HAND_7, // in use
    HwAr_GESTURE_HAND_8,
    HwAr_GESTURE_HAND_9,
    HwAr_GESTURE_HAND_10, // in use
    HwAr_GESTURE_HAND_11,
    HwAr_GESTURE_HAND_Length = 13
};

HWAR_DEFINE_ENUM(HwArHandSkeletonType) {
    HwAr_HandSkeleton_Root = 0,
    HwAr_HandSkeleton_Pinky_1,
    HwAr_HandSkeleton_Pinky_2,
    HwAr_HandSkeleton_Pinky_3,
    HwAr_HandSkeleton_Pinky_4,
    HwAr_HandSkeleton_Ring_1,
    HwAr_HandSkeleton_Ring_2,
    HwAr_HandSkeleton_Ring_3,
    HwAr_HandSkeleton_Ring_4,
    HwAr_HandSkeleton_Middle_1,
    HwAr_HandSkeleton_Middle_2,
    HwAr_HandSkeleton_Middle_3,
    HwAr_HandSkeleton_Middle_4,
    HwAr_HandSkeleton_Index_1,
    HwAr_HandSkeleton_Index_2,
    HwAr_HandSkeleton_Index_3,
    HwAr_HandSkeleton_Index_4,
    HwAr_HandSkeleton_Thumb_1,
    HwAr_HandSkeleton_Thumb_2,
    HwAr_HandSkeleton_Thumb_3,
    HwAr_HandSkeleton_Thumb_4,
    HwAr_HandSkeleton_Length = 21
};

HWAR_DEFINE_ENUM(HwArBodyParameterType) {
    HwAr_BODY_PARAMETER_HEIGHT = 0,
    HwAR_BODY_PARAMETER_LEG_LENGTH = 1,
    HwAR_BODY_PARAMETER_SHOULDER_LENGTH = 2,
    HwAR_BODY_PARAMETER_ARM_LENGTH = 3,
    HwAR_BODY_PARAMETER_UNKNOWN = 255,
};

HWAR_DEFINE_ENUM(HwArSemanticPlaneLabel) {
    HwAr_PLANE_OTHER = 0,
    HwAr_PLANE_WALL,
    HwAr_PLANE_FLOOR,
    HwAr_PLANE_SEAT,
    HwAr_PLANE_TABLE,
    HwAr_PLANE_CEILING,
    HwAr_PLANE_DOOR,
    HwAr_PLANE_WINDOW,
    HwAr_PLANE_BED
};

HWAR_DEFINE_ENUM(HwArCloudAnchorState) {
    HWAR_CLOUD_ANCHOR_STATE_NONE = 0,
    HWAR_CLOUD_ANCHOR_STATE_TASK_IN_PROGRESS = 1,
    HWAR_CLOUD_ANCHOR_STATE_SUCCESS = 2,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_INTERNAL = -1,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_NOT_AUTHORIZED = -2,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_SERVICE_UNAVAILABLE = -3,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_RESOURCE_EXHAUSTED = -4,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_HOSTING_DATASET_PROCESSING_FAILED = -5,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_CLOUD_ID_NOT_FOUND = -6,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_RESOLVING_LOCALIZATION_NO_MATCH = -7,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_RESOLVING_SDK_VERSION_TOO_OLD = -8,
    HWAR_CLOUD_ANCHOR_STATE_ERROR_RESOLVING_SDK_VERSION_TOO_NEW = -9,
};

HWAR_DEFINE_ENUM(HwArHealthParameterType) {
    HWAR_PARAMETER_INVILID = 0,
    HWAR_PARAMETER_HEART_RATE = 1,
    HWAR_PARAMETER_HEART_RATE_SNR = 2,
    HWAR_PARAMETER_HEART_RATE_CONFIDENCE = 3,
    HWAR_PARAMETER_BREATH_RATE = 4,
    HWAR_PARAMETER_BREATH_RATE_SNR = 5,
    HWAR_PARAMETER_BREATH_RATE_CONFIDENCE = 6,
    HWAR_PARAMETER_FACE_AGE = 7,
    HWAR_PARAMETER_GENDER_MALE_WEIGHT = 8,
    HWAR_PARAMETER_GENDER_FEMALE_WEIGHT = 9,
    HEAR_PARAMETER_FACE_HEALTH_STATUS = 13,
    HEAR_PARAMETER_FACE_HEALTH_PROC_PROGRESS = 14,
    HEAR_PARAMETER_HEART_WAVE = 15,
    HEAR_PARAMETER_SPO2_VALUE = 16,
};

HWAR_DEFINE_ENUM(HwArFaceHealthCheckState) {
    HWAR_DETECT_FAILED = -1,
    HWAR_DETECT_SUCCESS = 0,
    HWAR_NO_AVAILABLE_HEALTH_DATA = 1,
    HWAR_FACE_WITH_EXPRESSION = 10,
    HWAR_IMAGE_SIZE_WRONG = 20,
    HWAR_FACE_NOT_IN_ELLIPSE = 21,
    HWAR_FACE_MOTION_TOO_MUCH = 22,
    HWAR_EFFECTIVE_PIXEEL_TOO_LOW = 23,
    HWAR_LIGHT_TOO_DARK = 24,
    HWAR_LIGHT_NOT_UNIFORM = 25,
    HWAR_POSE_TOO_LARGE = 26,
    HWAR_SIGNAL_CAPTURE_FAILED = 27,
    HWAR_SIGNAL_NAN = 28,
    HWAR_FINGER_OUTSIDE_CAMERA = 29,
    HWAR_FINGER_SIGNAL_UNAVAILABLE = 30,
    HWAR_FRAUD_FACE = 31,
};

HWAR_DEFINE_ENUM(HwArServiceState) {
    HWAR_CLOUD_SERVICE_NORMAL = 10000,
    HWAR_CLOUD_SERVICE_ERROR_NETWORK_UNAVAILABLE = 10001,
    HWAR_CLOUD_SERVICE_ERROR_CLOUD_SERVICE_UNAVAILABLE = 10002,
    HWAR_CLOUD_SERVICE_ERROR_NOT_AUTHORIZED = 10003,
    HWAR_CLOUD_SERVICE_ERROR_SERVER_VERSION_TOO_OLD = 10004,
    HWAR_CLOUD_SERVICE_ERROR_TIME_EXHAUSTED = 10005,
    HWAR_CLOUD_SERVICE_ERROR_INTERNAL = 10006,
    HWAR_CLOUD_IMAGE_ERROR_IMAGE_GALLERY_INVALID = 20001,
    HWAR_CLOUD_IMAGE_ERROR_IMAGE_RECOGNIZE_FAILE = 20002,
    HWAR_CLOUD_OBJECT_ERROR_OBJECT_MODEL_INVALID = 30001,
    HWAR_CLOUD_OBJECT_ERROR_OBJECT_RECOGNIZE_FAILE = 30002,
};

HWAR_DEFINE_ENUM(HwArTargetLabel) {
    HwAr_TARGET_INVALID = -1,
    HwAr_TARGET_OTHER = 0,
    HwAr_TARGET_SEAT = 1,
    HwAr_TARGET_TABLE = 2,
};

HWAR_DEFINE_ENUM(HwArTargetShapeType) {
    HwAr_TARGET_SHAPE_INVALID = -1,
    HwAr_TARGET_SHAPE_OTHER = 0,
    HwAr_TARGET_SHAPE_CUBE = 1,
    HwAr_TARGET_SHAPE_CIRCLE = 2,
    HwAr_TARGET_SHAPE_RECT = 3,
};

HWAR_DEFINE_ENUM(HwArLightShadowType) {
    HWAR_LIGHT_SHADOW_TYPE_NONE = 0,
    HWAR_LIGHT_SHADOW_TYPE_HARD = 1,
    HWAR_LIGHT_SHADOW_TYPE_SOFT = 2,
};

HWAR_DEFINE_ENUM(HwArEnvironmentTextureUpdateMode) {
    HWAR_LIGHT_ENVIRONMENT_TEXTURE_UPDATE_UNKNOWN = -1,
    HWAR_LIGHT_ENVIRONMENT_TEXTURE_UPDATE_AUTO = 0,
    HWAR_LIGHT_ENVIRONMENT_TEXTURE_UPDATE_ONE_TIME = 1,
};

HWAR_DEFINE_ENUM(HwArEvent) {
    HWAR_EVENT_UNKNOWN = -1,
    HWAR_EVENT_CLOUD_SERVICE_STATE = 0,
    HWAR_EVENT_SERVICE_INVOKE_STATUS = 100,
};

#undef HWAR_DEFINE_ENUM

const int MAX_WINDOWS_COUNT = 64;
const int DISTORTION_COUNT = 5;

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------------------
 detailed functions added for AR Engine
 ----------------------------------------------------------------------------------------*/
// deprecated
HwArEnginesAvaliblity HwArEnginesSelector_checkAllAvailableEngines(
    void *env,
    void *applicationContext);

// deprecated
HwArEnginesType HwArEnginesSelector_setAREngine(HwArEnginesType arOptions);

// deprecated
HwArEnginesType HwArEnginesSelector_getCreatedEngine();

void HwArEnginesApk_checkAvailability(void *env,
                                      void *applicationContext,
                                      HwArAvailability *outAvailability);

// deprecated
HwArStatus HwArEnginesApk_requestInstall(void *env,
                                         void *applicationActivity,
                                         bool userRequestedInstall,
                                         HwArInstallStatus *outInstallStatus);

void HwArConfig_getArType(const HwArSession *session,
                          const HwArConfig *config,
                          HwArType *type);

void HwArConfig_setArType(const HwArSession *session,
                          HwArConfig *config,
                          HwArType type);

void HwArConfig_getCameraLensFacing(const HwArSession *session,
                                    const HwArConfig *config,
                                    HwArCameraLensFacing *facing);

void HwArConfig_setCameraLensFacing(const HwArSession *session,
                                    HwArConfig *config,
                                    HwArCameraLensFacing facing);

void HwArCameraConfig_create(const HwArSession *session,
                             HwArCameraConfig **outCameraConfig);

void HwArCameraConfig_destroy(HwArCameraConfig *cameraConfig);

void HwArCameraConfig_getImageDimensions(const HwArSession *session,
                                         const HwArCameraConfig *cameraConfig,
                                         int32_t *outWidth,
                                         int32_t *outHeight);

void HwArCameraConfig_getTextureDimensions(const HwArSession *session,
                                           const HwArCameraConfig *cameraConfig,
                                           int32_t *outWidth,
                                           int32_t *outHeight);

void HwArSession_getCameraConfig(const HwArSession *session,
                                 HwArCameraConfig *outCameraConfig);

void HwArSession_setEnvironmentTextureProbe(const HwArSession *session, const float *boundBox);

void HwArSession_stop(HwArSession *session);

// AugmentedImage config----------------------------------------------------------------
void HwArConfig_setAugmentedImageDatabase(const HwArSession *session,
                                          HwArConfig *config,
                                          const HwArAugmentedImageDatabase *augmentedImageDatabase);

void HwArConfig_getAugmentedImageDatabase(const HwArSession *session,
                                          const HwArConfig *config,
                                          HwArAugmentedImageDatabase *outAugmentedImageDatabase);

// HwArAugmentedImage--------------------------------------------------------------------
void HwArAugmentedImage_getCenterPose(const HwArSession *session,
                                      const HwArAugmentedImage *augmentedImage,
                                      HwArPose *outPose);

void HwArAugmentedImage_getExtentX(const HwArSession *session,
                                   const HwArAugmentedImage *augmentedImage,
                                   float *outExtentX);

void HwArAugmentedImage_getExtentZ(const HwArSession *session,
                                   const HwArAugmentedImage *augmentedImage,
                                   float *outExtentZ);

void HwArAugmentedImage_getIndex(const HwArSession *session,
                                 const HwArAugmentedImage *augmentedImage,
                                 int32_t *outIndex);

void HwArAugmentedImage_acquireName(const HwArSession *session,
                                    const HwArAugmentedImage *augmentedImage,
                                    char **outAugmentedImageName);

void HwArAugmentedImage_getCloudImageId(const HwArSession *session,
                                        const HwArAugmentedImage *augmentedImage,
                                        char **outCloudImageId);

void HwArAugmentedImage_getCloudImageMetadata(const HwArSession *session,
                                              const HwArAugmentedImage *augmentedImage,
                                              char **outCloudImageMetadata);

// HwArAugmentedImageDatabase--------------------------------------------------------------------
void HwArAugmentedImageDatabase_create(const HwArSession *session,
                                       HwArAugmentedImageDatabase **outAugmentedImageDatabase);

HwArStatus HwArAugmentedImageDatabase_deserialize(const HwArSession *session,
                                                  const uint8_t *databaseRawBytes,
                                                  int64_t databaseRawBytesSize,
                                                  HwArAugmentedImageDatabase **outAugmentedImageDatabase);

void HwArAugmentedImageDatabase_serialize(const HwArSession *session,
                                          const HwArAugmentedImageDatabase *augmentedImageDatabase,
                                          uint8_t **outImageDatabaseRawBytes,
                                          int64_t *outImageDatabaseRawBytesSize);

HwArStatus HwArAugmentedImageDatabase_addImage(
    const HwArSession *session,
    HwArAugmentedImageDatabase *augmentedImageDatabase,
    const char *imageName,
    const uint8_t *imageGrayscalePixels,
    int32_t imageWidthInPixels,
    int32_t imageHeightInPixels,
    int32_t imageStrideInPixels,
    int32_t *outIndex);

HwArStatus HwArAugmentedImageDatabase_addImageWithPhysicalSize(
    const HwArSession *session,
    HwArAugmentedImageDatabase *augmentedImageDatabase,
    const char *imageName,
    const uint8_t *imageGrayscalePixels,
    int32_t imageWidthInPixels,
    int32_t imageHeightInPixels,
    int32_t imageStrideInPixels,
    float imageWidthInMeters,
    int32_t *outIndex);

void HwArAugmentedImageDatabase_getNumImages(const HwArSession *session,
                                             const HwArAugmentedImageDatabase *augmentedImageDatabase,
                                             int32_t *outNumImages);

void HwArAugmentedImageDatabase_destroy(HwArAugmentedImageDatabase *augmentedImageDatabase);

// HwArHand --------------------------------------------------------------------
void HwArHand_getGestureCoordinateSystem(const HwArSession *session,
                                         const HwArHand *hand,
                                         HwArCoordinateSystemType *outGestureCoordinateSystem);

void HwArHand_getHandId(const HwArSession *session,
                        const HwArHand *hand,
                        int32_t *outHandId);

void HwArHand_getGestureType(const HwArSession *session,
                             const HwArHand *hand,
                             HwArHandGesutreType *outGestureType);

void HwArHand_getHandType(const HwArSession *session,
                          const HwArHand *hand,
                          HwArHandType *outHandType);

void HwArHand_getGestureHandBox(const HwArSession *session,
                                const HwArHand *hand,
                                const float **outHandBox);

void HwArHand_getGestureCenter(const HwArSession *session,
                               const HwArHand *hand,
                               const float **outCenter);

void HwArHand_getGestureActionSize(const HwArSession *session,
                                   const HwArHand *hand,
                                   int32_t *outCount);

void HwArHand_getGestureAction(const HwArSession *session,
                               const HwArHand *hand,
                               const int32_t **outAction);

void HwArHand_getGestureOrientation(const HwArSession *session,
                                    const HwArHand *hand,
                                    const float **outOrientation);

// Hand Skeleton--------------------------------------------------------
void HwArHand_getSkeletonCoordinateSystem(const HwArSession *session,
                                          const HwArHand *hand,
                                          HwArCoordinateSystemType *outSkeletonCoordinateSystem);

void HwArHand_getHandSkeletonType(const HwArSession *session,
                                  const HwArHand *hand,
                                  const HwArHandSkeletonType **outHandSkeletonType);

void HwArHand_getHandSkeletonCount(const HwArSession *session,
                                   const HwArHand *hand,
                                   int32_t *outSkeletonCount);

void HwArHand_getHandSkeletonArray(const HwArSession *session,
                                   const HwArHand *hand,
                                   const float **outSkeletonArray);

void HwArHand_getHandSkeletonConnectionSize(const HwArSession *session,
                                            const HwArHand *hand,
                                            int32_t *outConnectionSize);

void HwArHand_getHandSkeletonConnection(const HwArSession *session,
                                        const HwArHand *hand,
                                        const int32_t **outConnection);

// HwArBody --------------------------------------------------------------------
void HwArBody_getCoordinateSystemType(const HwArSession *session,
                                      const HwArBody *body,
                                      HwArCoordinateSystemType *outCoordinateSystemType);

void HwArBody_getBodyId(const HwArSession *session,
                        const HwArBody *body,
                        int32_t *outPersonId);

void HwArBody_getSkeletonPointCount(const HwArSession *session,
                                    const HwArBody *body,
                                    int32_t *outPointCount);

void HwArBody_getSkeletonTypes(const HwArSession *session,
                               const HwArBody *body,
                               const HwArBodySkeletonType **outSkeletonTypes);

void HwArBody_getSkeletonPointIsExist2D(const HwArSession *session,
                                        const HwArBody *body,
                                        const int32_t **outSkeletonPointIsExist2D);

void HwArBody_getSkeletonPointIsExist3D(const HwArSession *session,
                                        const HwArBody *body,
                                        const int32_t **outSkeletonPointIsExist3D);

void HwArBody_getSkeletonPoint2D(const HwArSession *session,
                                 const HwArBody *body,
                                 const float **outPoint2D);

void HwArBody_getSkeletonPoint3D(const HwArSession *session,
                                 const HwArBody *body,
                                 const float **outPoint3D);

void HwArBody_getSkeletonConfidence(const HwArSession *session,
                                    const HwArBody *body,
                                    const float **outConfidence);

void HwArBody_getSkeletonConnectionSize(const HwArSession *session,
                                        const HwArBody *body,
                                        int32_t *outConnectionCount);

void HwArBody_getSkeletonConnection(const HwArSession *session,
                                    const HwArBody *body,
                                    const int32_t **outSkeletonConnection);

void HwArBody_getBodyAction(const HwArSession *session,
                            const HwArBody *body,
                            HwArBodyAction *outBodyAction);

void HwArBody_getMaskConfidence(const HwArSession *session,
                                const HwArBody *body,
                                const float **outConfidence);

void HwArBody_getMaskDepth(const HwArSession *session,
                           const HwArBody *body,
                           const short **outDepth);

// HwArObject
void HwArObject_getCenterPose(const HwArSession *session,
                              const HwArObject *object,
                              HwArPose *pose);

void HwArObject_getName(const HwArSession *session,
                        const HwArObject *object,
                        char **objectName);

void HwArObject_getObjectId(const HwArSession *session,
                            const HwArObject *object,
                            int32_t *objectId);

void HwArObject_getAnchorId(const HwArSession *session,
                            const HwArObject *object,
                            int32_t *objectAnchorId);

// HwArFace --------------------------------------------------------------------
void HwArFace_getPose(const HwArSession *session,
                      const HwArFace *face,
                      HwArPose *pose);

void HwArFace_acquireGeometry(const HwArSession *session,
                              const HwArFace *face,
                              HwArFaceGeometry **geometry);

void HwArFace_acquireBlendShapes(const HwArSession *session,
                                 const HwArFace *face,
                                 HwArFaceBlendShapes **blendshapes);

// HwArFaceGeometry ---------------------------------------------------------------------------
void HwArFaceGeometry_release(HwArFaceGeometry *geometry);

void HwArFaceGeometry_getTriangleCount(const HwArSession *session,
                                       const HwArFaceGeometry *geometry,
                                       int32_t *count);

void HwArFaceGeometry_getVerticesSize(const HwArSession *session,
                                      const HwArFaceGeometry *geometry,
                                      int32_t *count);

void HwArFaceGeometry_acquireVertices(const HwArSession *session,
                                      const HwArFaceGeometry *geometry,
                                      const float **data);

void HwArFaceGeometry_getTexCoordSize(const HwArSession *session,
                                      const HwArFaceGeometry *geometry,
                                      int32_t *count);

void HwArFaceGeometry_acquireTexCoord(const HwArSession *session,
                                      const HwArFaceGeometry *geometry,
                                      const float **data);

void HwArFaceGeometry_getTriangleIndicesSize(const HwArSession *session,
                                             const HwArFaceGeometry *geometry,
                                             int32_t *count);

void HwArFaceGeometry_acquireTriangleIndices(const HwArSession *session,
                                             const HwArFaceGeometry *geometry,
                                             const int32_t **data);

void HwArFaceGeometry_getTriangleLabelsSize(const HwArSession *session,
                                            const HwArFaceGeometry *geometry,
                                            int32_t *count);

void HwArFaceGeometry_acquireTriangleLabels(const HwArSession *session,
                                            const HwArFaceGeometry *geometry,
                                            const HwArAnimojiTriangleLabel **data);

// HwArFaceBlendShapes ---------------------------------------------------------------------------
void HwArFaceBlendShapes_release(HwArFaceBlendShapes *blendshapes);

void HwArFaceBlendShapes_acquireData(const HwArSession *session,
                                     const HwArFaceBlendShapes *blendshapes,
                                     const float **data);

void HwArFaceBlendShapes_getCount(const HwArSession *session,
                                  const HwArFaceBlendShapes *blendshapes,
                                  int32_t *count);

void HwArFaceBlendShapes_acquireTypes(const HwArSession *session,
                                      const HwArFaceBlendShapes *blendshapes,
                                      const HwArAnimojiBlendShape **types);

// HwArSceneMesh ---------------------------------------------------------------------------
void HwArSceneMesh_release(HwArSceneMesh *sceneMesh);

void HwArSceneMesh_getVerticesSize(const HwArSession *session,
                                   const HwArSceneMesh *sceneMesh,
                                   int32_t *size);

void HwArSceneMesh_acquireVertices(const HwArSession *session,
                                   const HwArSceneMesh *sceneMesh,
                                   const float **data);

void HwArSceneMesh_acquireVertexNormals(const HwArSession *session,
                                        const HwArSceneMesh *sceneMesh,
                                        const float **data);

void HwArSceneMesh_getTriangleIndicesSize(const HwArSession *session,
                                          const HwArSceneMesh *sceneMesh,
                                          int32_t *size);

void HwArSceneMesh_acquireTriangleIndices(const HwArSession *session,
                                          const HwArSceneMesh *sceneMesh,
                                          const int32_t **data);

HwArStatus HwArSceneMesh_getSceneDepth(const HwArSession *session,
                                       const HwArSceneMesh *sceneMesh,
                                       const uint16_t **data,
                                       uint32_t *width,
                                       uint32_t *heigth);

void HwArSceneMesh_getSceneDepthHeight(const HwArSession *session,
                                       const HwArSceneMesh *sceneMesh,
                                       uint32_t *height);

void HwArSceneMesh_getSceneDepthWidth(const HwArSession *session,
                                      const HwArSceneMesh *sceneMesh,
                                      uint32_t *width);

bool HwArEnginesApk_isAREngineApkReady(void *env,
                                       void *applicationContext);

HwArStatus HwArSession_create(void *env,
                              void *applicationContext,
                              HwArSession **outSessionPointer);

void HwArConfig_create(const HwArSession *session,
                       HwArConfig **outConfig);

void HwArConfig_destroy(HwArConfig *config);

void HwArConfig_getLightingMode(const HwArSession *session,
                                const HwArConfig *config,
                                int32_t *lightEstimationMode);

void HwArConfig_setLightingMode(const HwArSession *session,
                                HwArConfig *config,
                                int32_t lightEstimationMode);

void HwArConfig_getPlaneFindingMode(const HwArSession *session,
                                    const HwArConfig *config,
                                    HwArPlaneFindingMode *planeFindingMode);

void HwArConfig_setPlaneFindingMode(const HwArSession *session,
                                    HwArConfig *config,
                                    HwArPlaneFindingMode planeFindingMode);

void HwArConfig_getUpdateMode(const HwArSession *session,
                              const HwArConfig *config,
                              HwArUpdateMode *updateMode);

void HwArConfig_setUpdateMode(const HwArSession *session,
                              HwArConfig *config,
                              HwArUpdateMode updateMode);

void HwArConfig_getPowerMode(const HwArSession *session,
                             const HwArConfig *config,
                             HwArPowerMode *powerMode);

void HwArConfig_setPowerMode(const HwArSession *session,
                             HwArConfig *config,
                             HwArPowerMode powerMode);

void HwArConfig_getFocusMode(const HwArSession *session,
                             const HwArConfig *config,
                             HwArFocusMode *focusMode);

void HwArConfig_setFocusMode(const HwArSession *session,
                             HwArConfig *config,
                             HwArFocusMode focusMode);

void HwArConfig_setPreviewSize(const HwArSession *session,
                               HwArConfig *config,
                               uint32_t width,
                               uint32_t heigth);

void HwArConfig_getEnableItem(const HwArSession *session,
                              const HwArConfig *config,
                              uint64_t *item);

void HwArConfig_setEnableItem(const HwArSession *session,
                              HwArConfig *config,
                              uint64_t item);

void HwArConfig_getImageInputMode(const HwArSession *session,
                                  const HwArConfig *config,
                                  HwArImageInputMode *mode);

void HwArConfig_setImageInputMode(const HwArSession *session,
                                  HwArConfig *config,
                                  HwArImageInputMode mode);

int32_t HwArConfig_getInputNativeWindows(const HwArSession *session,
                                         const HwArConfig *config,
                                         int32_t count,
                                         int64_t **windows);

int32_t HwArConfig_getInputNativeWindowTypes(const HwArSession *session,
                                             const HwArConfig *config,
                                             int32_t count,
                                             HwArNativeWindowType **types);

void HwArConfig_getHandFindingMode(const HwArSession *session,
                                   HwArConfig *config,
                                   HwArHandFindingMode *mode);

void HwArConfig_setHandFindingMode(const HwArSession *session,
                                   HwArConfig *config,
                                   HwArHandFindingMode mode);

void HwArConfig_getFaceDetectMode(const HwArSession *session,
                                  HwArConfig *config,
                                  uint64_t *mode);

void HwArConfig_setFaceDetectMode(const HwArSession *session,
                                  HwArConfig *config,
                                  uint64_t mode);

void HwArConfig_setSemanticMode(const HwArSession *session,
                                HwArConfig *config,
                                int32_t mode);

void HwArConfig_getSemanticMode(const HwArSession *session,
                                const HwArConfig *config,
                                int32_t *mode);

void HwArConfig_setMaxMapSize(const HwArSession *session,
                              HwArConfig *config,
                              uint64_t maxMapSize);

void HwArConfig_getMaxMapSize(const HwArSession *session,
                              const HwArConfig *config,
                              uint64_t *maxMapSize);

void HwArSession_setEnvironmentTextureUpdateMode(const HwArSession *session,
                                                 HwArEnvironmentTextureUpdateMode mode);

void HwArSession_destroy(HwArSession *session);

HwArStatus HwArSession_checkSupported(const HwArSession *session,
                                      const HwArConfig *config);

HwArStatus HwArSession_configure(HwArSession *session,
                                 const HwArConfig *config);

HwArStatus HwArSession_resume(HwArSession *session);

HwArStatus HwArSession_pause(HwArSession *session);

void HwArSession_setCameraTextureName(HwArSession *session,
                                      uint32_t textureId);

void HwArSession_setDisplayGeometry(HwArSession *session,
                                    int32_t rotation,
                                    int32_t width,
                                    int32_t height);

HwArStatus HwArSession_update(HwArSession *session,
                              HwArFrame *outFrame);

HwArStatus HwArSession_acquireNewAnchor(HwArSession *session,
                                        const HwArPose *pose,
                                        HwArAnchor **outAnchor);

using MonitorServiceCallback = void (*)(int eventID, int descSize, void *desc);

HwArStatus HwArSession_setNotifyDataCallback(const HwArSession *session,
                                             const MonitorServiceCallback notify);

HwArStatus HwArSession_setCloudServiceAuthInfo(const HwArSession *session, const char *authInfo);

void HwArSession_getAllAnchors(const HwArSession *session,
                               HwArAnchorList *outAnchorList);

void HwArSession_getAllTrackables(const HwArSession *session,
                                  HwArTrackableType filterType,
                                  HwArTrackableList *outTrackableList);

void HwArSession_getSupportedSemanticMode(const HwArSession *session,
                                          int32_t *mode);

void HwArPose_create(const HwArSession *session,
                     const float *poseRaw,
                     HwArPose **outPose);

void HwArPose_destroy(HwArPose *pose);

void HwArPose_getPoseRaw(const HwArSession *session,
                         const HwArPose *pose,
                         float *outPoseRaw);

void HwArPose_getMatrix(const HwArSession *session,
                        const HwArPose *pose,
                        float *outMatrixColMajor4x4);

void HwArCamera_getPose(const HwArSession *session,
                        const HwArCamera *camera,
                        HwArPose *outPose);

void HwArCamera_getDisplayOrientedPose(const HwArSession *session,
                                       const HwArCamera *camera,
                                       HwArPose *outPose);

void HwArCamera_getViewMatrix(const HwArSession *session,
                              const HwArCamera *camera,
                              float *outColMajor4x4);

void HwArCamera_getTrackingState(const HwArSession *session,
                                 const HwArCamera *camera,
                                 HwArTrackingState *outTrackingState);

void HwArCamera_getProjectionMatrix(const HwArSession *session,
                                    const HwArCamera *camera,
                                    float near,
                                    float far,
                                    float *destColMajor4x4);

void HwArCameraIntrinsics_create(const HwArSession *session,
                                 HwArCameraIntrinsics **outIntrinsics);

void HwArCameraIntrinsics_destroy(const HwArSession *session,
                                  HwArCameraIntrinsics *intrinsics);

void HwArCameraIntrinsics_getFocalLength(const HwArSession *session,
                                         const HwArCameraIntrinsics *intrinsics,
                                         float *outFocalX,
                                         float *outFocalY);

void HwArCameraIntrinsics_getPrincipalPoint(const HwArSession *session,
                                            const HwArCameraIntrinsics *intrinsics,
                                            float *outPrincipalX,
                                            float *outPrincipalY);

void HwArCameraIntrinsics_getImageDimensions(const HwArSession *session,
                                             const HwArCameraIntrinsics *intrinsics,
                                             int32_t *outWidth,
                                             int32_t *outHeight);

void HwArCamera_getImageIntrinsics(const HwArSession *session,
                                   const HwArCamera *camera,
                                   HwArCameraIntrinsics *outIntrinsics);

void HwArCameraIntrinsics_getDistortion(const HwArSession *session,
                                        const HwArCameraIntrinsics *intrinsics,
                                        float (&outDistortion)[DISTORTION_COUNT]);

void HwArCamera_release(HwArCamera *camera);

void HwArFrame_create(const HwArSession *session,
                      HwArFrame **outFrame);

void HwArFrame_destroy(HwArFrame *frame);

void HwArFrame_getDisplayGeometryChanged(const HwArSession *session,
                                         const HwArFrame *frame,
                                         int32_t *outGeometryChanged);

void HwArFrame_getTimestamp(const HwArSession *session,
                            const HwArFrame *frame,
                            int64_t *outTimestampNs);

void HwArFrame_transformDisplayUvCoords(const HwArSession *session,
                                        const HwArFrame *frame,
                                        int32_t numElements,
                                        const float *uvsIn,
                                        float *uvsOut);

void HwArFrame_hitTest(const HwArSession *session,
                       const HwArFrame *frame,
                       float pixelX,
                       float pixelY,
                       HwArHitResultList *hitResultList);

HwArStatus HwArFrame_hitTestArea(const HwArSession *session,
                                 const HwArFrame *frame,
                                 float *input2DPoints,
                                 int32_t length,
                                 HwArHitResultList *hitResultList);

void HwArFrame_getLightEstimate(const HwArSession *session,
                                const HwArFrame *frame,
                                HwArLightEstimate *outLightEstimate);

HwArStatus HwArFrame_acquirePointCloud(const HwArSession *session,
                                       const HwArFrame *frame,
                                       HwArPointCloud **outPointCloud);

void HwArFrame_acquireCamera(const HwArSession *session,
                             const HwArFrame *frame,
                             HwArCamera **outCamera);

HwArStatus HwArFrame_acquireImageMetadata(const HwArSession *session,
                                          const HwArFrame *frame,
                                          HwArImageMetadata **outMetadata);

void HwArFrame_getUpdatedAnchors(const HwArSession *session,
                                 const HwArFrame *frame,
                                 HwArAnchorList *outAnchorList);

void HwArFrame_getUpdatedTrackables(const HwArSession *session,
                                    const HwArFrame *frame,
                                    HwArTrackableType filterType,
                                    HwArTrackableList *outTrackableList);

HwArStatus HwArFrame_acquireCameraImage(HwArSession *session,
                                        HwArFrame *frame,
                                        HwArImage **outImage);

HwArStatus HwArFrame_acquireDepthImage(HwArSession *session,
                                       HwArFrame *frame,
                                       HwArImage **outImage);

HwArStatus HwArFrame_acquirePreviewImage(HwArSession *session,
                                         HwArFrame *frame,
                                         HwArImage **outImage);

HwArStatus HwArFrame_acquireSceneMesh(const HwArSession *session,
                                      const HwArFrame *frame,
                                      HwArSceneMesh **outSceneMesh);

void HwArImage_getNdkImage(const HwArImage *image,
                           const AImage **outNdkImage);

void HwArImage_release(HwArImage *image);

void HwArPointCloud_getNumberOfPoints(const HwArSession *session,
                                      const HwArPointCloud *pointCloud,
                                      int32_t *outNumberOfPoints);

void HwArPointCloud_getData(const HwArSession *session,
                            const HwArPointCloud *pointCloud,
                            const float **outPointCloudData);

void HwArPointCloud_getTimestamp(const HwArSession *session,
                                 const HwArPointCloud *pointCloud,
                                 int64_t *outTimestampNs);

void HwArPointCloud_release(HwArPointCloud *pointCloud);

void HwArImageMetadata_getNdkCameraMetadata(const HwArSession *session,
                                            const HwArImageMetadata *imageMetadata,
                                            const ACameraMetadata **outNdkMetadata);

void HwArImageMetadata_release(HwArImageMetadata *metadata);

void HwArLightEstimate_create(const HwArSession *session,
                              HwArLightEstimate **outLightEstimate);

void HwArLightEstimate_destroy(HwArLightEstimate *lightEstimate);

void HwArLightEstimate_getState(const HwArSession *session,
                                const HwArLightEstimate *lightEstimate,
                                HwArLightEstimateState *outLightEstimateState);

void HwArLightEstimate_getPixelIntensity(const HwArSession *session,
                                         const HwArLightEstimate *lightEstimate,
                                         float *outPixelIntensity);

void HwArLightEstimate_getSphericalHarmonicCoefficients(const HwArSession *session,
                                                        const HwArLightEstimate *lightEstimate,
                                                        const float **outCoefficients);

void HwArLightEstimate_getPrimaryLightDirection(const HwArSession *session,
                                                const HwArLightEstimate *lightEstimate,
                                                const float **outPrimaryLightDirection);

void HwArLightEstimate_getPrimaryLightIntensity(const HwArSession *session,
                                                const HwArLightEstimate *lightEstimate,
                                                float *outPrimaryLightIntensity);

void HwArLightEstimate_getShadowStrength(const HwArSession *session,
                                         const HwArLightEstimate *lightEstimate,
                                         float *outPrimaryShadowStrength);

void HwArLightEstimate_getPrimaryLightColor(const HwArSession *session,
                                            const HwArLightEstimate *lightEstimate,
                                            const float **outColorTemp);

void HwArLightEstimate_getLightShadowType(const HwArSession *session,
                                          const HwArLightEstimate *lightEstimate,
                                          HwArLightShadowType *outType);

void HwArLightEstimate_acquireEnvironmentTexture(const HwArSession *session,
                                                 const HwArLightEstimate *lightEstimate,
                                                 const uint8_t **outData);

void HwArAnchorList_create(const HwArSession *session,
                           HwArAnchorList **outAnchorList);

void HwArAnchorList_destroy(HwArAnchorList *anchorList);

void HwArAnchorList_getSize(const HwArSession *session,
                            const HwArAnchorList *anchorList,
                            int32_t *outSize);

void HwArAnchorList_acquireItem(const HwArSession *session,
                                const HwArAnchorList *anchorList,
                                int32_t index,
                                HwArAnchor **outAnchor);

void HwArAnchor_getPose(const HwArSession *session,
                        const HwArAnchor *anchor,
                        HwArPose *outPose);

void HwArAnchor_getTrackingState(const HwArSession *session,
                                 const HwArAnchor *anchor,
                                 HwArTrackingState *outTrackingState);

void HwArAnchor_detach(HwArSession *session,
                       HwArAnchor *anchor);

void HwArAnchor_release(HwArAnchor *anchor);

void HwArTrackableList_create(const HwArSession *session,
                              HwArTrackableList **outTrackableList);

void HwArTrackableList_destroy(HwArTrackableList *trackableList);

void HwArTrackableList_getSize(const HwArSession *session,
                               const HwArTrackableList *trackableList,
                               int32_t *outSize);

void HwArTrackableList_acquireItem(const HwArSession *session,
                                   const HwArTrackableList *trackableList,
                                   int32_t index,
                                   HwArTrackable **outTrackable);

void HwArTrackable_release(HwArTrackable *trackable);

void HwArTrackable_getType(const HwArSession *session,
                           const HwArTrackable *trackable,
                           HwArTrackableType *outTrackableType);

void HwArTrackable_getTrackingState(const HwArSession *session,
                                    const HwArTrackable *trackable,
                                    HwArTrackingState *outTrackingState);

HwArStatus HwArTrackable_acquireNewAnchor(HwArSession *session,
                                          HwArTrackable *trackable,
                                          HwArPose *pose,
                                          HwArAnchor **outAnchor);

void HwArTrackable_getAnchors(const HwArSession *session,
                              const HwArTrackable *trackable,
                              HwArAnchorList *outAnchorList);

void HwArFace_getHealthParameterCount(const HwArSession *session,
                                      const HwArTrackable *trackable,
                                      int32_t *count);

void HwArFace_getHealthParameterValueArray(const HwArSession *session,
                                           const HwArTrackable *trackable,
                                           const float **data);

void HwArFace_getHealthParameterTypeArray(const HwArSession *session,
                                          const HwArTrackable *trackable,
                                          const HwArHealthParameterType **types);

void HwArPlane_acquireSubsumedBy(const HwArSession *session,
                                 const HwArPlane *plane,
                                 HwArPlane **outSubsumedBy);

void HwArPlane_getType(const HwArSession *session,
                       const HwArPlane *plane,
                       HwArPlaneType *outPlaneType);

void HwArPlane_getCenterPose(const HwArSession *session,
                             const HwArPlane *plane,
                             HwArPose *outPose);

void HwArPlane_getExtentX(const HwArSession *session,
                          const HwArPlane *plane,
                          float *outExtentX);

void HwArPlane_getExtentZ(const HwArSession *session,
                          const HwArPlane *plane,
                          float *outExtentZ);

void HwArPlane_getPolygonSize(const HwArSession *session,
                              const HwArPlane *plane,
                              int32_t *outPolygonSize);

void HwArPlane_getPolygon(const HwArSession *session,
                          const HwArPlane *plane,
                          float *outPolygonXz);

void HwArPlane_isPoseInExtents(const HwArSession *session,
                               const HwArPlane *plane,
                               const HwArPose *pose,
                               int32_t *outPoseInExtents);

void HwArPlane_isPoseInPolygon(const HwArSession *session,
                               const HwArPlane *plane,
                               const HwArPose *pose,
                               int32_t *outPoseInPolygon);

void HwArPlane_getLabel(const HwArSession *session,
                        const HwArPlane *plane,
                        HwArSemanticPlaneLabel *label);

void HwArPoint_getPose(const HwArSession *session,
                       const HwArPoint *point,
                       HwArPose *outPose);

void HwArPoint_getOrientationMode(const HwArSession *session,
                                  const HwArPoint *point,
                                  HwArPointOrientationMode *outOrientationMode);

void HwArHitResultList_create(const HwArSession *session,
                              HwArHitResultList **outHitResultList);

void HwArHitResultList_destroy(HwArHitResultList *hitResultList);

void HwArHitResultList_getSize(const HwArSession *session,
                               const HwArHitResultList *hitResultList,
                               int32_t *outSize);

void HwArHitResultList_getItem(const HwArSession *session,
                               const HwArHitResultList *hitResultList,
                               int32_t index,
                               HwArHitResult *outHitResult);

void HwArHitResult_create(const HwArSession *session,
                          HwArHitResult **outHitResult);

void HwArHitResult_destroy(HwArHitResult *hitResult);

void HwArHitResult_getDistance(const HwArSession *session,
                               const HwArHitResult *hitResult,
                               float *outDistance);

void HwArHitResult_getHitPose(const HwArSession *session,
                              const HwArHitResult *hitResult,
                              HwArPose *outPose);

void HwArHitResult_acquireTrackable(const HwArSession *session,
                                    const HwArHitResult *hitResult,
                                    HwArTrackable **outTrackable);

HwArStatus HwArHitResult_acquireNewAnchor(HwArSession *session,
                                          HwArHitResult *hitResult,
                                          HwArAnchor **outAnchor);

void HwArVideoSource_Constructor(void *env02,
                                 void *applicationContext,
                                 const char *videoPath,
                                 HwArSession *sessionHandle,
                                 int64_t configHandle);

void HwArVideoSource_InitPlayer(void *env02,
                                void *applicationContext);

int32_t HwArVideoSource_GetVideoWidth(void *env02,
                                      void *applicationContext);

int32_t HwArVideoSource_GetVideoHeight(void *env02,
                                       void *applicationContext);

void HwArVideoSource_StartVideoPlayer(void *env02,
                                      void *applicationContext);

void HwArVideoSource_PauseVideoPlayer(void *env02,
                                      void *applicationContext);

void HwArVideoSource_ResumeVideoPlayer(void *env02,
                                       void *applicationContext,
                                       HwArSession *sessionHandle,
                                       int64_t configHandle);

void HwArVideoSource_StopVideoPlayer(void *env02,
                                     void *applicationContext);

void HwArVideoSource_SetVideoPath(void *env02,
                                  void *applicationContext,
                                  const char *videoPath);

void HwArVideoSource_ReleaseVideoPlayer(void *env02,
                                        void *applicationContext);

void HwArTarget_getLabel(const HwArSession *session,
                         const HwArTarget *target,
                         HwArTargetLabel *label);

void HwArTarget_getCenterPose(const HwArSession *session,
                              const HwArTarget *target,
                              HwArPose *outArPose);

void HwArTarget_getAxisAlignBoundingBox(const HwArSession *session, const HwArTarget *target,
                                        float *outAabb);

void HwArTarget_getRadius(const HwArSession *session, const HwArTarget *target, float *radius);

void HwArTarget_getShapeType(const HwArSession *session,
                             const HwArTarget *target,
                             HwArTargetShapeType *shape);
#ifdef __cplusplus
}
#endif

#endif // ARNDK_HWAR_INTERFACE_H
