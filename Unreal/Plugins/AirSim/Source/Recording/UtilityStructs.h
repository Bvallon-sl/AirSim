#pragma once

#include <string>
#include "CoreMinimal.h"
#include "Engine/World.h"
#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

#include "UtilityStructs.generated.h"


#define IS_MICROSECONDS true
#define INVALID_VALUE -999
#define BBOX_MINIMUM_VOLUME 0.005
#define MAX_DISTANCE_METER 40
#define SENSORS_FREQUENCY 333.0f

#define SAVE_SENSOR_DATA 0
#define SAVE_DETECTION 1


enum class BODY_PARTS_POSE_34
{
    PELVIS = 0,
    NAVAL_SPINE = 1,
    CHEST_SPINE = 2,
    NECK = 3,
    LEFT_CLAVICLE = 4,
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 7,
    LEFT_HAND = 8,
    LEFT_HANDTIP = 9,
    LEFT_THUMB = 10,
    RIGHT_CLAVICLE = 11,
    RIGHT_SHOULDER = 12,
    RIGHT_ELBOW = 13,
    RIGHT_WRIST = 14,
    RIGHT_HAND = 15,
    RIGHT_HANDTIP = 16,
    RIGHT_THUMB = 17,
    LEFT_HIP = 18,
    LEFT_KNEE = 19,
    LEFT_ANKLE = 20,
    LEFT_FOOT = 21,
    RIGHT_HIP = 22,
    RIGHT_KNEE = 23,
    RIGHT_ANKLE = 24,
    RIGHT_FOOT = 25,
    HEAD = 26,
    NOSE = 27,
    LEFT_EYE = 28,
    LEFT_EAR = 29,
    RIGHT_EYE = 30,
    RIGHT_EAR = 31,
    LEFT_HEEL = 32,
    RIGHT_HEEL = 33,
    LAST = 34
};

enum class BODY_PARTS_POSE_18
{
    NOSE = 0,
    NECK = 1,
    RIGHT_SHOULDER = 2,
    RIGHT_ELBOW = 3,
    RIGHT_WRIST = 4,
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 7,
    RIGHT_HIP = 8,
    RIGHT_KNEE = 9,
    RIGHT_ANKLE = 10,
    LEFT_HIP = 11,
    LEFT_KNEE = 12,
    LEFT_ANKLE = 13,
    RIGHT_EYE = 14,
    LEFT_EYE = 15,
    RIGHT_EAR = 16,
    LEFT_EAR = 17,
    LAST = 18
};


//Name of bones of the skeleton rig // for mixamo's models
const TArray<FName> targetBone_34 = {
    "Hips",
    "Spine1",
    "Spine2",
    "Neck",
    "LeftShoulder",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "LeftHandMiddle1", 
    "LeftHandMiddle4",
    "LeftHandThumb4",
    "RightShoulder",
    "RightArm",
    "RightForeArm",
    "RightHand",
    "RightHandMiddle1",
    "RightHandMiddle4",
    "RightHandThumb4",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToe_End",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToe_End",
    "Head",
    "not_found", //nose
    "LeftEye",
    "not_found", //left ear
    "RightEye",
    "not_found", //right ear
    "not_found", //left heel
    "not_found" //right heel
};

//Name of bones of the skeleton rig // for mixamo's models
const TArray<FName> targetBone_18 = {
    "not_found", //nose
    "Neck",
    "RightArm",
    "RightForeArm",
    "RightHand",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "RightEye",
    "LeftEye",
    "not_found", //right ear
    "not_found" //left ear
};


const TArray<float> occlusion_thresholds = {
    20, //Hips
    30, //Spine 1
    30, //Spine 2
    15, //Neck
    25, //LeftShoulder
    20, //LeftArm
    12, //LeftForeArm
    8, //LeftHand
    8, //LeftHandMiddle1
    5, //LeftHandMiddle4
    4, //LeftHandThumb4
    25, //RightShoulder
    20, //RightArm
    12, //RightForeArm
    8, //RightHand
    8, //RightHandMiddle1
    5, //RightHandMiddle4
    4, //RightHandThumb4
    25, //LeftUpLeg
    18, //LeftLeg
    12, //LeftFoot
    7, // LeftToe_End
    25, //RightUpLeg
    18, //RightLeg
    12, //RightFoot
    7, // RightToe_End
    15, //Head
    999, //
    7, //LeftEye
    999,
    7, //RightEye
    999,
    999,
    999
};

// enum to string fnct
static FString enumToString(BODY_PARTS_POSE_34 joint)
{
    FString tostring;

    switch (joint) {

    case BODY_PARTS_POSE_34::PELVIS:
        tostring = "PELVIS";
        break;
    case BODY_PARTS_POSE_34::NAVAL_SPINE:
        tostring = "NAVAL_SPINE";
        break;
    case BODY_PARTS_POSE_34::CHEST_SPINE:
        tostring = "CHEST_SPINE";
        break;
    case BODY_PARTS_POSE_34::NECK:
        tostring = "NECK";
        break;
    case BODY_PARTS_POSE_34::LEFT_CLAVICLE:
        tostring = "LEFT_CLAVICLE";
        break;
    case BODY_PARTS_POSE_34::LEFT_SHOULDER:
        tostring = "LEFT_SHOULDER";
        break;
    case BODY_PARTS_POSE_34::LEFT_ELBOW:
        tostring = "LEFT_ELBOW";
        break;
    case BODY_PARTS_POSE_34::LEFT_WRIST:
        tostring = "LEFT_WRIST";
        break;
    case BODY_PARTS_POSE_34::LEFT_HAND:
        tostring = "LEFT_HAND";
        break;
    case BODY_PARTS_POSE_34::LEFT_HANDTIP:
        tostring = "LEFT_HANDTIP";
        break;
    case BODY_PARTS_POSE_34::LEFT_THUMB:
        tostring = "LEFT_THUMB";
        break;
    case BODY_PARTS_POSE_34::RIGHT_CLAVICLE:
        tostring = "RIGHT_CLAVICLE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_SHOULDER:
        tostring = "RIGHT_SHOULDER";
        break;
    case BODY_PARTS_POSE_34::RIGHT_ELBOW:
        tostring = "RIGHT_ELBOW";
        break;
    case BODY_PARTS_POSE_34::RIGHT_WRIST:
        tostring = "RIGHT_WRIST";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HAND:
        tostring = "RIGHT_HAND";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HANDTIP:
        tostring = "RIGHT_HANDTIP";
        break;
    case BODY_PARTS_POSE_34::RIGHT_THUMB:
        tostring = "RIGHT_THUMB";
        break;
    case BODY_PARTS_POSE_34::LEFT_HIP:
        tostring = "LEFT_HIP";
        break;
    case BODY_PARTS_POSE_34::LEFT_KNEE:
        tostring = "LEFT_KNEE";
        break;
    case BODY_PARTS_POSE_34::LEFT_ANKLE:
        tostring = "LEFT_ANKLE";
        break;
    case BODY_PARTS_POSE_34::LEFT_FOOT:
        tostring = "LEFT_FOOT";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HIP:
        tostring = "RIGHT_HIP";
        break;
    case BODY_PARTS_POSE_34::RIGHT_KNEE:
        tostring = "RIGHT_KNEE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_ANKLE:
        tostring = "RIGHT_ANKLE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_FOOT:
        tostring = "RIGHT_FOOT";
        break;
    case BODY_PARTS_POSE_34::HEAD:
        tostring = "HEAD";
        break;
    case BODY_PARTS_POSE_34::NOSE:
        tostring = "NOSE";
        break;
    case BODY_PARTS_POSE_34::LEFT_EYE:
        tostring = "LEFT_EYE";
        break;
    case BODY_PARTS_POSE_34::LEFT_EAR:
        tostring = "LEFT_EAR";
        break;
    case BODY_PARTS_POSE_34::RIGHT_EYE:
        tostring = "RIGHT_EYE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_EAR:
        tostring = "RIGHT_EAR";
        break;
    case BODY_PARTS_POSE_34::LEFT_HEEL:
        tostring = "LEFT_HEEL";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HEEL:
        tostring = "RIGHT_HEEL";
        break;
    default:
        tostring = "WRONG JOINT NAME";
    }

    return tostring;
}

static FString enumToString(BODY_PARTS_POSE_18 joint)
{
    FString tostring;

    switch (joint) {

    case BODY_PARTS_POSE_18::LEFT_SHOULDER:
        tostring = "LEFT_SHOULDER";
        break;
    case BODY_PARTS_POSE_18::LEFT_ELBOW:
        tostring = "LEFT_ELBOW";
        break;

    case BODY_PARTS_POSE_18::LEFT_WRIST:
        tostring = "LEFT_WRIST";
        break;
    case BODY_PARTS_POSE_18::RIGHT_SHOULDER:
        tostring = "RIGHT_SHOULDER";
        break;
    case BODY_PARTS_POSE_18::RIGHT_ELBOW:
        tostring = "RIGHT_ELBOW";
        break;
    case BODY_PARTS_POSE_18::RIGHT_WRIST:
        tostring = "RIGHT_WRIST";
        break;
    case BODY_PARTS_POSE_18::LEFT_HIP:
        tostring = "LEFT_HIP";
        break;
    case BODY_PARTS_POSE_18::LEFT_KNEE:
        tostring = "LEFT_KNEE";
        break;
    case BODY_PARTS_POSE_18::LEFT_ANKLE:
        tostring = "LEFT_ANKLE";
        break;
    case BODY_PARTS_POSE_18::RIGHT_HIP:
        tostring = "RIGHT_HIP";
        break;
    case BODY_PARTS_POSE_18::RIGHT_KNEE:
        tostring = "RIGHT_KNEE";
        break;
    case BODY_PARTS_POSE_18::RIGHT_ANKLE:
        tostring = "RIGHT_ANKLE";
        break;
    case BODY_PARTS_POSE_18::NOSE:
        tostring = "NOSE";
        break;
    case BODY_PARTS_POSE_18::NECK:
        tostring = "NECK";
        break;
    case BODY_PARTS_POSE_18::LEFT_EYE:
        tostring = "LEFT_EYE";
        break;
    case BODY_PARTS_POSE_18::LEFT_EAR:
        tostring = "LEFT_EAR";
        break;
    case BODY_PARTS_POSE_18::RIGHT_EYE:
        tostring = "RIGHT_EYE";
        break;
    case BODY_PARTS_POSE_18::RIGHT_EAR:
        tostring = "RIGHT_EAR";
        break;
    default:
        tostring = "WRONG JOINT NAME";
    }
    return tostring;
}

static bool isInvalidValue(FVector in) {
    bool isInvalid = false;

    if (in.X == INVALID_VALUE /* && in.Y <= INVALID_VALUE && in.Z <= INVALID_VALUE*/) {
        isInvalid = true;
    }

    return isInvalid;
}

static bool isInvalidValue(FQuat in)
{
    bool isInvalid = false;

    if (in.X <= INVALID_VALUE && in.Y <= INVALID_VALUE && in.Z <= INVALID_VALUE && in.W <= INVALID_VALUE) {
        isInvalid = true;
    }

    return isInvalid;
}

static FVector warpPoint_(FVector pt_curr, const FMatrix path, float scale = 1)
{
    FVector proj3D;
    const auto p_path = path.M;

    proj3D.X = ((pt_curr.X * p_path[0][0]) + (pt_curr.Y * p_path[0][1]) + (pt_curr.Z * p_path[0][2]) + p_path[0][3]) * scale;
    proj3D.Y = ((pt_curr.X * p_path[1][0]) + (pt_curr.Y * p_path[1][1]) + (pt_curr.Z * p_path[1][2]) + p_path[1][3]) * scale;
    proj3D.Z = ((pt_curr.X * p_path[2][0]) + (pt_curr.Y * p_path[2][1]) + (pt_curr.Z * p_path[2][2]) + p_path[2][3]) * scale;
    return proj3D;
}

static FVector2D projectPoint_(FVector pt, float fx, float fy, float cx, float cy) {
    FVector2D pxl = FVector2D::ZeroVector;

    pxl.X = (pt.X / pt.Z) * fx + cx;
    pxl.Y = (pt.Y / pt.Z) * fy + cy;

    return pxl;
}

static FVector convertFromImageToUnityCoordinateSystem(FVector in)
{
    FVector out(in.X, - in.Y, in.Z);
    out /= 100;

    return out;
}

static FVector convertFromImageToZEDIMUCoordinateSystem(FVector in)
{
    FVector out(in.Y, in.X, -in.Z);
    return out;
}

static FVector convertFromUUToUnityCoordinateSystem(FVector in, bool convertUnit = true)
{
    int factor = convertUnit ? 100 : 1;
    FVector out(in.Y, in.Z, in.X);
    out /= factor;

    return out;
}

static FTransform getCoordinateTransformConversion4f()
{
    FMatrix tmp, coordTransf;
    tmp.SetIdentity();
    coordTransf.SetIdentity();

    //getCoordinateTransform
    tmp.M[0][0] = 0;
    tmp.M[0][1] = 1;
    tmp.M[1][0] = 0;
    tmp.M[1][1] = 0;
    tmp.M[1][2] = -1;
    tmp.M[2][0] = 1;
    tmp.M[2][1] = 0;
    tmp.M[2][2] = 0;

    //getInverseCoordinateTransform
    coordTransf.M[1][1] = -1;

    return FTransform(coordTransf.Inverse()) * FTransform(tmp);
}

static FTransform convertFromUUToUnityCoordinateSystem(FTransform in)
{
    FTransform coordTransf = getCoordinateTransformConversion4f();
    FTransform out = (coordTransf * in * coordTransf.Inverse());

    return out;
}

static FVector convertFromUUToImageCoordinateSystem(FVector in)
{
    FVector out = FVector(in.Y, -in.Z, in.X);
    return out;
}

static FTransform convertFromUUToImageCoordinateSystem(FTransform in)
{
    FMatrix tmp;
    tmp.SetIdentity();
    FMatrix coordTransf;
    coordTransf.SetIdentity();

    //getCoordinateTransform
    tmp.M[0][0] = 0;    tmp.M[0][1] = 1;    tmp.M[0][2] = 0; //src is left handed - z - up
    tmp.M[1][0] = 0;    tmp.M[1][1] = 0;    tmp.M[1][2] = -1;
    tmp.M[2][0] = 1;    tmp.M[2][1] = 0;    tmp.M[2][2] = 0;

    coordTransf.SetIdentity(); // dest is IMAGE

    FTransform tf = FTransform(coordTransf.Inverse()) * FTransform(tmp);
    FTransform out = tf * in * tf.Inverse(); 

    return out;
}

static FTransform convertFromUnityToImageCoordinateSystem(FTransform in)
{
    FMatrix tmp;
    tmp.SetIdentity();
    FMatrix coordTransf;
    coordTransf.SetIdentity();

    //getCoordinateTransform
    tmp.M[1][1] = -1;

    coordTransf.SetIdentity(); // dest is IMAGE

    FTransform tf = FTransform(coordTransf.Inverse()) * FTransform(tmp);
    FTransform out = tf * in * tf.Inverse();

    return out;
}

static FVector camToWorld(FTransform camPose, FVector in) {
    //in = camPose.TransformPosition(in);
    in = camPose.GetRotation().RotateVector(in);
    in += camPose.GetTranslation();
    return in;
}

static FQuat camToWorld(FTransform camPose, FQuat in)
{
    in = camPose.TransformRotation(in);
    return in;
}

static FVector worldToCam(FTransform camPose, FVector in)
{
    camPose = camPose.Inverse();
    in = camPose.GetRotation().RotateVector(in);
    in += camPose.GetTranslation();
    return in;
}

static FQuat worldToCam(FTransform camPose, FQuat in)
{
    camPose = camPose.Inverse();
    in = camPose.TransformRotation(in);
    return in;
}

// 3x3 matrix to 3x1 rodrigues vector
static FVector convertMatrixToRot(FMatrix mat) {
    FVector output;

    cv::Mat Rot3x3 = cv::Mat(3, 3, CV_32FC1);
    Rot3x3.at<float>(0, 0) = mat.M[0][0];
    Rot3x3.at<float>(0, 1) = mat.M[0][1];
    Rot3x3.at<float>(0, 2) = mat.M[0][2];

    Rot3x3.at<float>(1, 0) = mat.M[1][0];
    Rot3x3.at<float>(1, 1) = mat.M[1][1];
    Rot3x3.at<float>(1, 2) = mat.M[1][2];

    Rot3x3.at<float>(2, 0) = mat.M[2][0];
    Rot3x3.at<float>(2, 1) = mat.M[2][1];
    Rot3x3.at<float>(2, 2) = mat.M[2][2];

    cv::Mat Rot3x1;
    cv::Rodrigues(Rot3x3, Rot3x1);

    output = FVector(Rot3x1.at<float>(0, 0), Rot3x1.at<float>(1, 0), Rot3x1.at<float>(2, 0));

    return output;
}


static bool Raycast(AActor* actor, FVector start, FVector end) {
    bool hit = false;

    FHitResult Hit; 

    actor->GetWorld()->LineTraceSingleByChannel(OUT Hit, start, end, ECollisionChannel::ECC_PhysicsBody);

    // See what if anything has been hit and return what
    AActor* ActorHit = Hit.GetActor();

    UKismetSystemLibrary::DrawDebugLine(actor->GetWorld(), start, end, FColor(100, 0, 0), 1, 5);

    if (ActorHit->GetName() == actor->GetName()) {
        hit = true;
        UE_LOG(LogTemp, Warning, TEXT("Line trace has hit: %s . Bone name : %s"), *(ActorHit->GetName()), *Hit.BoneName.ToString());
    }


    return hit;
}


struct ImageToSave
{

public:
    int height;

    int width;

    int image_type;

    std::vector<uint8_t> image_data_uint8;

    std::vector<float> image_data_float;

    cv::Mat mask;

    std::string image_full_file_path;
};

USTRUCT()
struct FJsonGeoPoint
{
    GENERATED_BODY()

public:
    UPROPERTY()
    double latitude;
    UPROPERTY()
    double longitude;
    UPROPERTY()
    float altitude;
};

USTRUCT()
struct FJsonGPSData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    int64 EpochTimeStamp;
    UPROPERTY()
    FJsonGeoPoint geoPoint;
    UPROPERTY()
    float eph; //GPS HDOP/VDOP horizontal/vertical dilution of position (unitless), 0-100%
    UPROPERTY()
    float epv;
    UPROPERTY()
    FVector velocity;
};

USTRUCT()
struct FJsonMagnometerData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    int64 EpochTimeStamp;
    UPROPERTY()
    FVector magneticFieldBody; //in Gauss
    //UPROPERTY()
    //TArray<float> magneticFieldCovariance; //9 elements 3x3 matrix
};

USTRUCT()
struct FJsonBarometerData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    int64 EpochTimeStamp;
    UPROPERTY()
    float altitude; //meters
    UPROPERTY()
    float pressure; //Pascal
    UPROPERTY()
    float qnh;
};

USTRUCT()
struct FJsonIMUData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    int64 EpochTimeStamp;
    UPROPERTY()
    FQuat orientation;
    UPROPERTY()
    FVector angularVelocity;
    UPROPERTY()
    FVector linearAcceleration;
};


USTRUCT()
struct FJsonSensorsData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    TArray<FJsonGPSData> GPSData;

    UPROPERTY()
    TArray<FJsonMagnometerData> magnetometerData;

    UPROPERTY()
    TArray<FJsonBarometerData> barometerData;

    UPROPERTY()
    TArray<FJsonIMUData> IMUData;
};

USTRUCT()
struct FJsonIMUConfiguration
{
    GENERATED_BODY()

public:
    UPROPERTY()
    float GyroRandomWalk;

    UPROPERTY()
    float AngularNoiseDensity;

    UPROPERTY()
    float AccelNoiseDensity;

    UPROPERTY()
    float AccelRandomWalk;
};

USTRUCT()
struct FJsonSensorsConfiguration
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FJsonIMUConfiguration IMUConfiguration;

};

USTRUCT()
struct FJsonCamerasConfiguration
{
    GENERATED_BODY()

public:
    UPROPERTY()
    float baseline;

    UPROPERTY()
    float fx;

    UPROPERTY()
    float fy;

    UPROPERTY()
    float cx;

    UPROPERTY()
    float cy;
};


USTRUCT()
struct FJsonMatrix4x4
{
    GENERATED_BODY()

public:
    UPROPERTY()
    float e00;

    UPROPERTY()
    float e01;

    UPROPERTY()
    float e02;

    UPROPERTY()
    float e03;

    UPROPERTY()
    float e10;

    UPROPERTY()
    float e11;

    UPROPERTY()
    float e12;

    UPROPERTY()
    float e13;

    UPROPERTY()
    float e20;

    UPROPERTY()
    float e21;

    UPROPERTY()
    float e22;

    UPROPERTY()
    float e23;

    UPROPERTY()
    float e30;

    UPROPERTY()
    float e31;

    UPROPERTY()
    float e32;

    UPROPERTY()
    float e33;

    FJsonMatrix4x4() {
        e00 = INVALID_VALUE;
        e01 = INVALID_VALUE;
        e02 = INVALID_VALUE; 
        e03 = INVALID_VALUE;

        e10 = INVALID_VALUE;
        e11 = INVALID_VALUE;
        e12 = INVALID_VALUE;
        e13 = INVALID_VALUE;

        e20 = INVALID_VALUE;
        e21 = INVALID_VALUE;
        e22 = INVALID_VALUE;
        e23 = INVALID_VALUE;

        e30 = INVALID_VALUE;
        e31 = INVALID_VALUE;
        e32 = INVALID_VALUE;
        e33 = INVALID_VALUE;
    }

    FJsonMatrix4x4(msr::airlib::Pose pose) {

        e03 = pose.position.x();
        e13 = pose.position.y();
        e23 = pose.position.z();
        e33 = 1.0;

        Eigen::Matrix3f rot = pose.orientation.toRotationMatrix();
        e00 = rot(0, 0);
        e01 = rot(0, 1);
        e02 = rot(0, 2);
        e10 = rot(1, 0);
        e11 = rot(1, 1);
        e12 = rot(1, 2);
        e20 = rot(2, 0);
        e21 = rot(2, 1);
        e22 = rot(2, 2);

        e30 = 0;
        e31 = 0;
        e32 = 0;
    }

    FJsonMatrix4x4(FTransform transform)
    {
        //Convert to Unity Coordinate system (left handed Z up to left handed Y up)
        FTransform coordTransf = getCoordinateTransformConversion4f();
        FMatrix mat = (coordTransf * transform * coordTransf.Inverse()).ToMatrixWithScale();
        mat = mat.GetTransposed();

        e03 = mat.M[0][3] / 100.0;
        e13 = mat.M[1][3] / 100.0;
        e23 = mat.M[2][3] / 100.0;
        e33 = 1.0;

        e00 = mat.M[0][0];
        e01 = mat.M[0][1];
        e02 = mat.M[0][2];
        e10 = mat.M[1][0];
        e11 = mat.M[1][1];
        e12 = mat.M[1][2];
        e20 = mat.M[2][0];
        e21 = mat.M[2][1];
        e22 = mat.M[2][2];

        e30 = 0;
        e31 = 0;
        e32 = 0;
    }

    FTransform toTransform() {
        FTransform transform;

        FMatrix mat = FMatrix();
        mat.SetIdentity();

        mat.M[0][0] = e00;
        mat.M[0][1] = e01;
        mat.M[0][2] = e02;
        mat.M[0][3] = e03;
        mat.M[1][0] = e10;
        mat.M[1][1] = e11;
        mat.M[1][2] = e12;
        mat.M[1][3] = e13;
        mat.M[2][0] = e20;
        mat.M[2][1] = e21;
        mat.M[2][2] = e22;
        mat.M[2][3] = e23;
        mat.M[3][0] = e20;
        mat.M[3][1] = e31;
        mat.M[3][2] = e32;
        mat.M[3][3] = e33;


        transform.SetRotation(FQuat(mat));
        transform.SetTranslation(FVector(e03, e13, e23));
        //transform.SetFromMatrix(mat.GetTransposed());
        return transform;
    }
};


USTRUCT()
struct FJsonMetaData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    int64 SequenceID;

    UPROPERTY()
    int32 ZEDSerialNumber;

    UPROPERTY()
    bool IsRealZED;

    UPROPERTY()
    bool IsRectified;

    UPROPERTY()
    int32 ImageWidth;

    UPROPERTY()
    int32 ImageHeight;

    UPROPERTY()
    int32 TargetFPS;

    UPROPERTY()
    float InvalidValue;

    UPROPERTY()
    float Bbox3dMinimumVolume;

    UPROPERTY()
    bool IsMicroseconds;
};

USTRUCT()
struct FJsonFramePoseData
{
    GENERATED_BODY()

public:

    UPROPERTY()
   FJsonMatrix4x4 WorldPose;
};


USTRUCT()
struct FJsonBoundingBox3DData
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FVector A;

    UPROPERTY()
    FVector B;

    UPROPERTY()
    FVector C;

    UPROPERTY()
    FVector D;

    UPROPERTY()
    FVector E;

    UPROPERTY()
    FVector F;

    UPROPERTY()
    FVector G;

    UPROPERTY()
    FVector H;

    FJsonBoundingBox3DData() {
        A = FVector(INVALID_VALUE,INVALID_VALUE,INVALID_VALUE);
        B = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
        C = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
        D = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
        E = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
        F = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
        G = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
        H = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
    }
};

USTRUCT()
struct FJsonBoundingBox2DData
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FVector2D A;

    UPROPERTY()
    FVector2D B;

    UPROPERTY()
    FVector2D C;

    UPROPERTY()
    FVector2D D;

    FJsonBoundingBox2DData()
    {
        A = FVector2D(INVALID_VALUE,INVALID_VALUE);
        B = FVector2D(INVALID_VALUE, INVALID_VALUE);
        C = FVector2D(INVALID_VALUE, INVALID_VALUE);
        D = FVector2D(INVALID_VALUE, INVALID_VALUE);
    }

};

USTRUCT()
struct FJsonSingleDetection
{
    GENERATED_BODY()
public:

    UPROPERTY()
    int ObjectID;

    UPROPERTY()
    int ObjectType;

    UPROPERTY()
    FVector Position3D_World_Floor;

    UPROPERTY()
    FVector Velocity3D_MPS;

    UPROPERTY()
    FVector Dimensions3D;

    UPROPERTY()
    FJsonBoundingBox3DData BoundingBox3D_World;

    UPROPERTY()
    FJsonBoundingBox3DData BoundingBox3D_World_Raw;

    UPROPERTY()
    FJsonBoundingBox2DData BoundingBox2D;

    UPROPERTY()
    FJsonBoundingBox2DData BoundingBox2D_Raw;

    UPROPERTY()
    FQuat GlobalRootOrientation;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D_34;

    UPROPERTY()
    TArray<FVector> Keypoints3D;

    UPROPERTY()
    TArray<FVector> Keypoints3D_34;

    UPROPERTY()
    TArray<FVector> LocalPositionPerJoint;

    UPROPERTY()
    TArray<FQuat> LocalOrientationPerJoint;
};

USTRUCT()
struct FJsonSkeletonData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FQuat GlobalRootOrientation;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D_34;

    UPROPERTY()
    TArray<FVector> Keypoints3D;

    UPROPERTY()
    TArray<FVector> Keypoints3D_34;

    UPROPERTY()
    TArray<FVector> LocalPositionPerJoint;

    UPROPERTY()
    TArray<FQuat> LocalOrientationPerJoint;
};

USTRUCT()
struct FJsonFrameDetections
{
    GENERATED_BODY()

public :
    UPROPERTY()
        TArray<FJsonSingleDetection> ObjectDetections;
};

USTRUCT()
struct FJsonFrameData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FString ImageFileName;

    UPROPERTY()
    int32 FrameIndex;

    UPROPERTY()
    int64 EpochTimeStamp;

    UPROPERTY()
    FJsonFrameDetections Detections;

    UPROPERTY()
    FJsonFramePoseData TrackedPose;
    
};

USTRUCT()
struct FJsonDataSet
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FJsonMetaData Metadata;

    UPROPERTY()
    FJsonMatrix4x4 InitialWorldPosition;

    UPROPERTY()
    FJsonCamerasConfiguration CamerasConfiguration;

    UPROPERTY()
    FJsonSensorsConfiguration SensorsConfiguration;

    UPROPERTY()
    FJsonGeoPoint GeoPoint;

    UPROPERTY()
    TArray<FJsonFrameData> Frames;

    UPROPERTY()
    FJsonSensorsData Sensors;
};

USTRUCT()
struct FJsonSingleCalib
{
    GENERATED_BODY()

public:
    UPROPERTY()
    int inputType;

    UPROPERTY()
    FString inputPath;

    UPROPERTY()
    float rx;

    UPROPERTY()
    float ry;

    UPROPERTY()
    float rz;

    UPROPERTY()
    int64 serial;

    UPROPERTY()
    float tx;

    UPROPERTY()
    float ty;

    UPROPERTY()
    float tz;

};

USTRUCT()
struct FJsonMulticamCalib
{
    GENERATED_BODY()

public:
    UPROPERTY()
    TArray<FJsonSingleCalib> singleCalibs;
};

static FString SerializeJson(FJsonMulticamCalib calibs) {

    FString OutputString;
    TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

    TArray<TSharedPtr<FJsonValue>> objArray;

	for (int32 i = 0; i < calibs.singleCalibs.Num(); i++)
    {
        FJsonSingleCalib calib = calibs.singleCalibs[i];

        TSharedPtr<FJsonObject> json = MakeShareable(new FJsonObject);

        json->SetNumberField("input", calib.inputType);
        json->SetStringField("input_path", calib.inputPath);

        json->SetNumberField("rx", calib.rx);
        json->SetNumberField("ry", calib.ry);
        json->SetNumberField("rz", calib.rz);

        json->SetNumberField("tx", calib.tx);
        json->SetNumberField("ty", calib.ty);
        json->SetNumberField("tz", calib.tz);

        json->SetNumberField("serial", calib.serial);

        TSharedPtr<FJsonValue> value = MakeShareable(new FJsonValueObject(json));

        objArray.Add(value);
    }
        
    JsonObject->SetArrayField("ZED", objArray);

    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
    FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

    return OutputString;
}


static FString SerializeJson(FJsonDataSet data)
{
    FString OutputString;

    // create a Json object and add a string field
    TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

    ////////////////////////////////////
    /// METADATA ///////////////////////
    ////////////////////////////////////    
    
    TSharedPtr<FJsonObject> metadata = MakeShareable(new FJsonObject);
    metadata->SetNumberField("SequenceID", data.Metadata.SequenceID);
    metadata->SetNumberField("ZEDSerialNumber", data.Metadata.ZEDSerialNumber);
    metadata->SetBoolField("IsRealZED", data.Metadata.IsRealZED);
    metadata->SetBoolField("IsRectified", data.Metadata.IsRectified);
    metadata->SetNumberField("ImageWidth", data.Metadata.ImageWidth);
    metadata->SetNumberField("ImageHeight", data.Metadata.ImageHeight);
    metadata->SetNumberField("TargetFPS", data.Metadata.TargetFPS);
    metadata->SetNumberField("InvalidValue", data.Metadata.InvalidValue);
    metadata->SetNumberField("Bbox3dMinimumVolume", data.Metadata.Bbox3dMinimumVolume);
    metadata->SetBoolField("IsMicroseconds", data.Metadata.IsMicroseconds);

    JsonObject->SetObjectField("Metadata", metadata);

    // Initial World Position
    TSharedPtr<FJsonObject> InitialWorldPosition = MakeShareable(new FJsonObject);
    InitialWorldPosition->SetNumberField("e00", data.InitialWorldPosition.e00);
    InitialWorldPosition->SetNumberField("e01", data.InitialWorldPosition.e01);
    InitialWorldPosition->SetNumberField("e02", data.InitialWorldPosition.e02);
    InitialWorldPosition->SetNumberField("e03", data.InitialWorldPosition.e03);
    InitialWorldPosition->SetNumberField("e10", data.InitialWorldPosition.e10);
    InitialWorldPosition->SetNumberField("e11", data.InitialWorldPosition.e11);
    InitialWorldPosition->SetNumberField("e12", data.InitialWorldPosition.e12);
    InitialWorldPosition->SetNumberField("e13", data.InitialWorldPosition.e13);
    InitialWorldPosition->SetNumberField("e20", data.InitialWorldPosition.e20);
    InitialWorldPosition->SetNumberField("e21", data.InitialWorldPosition.e21);
    InitialWorldPosition->SetNumberField("e22", data.InitialWorldPosition.e22);
    InitialWorldPosition->SetNumberField("e23", data.InitialWorldPosition.e23);
    InitialWorldPosition->SetNumberField("e30", data.InitialWorldPosition.e30);
    InitialWorldPosition->SetNumberField("e31", data.InitialWorldPosition.e31);
    InitialWorldPosition->SetNumberField("e32", data.InitialWorldPosition.e32);
    InitialWorldPosition->SetNumberField("e33", data.InitialWorldPosition.e33);

    JsonObject->SetObjectField("InitialWorldPosition", InitialWorldPosition);

    // Camera configuration

    TSharedPtr<FJsonObject> CamerasConfiguration = MakeShareable(new FJsonObject);
    CamerasConfiguration->SetNumberField("fx", data.CamerasConfiguration.fx);
    CamerasConfiguration->SetNumberField("fy", data.CamerasConfiguration.fy);
    CamerasConfiguration->SetNumberField("cx", data.CamerasConfiguration.cx);
    CamerasConfiguration->SetNumberField("cy", data.CamerasConfiguration.cy);
    CamerasConfiguration->SetNumberField("baseline", data.CamerasConfiguration.baseline);

    JsonObject->SetObjectField("CamerasConfiguration", CamerasConfiguration);

#if SAVE_SENSOR_DATA
    TSharedPtr<FJsonObject> OriginGeoPoint = MakeShareable(new FJsonObject);

    OriginGeoPoint->SetNumberField("Altitude", data.GeoPoint.altitude);
    OriginGeoPoint->SetNumberField("Latitude", data.GeoPoint.latitude);
    OriginGeoPoint->SetNumberField("Longitude", data.GeoPoint.longitude);

    JsonObject->SetObjectField("OriginGeopoint", OriginGeoPoint);

    TSharedPtr<FJsonObject> SensorsConfig = MakeShareable(new FJsonObject);
    TSharedPtr<FJsonObject> IMUConfig = MakeShareable(new FJsonObject);

    IMUConfig->SetNumberField("AngularNoiseDensity", data.SensorsConfiguration.IMUConfiguration.AngularNoiseDensity);
    IMUConfig->SetNumberField("GyroRandomWalk", data.SensorsConfiguration.IMUConfiguration.GyroRandomWalk);
    IMUConfig->SetNumberField("AccelNoiseDensity", data.SensorsConfiguration.IMUConfiguration.AccelNoiseDensity);
    IMUConfig->SetNumberField("AccelRandomWalk", data.SensorsConfiguration.IMUConfiguration.AccelRandomWalk);

    SensorsConfig->SetObjectField("IMUConfiguration", IMUConfig);

    JsonObject->SetObjectField("SensorsConfiguration", SensorsConfig);
#endif

    TArray<TSharedPtr<FJsonValue>> Frames;
    for(auto frame : data.Frames) {

        TSharedPtr<FJsonObject> frameObj = MakeShareable(new FJsonObject);
        frameObj->SetStringField("ImageFileName", frame.ImageFileName);
        frameObj->SetNumberField("FrameIndex", frame.FrameIndex);
        frameObj->SetNumberField("EpochTimeStamp", frame.EpochTimeStamp);

        ////////////////////////////////////
        /// ODOMETRY DATA //////////////////
        ////////////////////////////////////

        TSharedPtr<FJsonObject> TrackedPose = MakeShareable(new FJsonObject);
        TSharedPtr<FJsonObject> WorldPose = MakeShareable(new FJsonObject);

        WorldPose->SetNumberField("e00", frame.TrackedPose.WorldPose.e00);
        WorldPose->SetNumberField("e01", frame.TrackedPose.WorldPose.e01);
        WorldPose->SetNumberField("e02", frame.TrackedPose.WorldPose.e02);
        WorldPose->SetNumberField("e03", frame.TrackedPose.WorldPose.e03);
        WorldPose->SetNumberField("e10", frame.TrackedPose.WorldPose.e10);
        WorldPose->SetNumberField("e11", frame.TrackedPose.WorldPose.e11);
        WorldPose->SetNumberField("e12", frame.TrackedPose.WorldPose.e12);
        WorldPose->SetNumberField("e13", frame.TrackedPose.WorldPose.e13);
        WorldPose->SetNumberField("e20", frame.TrackedPose.WorldPose.e20);
        WorldPose->SetNumberField("e21", frame.TrackedPose.WorldPose.e21);
        WorldPose->SetNumberField("e22", frame.TrackedPose.WorldPose.e22);
        WorldPose->SetNumberField("e23", frame.TrackedPose.WorldPose.e23);
        WorldPose->SetNumberField("e30", frame.TrackedPose.WorldPose.e30);
        WorldPose->SetNumberField("e31", frame.TrackedPose.WorldPose.e31);
        WorldPose->SetNumberField("e32", frame.TrackedPose.WorldPose.e32);
        WorldPose->SetNumberField("e33", frame.TrackedPose.WorldPose.e33);

        TrackedPose->SetObjectField("WorldPose", WorldPose);
        frameObj->SetObjectField("TrackedPose", TrackedPose);

        ////////////////////////////////////
        /// DETECTION DATA /////////////////
        ////////////////////////////////////

        TSharedPtr<FJsonObject> Detections = MakeShareable(new FJsonObject);

        TArray<TSharedPtr<FJsonValue>> ObjectDetections;

        for (auto detection : frame.Detections.ObjectDetections) 
        {
            TSharedPtr<FJsonObject> singleDetectionObj = MakeShareable(new FJsonObject);

            singleDetectionObj->SetNumberField("ObjectID", detection.ObjectID);
            singleDetectionObj->SetNumberField("ObjectType", detection.ObjectType);

            TArray<TSharedPtr<FJsonValue>> position3D;
            position3D.SetNum(3);
            position3D[0] = (MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[0])));
            position3D[1] = (MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[1])));
            position3D[2] = (MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[2])));
            singleDetectionObj->SetArrayField("Position3D_World_Floor", position3D);

            TArray<TSharedPtr<FJsonValue>> velocity3D;
            velocity3D.SetNum(3);
            velocity3D[0] = (MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[0])));
            velocity3D[1] = (MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[1])));
            velocity3D[2] = (MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[2])));
            singleDetectionObj->SetArrayField("Velocity3D_MPS", velocity3D);

            TArray<TSharedPtr<FJsonValue>> dimensions3D;
            dimensions3D.SetNum(3);
            dimensions3D[0] = (MakeShareable(new FJsonValueNumber(detection.Dimensions3D[0])));
            dimensions3D[1] = (MakeShareable(new FJsonValueNumber(detection.Dimensions3D[1])));
            dimensions3D[2] = (MakeShareable(new FJsonValueNumber(detection.Dimensions3D[2])));
            singleDetectionObj->SetArrayField("Dimensions3D", dimensions3D);

            ////////////////////////////////////
            /// 2D DATA ////// /////////////////
            ////////////////////////////////////
            
           TSharedPtr<FJsonObject> Bbox_2D = MakeShareable(new FJsonObject);
            TArray<TSharedPtr<FJsonValue>> A;
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.A[1])));
            Bbox_2D->SetArrayField("A", A);

            TArray<TSharedPtr<FJsonValue>> B;
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.B[1])));
            Bbox_2D->SetArrayField("B", B);

            TArray<TSharedPtr<FJsonValue>> C;
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.C[1])));
            Bbox_2D->SetArrayField("C", C);

            TArray<TSharedPtr<FJsonValue>> D;
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.D[1])));
            Bbox_2D->SetArrayField("D", D);

            singleDetectionObj->SetObjectField("BoundingBox2D", Bbox_2D);

            TSharedPtr<FJsonObject> Bbox_2D_Raw = MakeShareable(new FJsonObject);
            A.Reset();
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.A[1])));
            Bbox_2D_Raw->SetArrayField("A", A);

            B.Reset();
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.B[1])));
            Bbox_2D_Raw->SetArrayField("B", B);

            C.Reset();
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.C[1])));
            Bbox_2D_Raw->SetArrayField("C", C);

            D.Reset();
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.D[1])));
            Bbox_2D_Raw->SetArrayField("D", D);

            singleDetectionObj->SetObjectField("BoundingBox2D_Raw", Bbox_2D_Raw);


            ////////////////////////////////////
            /// 3D DATA ////////////////////////
            ////////////////////////////////////

            ////////////////////////////////////
            /// BOUNDING BOX  //////////////////
            ////////////////////////////////////
           
          TSharedPtr<FJsonObject> Bbox_3D = MakeShareable(new FJsonObject);
            A.Reset(3);
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.A[1])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.A[2])));
            Bbox_3D->SetArrayField("A", A);

            B.Reset(3);
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.B[1])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.B[2])));
            Bbox_3D->SetArrayField("B", B);

            C.Reset(3);
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.C[1])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.C[2])));
            Bbox_3D->SetArrayField("C", C);

            D.Reset(3);
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.D[1])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.D[2])));
            Bbox_3D->SetArrayField("D", D);

            TArray<TSharedPtr<FJsonValue>> E;
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.E[0])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.E[1])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.E[2])));
            Bbox_3D->SetArrayField("E", E);

            TArray<TSharedPtr<FJsonValue>> F;
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.F[0])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.F[1])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.F[2])));
            Bbox_3D->SetArrayField("F", F);

            TArray<TSharedPtr<FJsonValue>> G;
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.G[0])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.G[1])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.G[2])));
            Bbox_3D->SetArrayField("G", G);

            TArray<TSharedPtr<FJsonValue>> H;
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.H[0])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.H[1])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.H[2])));
            Bbox_3D->SetArrayField("H", H);

            singleDetectionObj->SetObjectField("BoundingBox3D_World", Bbox_3D);

            TSharedPtr<FJsonObject> Bbox_3D_Raw = MakeShareable(new FJsonObject);
            A.Reset();
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.A[1])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.A[2])));
            Bbox_3D_Raw->SetArrayField("A", A);

            B.Reset();
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.B[1])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.B[2])));
            Bbox_3D_Raw->SetArrayField("B", B);

            C.Reset();
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.C[1])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.C[2])));
            Bbox_3D_Raw->SetArrayField("C", C);

            D.Reset();
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.D[1])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.D[2])));
            Bbox_3D_Raw->SetArrayField("D", D);

            E.Reset();
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.E[0])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.E[1])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.E[2])));
            Bbox_3D_Raw->SetArrayField("E", E);

            F.Reset();
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.F[0])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.F[1])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.F[2])));
            Bbox_3D_Raw->SetArrayField("F", F);

            G.Reset();
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.G[0])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.G[1])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.G[2])));
            Bbox_3D_Raw->SetArrayField("G", G);

            H.Reset();
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.H[0])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.H[1])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.H[2])));
            Bbox_3D_Raw->SetArrayField("H", H);

            singleDetectionObj->SetObjectField("BoundingBox3D_World_Raw", Bbox_3D_Raw);

            
            ////////////////////////////////////
            // SKELETON   //////////////////////
            ////////////////////////////////////

            TArray<TSharedPtr<FJsonValue>> globalOrientation;
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.X)));
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.Y)));
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.Z)));
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.W)));

            TSharedPtr<FJsonObject> LocalPositionPerJoint = MakeShareable(new FJsonObject);
            TSharedPtr<FJsonObject> LocalOrientationPerJoint = MakeShareable(new FJsonObject);
            TSharedPtr<FJsonObject> Keypoints3D_34 = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.LocalPositionPerJoint.Num(); i++) {
            
                TArray<TSharedPtr<FJsonValue>> joint_position;
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.LocalPositionPerJoint[i].X)));
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.LocalPositionPerJoint[i].Y)));
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.LocalPositionPerJoint[i].Z)));

                LocalPositionPerJoint->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), joint_position);

                TArray<TSharedPtr<FJsonValue>> keypoints;
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].X)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].Y)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].Z)));

                Keypoints3D_34->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), keypoints);

                TArray<TSharedPtr<FJsonValue>> joint_orientation;
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].X)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].Y)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].Z)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].W)));

                LocalOrientationPerJoint->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), joint_orientation);
            }

            TSharedPtr<FJsonObject> Keypoints3D = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.Keypoints3D.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> kp_3d;
                kp_3d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D[i].X)));
                kp_3d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D[i].Y)));
                kp_3d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D[i].Z)));

                Keypoints3D->SetArrayField(enumToString((BODY_PARTS_POSE_18)i), kp_3d);
            }

            TSharedPtr<FJsonObject> Keypoints2D_34 = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.Keypoints2D_34.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> kp_2d_34;
                kp_2d_34.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D_34[i].X)));
                kp_2d_34.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D_34[i].Y)));

                Keypoints2D_34->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), kp_2d_34);
            }

            TSharedPtr<FJsonObject> Keypoints2D = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.Keypoints2D.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> kp_2d;
                kp_2d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D[i].X)));
                kp_2d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D[i].Y)));

                Keypoints2D->SetArrayField(enumToString((BODY_PARTS_POSE_18)i), kp_2d);
            }

            singleDetectionObj->SetObjectField("Keypoints3D_34", Keypoints3D_34);
            singleDetectionObj->SetObjectField("Local_Position_Per_Joint", LocalPositionPerJoint);
            singleDetectionObj->SetObjectField("Local_Orientation_Per_Joint", LocalOrientationPerJoint);
            singleDetectionObj->SetArrayField("Global_Root_Orientation", globalOrientation);
            singleDetectionObj->SetObjectField("Keypoints3D", Keypoints3D);
            singleDetectionObj->SetObjectField("Keypoints2D", Keypoints2D);
            singleDetectionObj->SetObjectField("Keypoints2D_34", Keypoints2D_34);

            
            TSharedRef<FJsonValueObject> singleDetectionValue = MakeShareable(new FJsonValueObject(singleDetectionObj));
            ObjectDetections.Add(singleDetectionValue);
        }
        Detections->SetArrayField("ObjectDetections", ObjectDetections);
        frameObj->SetObjectField("Detections", Detections);

        TSharedRef<FJsonValueObject> frameValue = MakeShareable(new FJsonValueObject(frameObj));
        Frames.Add(frameValue);
    }

    JsonObject->SetArrayField("Frames", Frames);
    
    ////////////////////////////////////
    /// SENSORS DATA ///////////////////
    ////////////////////////////////////

#if SAVE_SENSOR_DATA

    TSharedPtr<FJsonObject> SensorsDataObj = MakeShareable(new FJsonObject);

    TArray<TSharedPtr<FJsonValue>> IMUData;
    TArray<TSharedPtr<FJsonValue>> MagnetometerData;
    TArray<TSharedPtr<FJsonValue>> BarometerData;
    TArray<TSharedPtr<FJsonValue>> GPSData;

    /////////////////// IMU DATA ///////////////////
    for (auto imusensor : data.Sensors.IMUData) {

        if (imusensor.EpochTimeStamp != -1) {
        
            TSharedPtr<FJsonObject> imuObj = MakeShareable(new FJsonObject);
            imuObj->SetNumberField("EpochTimeStamp", imusensor.EpochTimeStamp);
            FVector angVel = imusensor.angularVelocity;
            TArray<TSharedPtr<FJsonValue>> angularVelocityValue;
            angularVelocityValue.Add(MakeShareable(new FJsonValueNumber(angVel.X)));
            angularVelocityValue.Add(MakeShareable(new FJsonValueNumber(angVel.Y)));
            angularVelocityValue.Add(MakeShareable(new FJsonValueNumber(angVel.Z)));
            imuObj->SetArrayField("AngularVelocityRaw", angularVelocityValue);

            FVector linearAccel = imusensor.linearAcceleration;
            TArray<TSharedPtr<FJsonValue>> linearAccelerationValue;
            linearAccelerationValue.Add(MakeShareable(new FJsonValueNumber(linearAccel.X)));
            linearAccelerationValue.Add(MakeShareable(new FJsonValueNumber(linearAccel.Y)));
            linearAccelerationValue.Add(MakeShareable(new FJsonValueNumber(linearAccel.Z)));
            imuObj->SetArrayField("LinearAccelerationRaw", linearAccelerationValue);

            FQuat orientation = imusensor.orientation;
            TArray<TSharedPtr<FJsonValue>> orientationValue;
            orientationValue.Add(MakeShareable(new FJsonValueNumber(orientation.X)));
            orientationValue.Add(MakeShareable(new FJsonValueNumber(orientation.Y)));
            orientationValue.Add(MakeShareable(new FJsonValueNumber(orientation.Z)));
            orientationValue.Add(MakeShareable(new FJsonValueNumber(orientation.W)));
            imuObj->SetArrayField("Orientation", orientationValue);

            TSharedRef<FJsonValueObject> imuValue = MakeShareable(new FJsonValueObject(imuObj));

            IMUData.Add(imuValue);
        }
    }
    if (IMUData.Num() > 0)
        SensorsDataObj->SetArrayField("IMU", IMUData);

    /////////////////// MAGNETOMETER DATA ///////////////////

    for (auto magnetosensor : data.Sensors.magnetometerData) {

        if (magnetosensor.EpochTimeStamp != -1) {
            TSharedPtr<FJsonObject> magnetoObj = MakeShareable(new FJsonObject);

            magnetoObj->SetNumberField("EpochTimeStamp", magnetosensor.EpochTimeStamp);
            FVector magFieldBody = magnetosensor.magneticFieldBody;
            TArray<TSharedPtr<FJsonValue>> magneticFieldBodyValue;
            magneticFieldBodyValue.Add(MakeShareable(new FJsonValueNumber(magFieldBody.X)));
            magneticFieldBodyValue.Add(MakeShareable(new FJsonValueNumber(magFieldBody.Y)));
            magneticFieldBodyValue.Add(MakeShareable(new FJsonValueNumber(magFieldBody.Z)));
            magnetoObj->SetArrayField("MagneticFieldBody", magneticFieldBodyValue);

            //TArray<float> magFieldCovariance = magnetosensor.magneticFieldCovariance;
            //TArray<TSharedPtr<FJsonValue>> magneticFieldCovarianceValue;

            //for (int i = 0; i < magnetosensor.magneticFieldCovariance.Num(); i++) {
            //    magneticFieldCovarianceValue.Add(MakeShareable(new FJsonValueNumber(magFieldCovariance[i])));
            //}

            //magnetoObj->SetArrayField("MagneticFieldCovariance", magneticFieldCovarianceValue);

            TSharedRef<FJsonValueObject> magnetoValue = MakeShareable(new FJsonValueObject(magnetoObj));
            MagnetometerData.Add(magnetoValue);
        }
    }
    if (MagnetometerData.Num() > 0)
        SensorsDataObj->SetArrayField("Magnetometer", MagnetometerData);

    /////////////////// BAROMETER DATA ///////////////////

    for (auto barosensor : data.Sensors.barometerData) {

        if (barosensor.EpochTimeStamp != -1) {
            TSharedPtr<FJsonObject> baroObj = MakeShareable(new FJsonObject);

            float altitude = barosensor.altitude;
            float pressure = barosensor.pressure;
            float qnh = barosensor.qnh;

            baroObj->SetNumberField("EpochTimeStamp", barosensor.EpochTimeStamp);
            baroObj->SetNumberField("Altitude", altitude);
            baroObj->SetNumberField("Pressure", pressure);
            baroObj->SetNumberField("Qnh", qnh);

            TSharedRef<FJsonValueObject> baroValue = MakeShareable(new FJsonValueObject(baroObj));
            BarometerData.Add(baroValue);
        }
    }
    if (BarometerData.Num() > 0)
        SensorsDataObj->SetArrayField("Barometer", BarometerData);

    /////////////////// GPS DATA ///////////////////

    for (auto gpssensor : data.Sensors.GPSData) {

        if (gpssensor.EpochTimeStamp != -1) {
            TSharedPtr<FJsonObject> gpsObj = MakeShareable(new FJsonObject);

            TSharedPtr<FJsonObject> geoPoint = MakeShareable(new FJsonObject);

            float eph = gpssensor.eph;
            float epv = gpssensor.epv;
            FJsonGeoPoint geopoint = gpssensor.geoPoint;
            FVector velocity = gpssensor.velocity;

            gpsObj->SetNumberField("EpochTimeStamp", gpssensor.EpochTimeStamp);

            gpsObj->SetNumberField("Eph", eph);
            gpsObj->SetNumberField("Epv", epv);

            TArray<TSharedPtr<FJsonValue>> velocityValue;
            velocityValue.Add(MakeShareable(new FJsonValueNumber(velocity.X)));
            velocityValue.Add(MakeShareable(new FJsonValueNumber(velocity.Y)));
            velocityValue.Add(MakeShareable(new FJsonValueNumber(velocity.Z)));
            gpsObj->SetArrayField("Velocity", velocityValue);

            geoPoint->SetNumberField("Latitude", geopoint.latitude);
            geoPoint->SetNumberField("Longitude", geopoint.longitude);
            geoPoint->SetNumberField("Altitude", geopoint.altitude);

            gpsObj->SetObjectField("Geopoint", geoPoint);

            TSharedRef<FJsonValueObject> gpsValue = MakeShareable(new FJsonValueObject(gpsObj));
            GPSData.Add(gpsValue);
        }
    }
    if (GPSData.Num() > 0)
        SensorsDataObj->SetArrayField("GPS", GPSData);

    JsonObject->SetObjectField("Sensors", SensorsDataObj);
#endif

    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
    FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

    return OutputString;
}
