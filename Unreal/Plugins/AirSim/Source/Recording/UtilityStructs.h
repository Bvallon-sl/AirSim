#pragma once

#include <string>
#include "CoreMinimal.h"
#include "common/Common.hpp"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

#include "UtilityStructs.generated.h"

#define IS_MICROSECONDS true
#define INVALID_VALUE -999.0
#define BBOX_MINIMUM_VOLUME 0.005
#define MAX_DISTANCE_METER 40

#define __cx 960
#define __cy 540
#define __fx 1400
#define __fy 1400


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

//Name of bones of the skeleton rig //for carla/eric/... all unity's models
/* const TArray<FName> targetBone = {
    "hip",
    "spine_01",
    "spine_02",
    "neck",
    "shoulder_l",
    "upperarm_l",
    "lowerarm_l",
    "hand_l",
    "not_found", //left hand
    "middle_end_l",
    "thumb_end_l",
    "shoulder_r",
    "upperarm_r",
    "lowerarm_r",
    "hand_r",
    "not_found", //right hand
    "middle_end_r",
    "thumb_end_r",
    "upperleg_l",
    "lowerleg_l",
    "foot_l",
    "foot_end_l",
    "upperleg_r",
    "lowerleg_r",
    "foot_r",
    "foot_end_r",
    "head",
    "not_found", //nose
    "eye_l",
    "not_found", //left ear
    "eye_r",
    "not_found", //right ear
    "not_found", //left heel
    "not_found"  //right heel
};*/

//Name of bones of the skeleton rig // for mixamo's models
const TArray<FName> targetBone = {
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


static FVector warpPoint_(FVector pt_curr, const FMatrix path, float scale = 1)
{
    FVector proj3D;
    const auto p_path = path.M;

    proj3D.X = ((pt_curr.X * p_path[0][0]) + (pt_curr.Y * p_path[0][1]) + (pt_curr.Z * p_path[0][2]) + p_path[0][3]) * scale;
    proj3D.Y = ((pt_curr.X * p_path[1][0]) + (pt_curr.Y * p_path[1][1]) + (pt_curr.Z * p_path[1][2]) + p_path[1][3]) * scale;
    proj3D.Z = ((pt_curr.X * p_path[2][0]) + (pt_curr.Y * p_path[2][1]) + (pt_curr.Z * p_path[2][2]) + p_path[2][3]) * scale;
    return proj3D;
}

static FVector convertFromImageToUnityCoordinateSystem(FVector in)
{
    FVector out(in.X, - in.Y, in.Z);
    out /= 100;

    return out;
}

static FVector convertFromUUToUnityCoordinateSystem(FVector in)
{
    FVector out(in.Y, in.Z, in.X);
    out /= 100;

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
    //FVector trans = out.GetLocation() / 100;
    //out.SetLocation(trans);

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
    //in = camPose.TransformPosition(in);
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

        transform.SetFromMatrix(mat);

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
        A = FVector::ZeroVector;
        B = FVector::ZeroVector;
        C = FVector::ZeroVector;
        D = FVector::ZeroVector;
        E = FVector::ZeroVector;
        F = FVector::ZeroVector;
        G = FVector::ZeroVector;
        H = FVector::ZeroVector;
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
        A = FVector2D::ZeroVector;
        B = FVector2D::ZeroVector;
        C = FVector2D::ZeroVector;
        D = FVector2D::ZeroVector;
    }

};

USTRUCT()
struct FJsonSkeleton3DData
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FQuat global_root_orientation;

    UPROPERTY()
    TArray<FVector> keypoints;

    UPROPERTY()
    TArray<FVector> local_position_per_joint;

    UPROPERTY()
    TArray<FQuat> local_orientation_per_joint;
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
    FJsonSkeleton3DData Skeleton3D_Camera_Raw;
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
    TArray<FJsonFrameData> Frames;
};


static FString SerializeJson(FJsonDataSet data)
{
    FString OutputString;

    // create a Json object and add a string field
    TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

    // METADATA
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

    TArray<TSharedPtr<FJsonValue>> Frames;
    for(auto frame : data.Frames) {

        TSharedPtr<FJsonObject> frameObj = MakeShareable(new FJsonObject);
        frameObj->SetStringField("ImageFileName", frame.ImageFileName);
        frameObj->SetNumberField("FrameIndex", frame.FrameIndex);
        frameObj->SetNumberField("EpochTimeStamp", frame.EpochTimeStamp);

        ////////////////////////////////////
        // ODOMETRY DATA ///////////////////
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
        // DETECTION DATA //////////////////
        ////////////////////////////////////
        TSharedPtr<FJsonObject> Detections = MakeShareable(new FJsonObject);

        TArray<TSharedPtr<FJsonValue>> ObjectDetections;
        for (auto detection : frame.Detections.ObjectDetections) 
        {
            TSharedPtr<FJsonObject> singleDetectionObj = MakeShareable(new FJsonObject);

            singleDetectionObj->SetNumberField("ObjectID", detection.ObjectID);
            singleDetectionObj->SetNumberField("ObjectType", detection.ObjectType);

            TArray<TSharedPtr<FJsonValue>> position3D;
            position3D.Add(MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[0])));
            position3D.Add(MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[1])));
            position3D.Add(MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[2])));
            singleDetectionObj->SetArrayField("Position3D_World_Floor", position3D);

            TArray<TSharedPtr<FJsonValue>> velocity3D;
            velocity3D.Add(MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[0])));
            velocity3D.Add(MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[1])));
            velocity3D.Add(MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[2])));
            singleDetectionObj->SetArrayField("Velocity3D_MPS", velocity3D);

            TArray<TSharedPtr<FJsonValue>> dimensions3D;
            dimensions3D.Add(MakeShareable(new FJsonValueNumber(detection.Dimensions3D[0])));
            dimensions3D.Add(MakeShareable(new FJsonValueNumber(detection.Dimensions3D[1])));
            dimensions3D.Add(MakeShareable(new FJsonValueNumber(detection.Dimensions3D[2])));
            singleDetectionObj->SetArrayField("Dimensions3D", dimensions3D);

            ////////////////////////////////////
            // 2D DATA ////// //////////////////
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
            // 3D DATA /////////////////////////
            ////////////////////////////////////

            ////////////////////////////////////
            // BOUNDING BOX  ///////////////////
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

            TSharedPtr<FJsonObject> Skeleton_Raw_3D = MakeShareable(new FJsonObject);

            TArray<TSharedPtr<FJsonValue>> globalOrientationPerJoint;
            globalOrientationPerJoint.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.global_root_orientation.X)));
            globalOrientationPerJoint.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.global_root_orientation.Y)));
            globalOrientationPerJoint.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.global_root_orientation.Z)));
            globalOrientationPerJoint.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.global_root_orientation.W)));
            Skeleton_Raw_3D->SetArrayField("Global_Root_Orientation", globalOrientationPerJoint);

            TArray<TSharedPtr<FJsonValue>> LocalPositionPerJoint;
            TArray<TSharedPtr<FJsonValue>> LocalOrientationPerJoint;
            TArray<TSharedPtr<FJsonValue>> Keypoints;
            for (int i = 0; i < detection.Skeleton3D_Camera_Raw.local_position_per_joint.Num(); i++) {
            
                TArray<TSharedPtr<FJsonValue>> joint_position;
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.local_position_per_joint[i].X)));
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.local_position_per_joint[i].Y)));
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.local_position_per_joint[i].Z)));

                LocalPositionPerJoint.Add(MakeShareable(new FJsonValueArray(joint_position)));

                TArray<TSharedPtr<FJsonValue>> keypoints;
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.keypoints[i].X)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.keypoints[i].Y)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.keypoints[i].Z)));

                Keypoints.Add(MakeShareable(new FJsonValueArray(keypoints)));

                TArray<TSharedPtr<FJsonValue>> joint_orientation;
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.local_orientation_per_joint[i].X)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.local_orientation_per_joint[i].Y)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.local_orientation_per_joint[i].Z)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.Skeleton3D_Camera_Raw.local_orientation_per_joint[i].W)));

                LocalOrientationPerJoint.Add(MakeShareable(new FJsonValueArray(joint_orientation)));
            }

            Skeleton_Raw_3D->SetArrayField("Local_Position_Per_Joint", LocalPositionPerJoint);
            Skeleton_Raw_3D->SetArrayField("Keypoints", Keypoints);
            Skeleton_Raw_3D->SetArrayField("Local_Orientation_Per_Joint", LocalOrientationPerJoint);

            singleDetectionObj->SetObjectField("Skeleton3D_Camera_Raw", Skeleton_Raw_3D);

            TSharedRef<FJsonValueObject> singleDetectionValue = MakeShareable(new FJsonValueObject(singleDetectionObj));
            ObjectDetections.Add(singleDetectionValue);
        }
        Detections->SetArrayField("ObjectDetections", ObjectDetections);
        frameObj->SetObjectField("Detections", Detections);

        TSharedRef<FJsonValueObject> frameValue = MakeShareable(new FJsonValueObject(frameObj));
        Frames.Add(frameValue);
    }

    JsonObject->SetArrayField("Frames", Frames);

    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
    FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

    return OutputString;
}

static FJsonBoundingBox3DData compute3DBoxesFrom2D(APIPCamera* camera, FJsonBoundingBox2DData box2D, msr::airlib::DetectionInfo_UU detection)
{
    FJsonBoundingBox3DData box_3d;
    FTransform cam_pose = convertFromUnityToImageCoordinateSystem(convertFromUUToUnityCoordinateSystem(camera->getUUPose())); // to image

    float left = box2D.A.X;
    float top = box2D.A.Y;
    float right = box2D.C.X;
    float bot = box2D.C.Y;

    float root_depth = convertFromUUToImageCoordinateSystem(detection.relative_transform.GetTranslation()).Z; 

    FVector A, B, C, D;

    A.X = ((left - __cx) / __fx) * root_depth;
    A.Y = ((top - __cy) / __fy) * root_depth;
    A.Z = root_depth;
    B.X = ((right - __cx) / __fx) * root_depth;
    B.Y = ((top - __cy) / __fy) * root_depth;
    B.Z = root_depth;
    C.X = ((right - __cx) / __fx) * root_depth;
    C.Y = ((bot - __cy) / __fy) * root_depth;
    C.Z = root_depth;
    D.X = ((left - __cx) / __fx) * root_depth;
    D.Y = ((bot - __cy) / __fy) * root_depth;
    D.Z = root_depth;

    float AB_dist = FVector::Distance(A, B);
    float far_plane_depth = root_depth + AB_dist * 0.5f;
    float close_plane_depth = root_depth - AB_dist * 0.5f;

    FVector A1, B1, C1, D1;
    FVector A2, B2, C2, D2;

    A1 = A;
    A1.Z = close_plane_depth;
    B1 = B;
    B1.Z = close_plane_depth;
    C1 = C;
    C1.Z = close_plane_depth;
    D1 = D;
    D1.Z = close_plane_depth;

    A2 = A;
    A2.Z = far_plane_depth;
    B2 = B;
    B2.Z = far_plane_depth;
    C2 = C;
    C2.Z = far_plane_depth;
    D2 = D;
    D2.Z = far_plane_depth;

    box_3d.A = A1;
    box_3d.B = A2;
    box_3d.C = B2;
    box_3d.D = B1;
    box_3d.E = D1;
    box_3d.F = D2;
    box_3d.G = C2;
    box_3d.H = C1;

    FVector root_position_tmp = convertFromUUToImageCoordinateSystem(detection.relative_transform.GetTranslation());
    FTransform tf_gravity =  cam_pose;

    tf_gravity.SetTranslation(FVector::ZeroVector);

    // Remove yaw
    FVector vec = tf_gravity.GetRotation().Euler();
    vec.Y = 0;
    tf_gravity.SetRotation(FQuat::MakeFromEuler(vec));
    ///

    FMatrix mat_gravity = tf_gravity.Inverse().ToMatrixWithScale().GetTransposed();
    
    box_3d.A -= root_position_tmp;
    box_3d.A = warpPoint_(box_3d.A, mat_gravity);
    box_3d.A += root_position_tmp;
    box_3d.A = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.A));

    box_3d.B -= root_position_tmp;
    box_3d.B = warpPoint_(box_3d.B, mat_gravity);
    box_3d.B += root_position_tmp;
    box_3d.B = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.B));

    box_3d.C -= root_position_tmp;
    box_3d.C = warpPoint_(box_3d.C, mat_gravity);
    box_3d.C += root_position_tmp;
    box_3d.C = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.C));

    box_3d.D -= root_position_tmp;
    box_3d.D = warpPoint_(box_3d.D, mat_gravity);
    box_3d.D += root_position_tmp;
    box_3d.D = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.D));

    box_3d.E -= root_position_tmp;
    box_3d.E = warpPoint_(box_3d.E, mat_gravity);
    box_3d.E += root_position_tmp;
    box_3d.E = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.E));

    box_3d.F -= root_position_tmp;
    box_3d.F = warpPoint_(box_3d.F, mat_gravity);
    box_3d.F += root_position_tmp;
    box_3d.F = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.F));

    box_3d.G -= root_position_tmp;
    box_3d.G = warpPoint_(box_3d.G, mat_gravity);
    box_3d.G += root_position_tmp;
    box_3d.G = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.G));

    box_3d.H -= root_position_tmp;
    box_3d.H = warpPoint_(box_3d.H, mat_gravity);
    box_3d.H += root_position_tmp;
    box_3d.H = convertFromImageToUnityCoordinateSystem(camToWorld(cam_pose, box_3d.H));

    return box_3d;
}
