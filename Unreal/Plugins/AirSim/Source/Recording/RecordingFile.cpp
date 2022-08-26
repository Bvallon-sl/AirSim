#include "RecordingFile.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/FileHelper.h"
#include <sstream>
#include "ImageUtils.h"
#include "common/ClockFactory.hpp"
#include "common/common_utils/FileSystem.hpp"

void RecordingFile::appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
                                 std::vector<msr::airlib::DetectionInfo_UU>& detections,
                                 msr::airlib::VehicleSimApiBase* vehicle_sim_api, int64 timestamp)
{

    int64 ts = IS_MICROSECONDS ? (timestamp / 1000) : timestamp;

    std::vector<ImageToSave> imagesFrame;
    cv::Mat mask;
    imagesFrame.clear();

    for (auto i = 0; i < responses.size(); ++i) {
        const auto& response = responses.at(i);
        std::string camera_name = response.camera_name;

        if (response.image_type == msr::airlib::ImageCaptureBase::ImageType::DepthPlanar) {
            camera_name = "LeftDepth";
        }
        else if (response.image_type == msr::airlib::ImageCaptureBase::ImageType::Segmentation) {
            camera_name = "ObjectMask";
        }
        //build image file name
        std::ostringstream image_file_name;
        image_file_name << camera_name << "_" << ts << "_" << std::setw(5) << std::setfill('0') << frameIndex;
        //image_file_name << camera_name << "_" << std::setw(3) << std::setfill('0') << frameIndex;
        std::string extension = ".png";
        image_file_name << extension;

        std::string image_full_file_path = common_utils::FileSystem::combine(image_path_, image_file_name.str());
        ImageToSave imageToSave;
        imageToSave.height = response.height;
        imageToSave.width = response.width;
        imageToSave.image_type = (int)response.image_type;
        imageToSave.image_full_file_path = image_full_file_path;

        if (response.image_type == msr::airlib::ImageCaptureBase::ImageType::DepthPlanar) {
            imageToSave.image_data_float = std::move(response.image_data_float);
        }
        else if (response.image_type == msr::airlib::ImageCaptureBase::ImageType::Segmentation) {
            cv::Mat mat = cv::Mat(response.height, response.width, CV_8UC3, (unsigned char*)response.image_data_uint8.data());
            mask = cv::Mat(response.height, response.width, CV_8UC1);
            mat.convertTo(mask, CV_8UC1);
            cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
            imageToSave.mask = mask;

            //imageToSave.image_data_uint8 = response.image_data_uint8;
        }
        else if (response.image_type == msr::airlib::ImageCaptureBase::ImageType::Scene) {
            imageToSave.image_data_uint8 = std::move(response.image_data_uint8);
        }

        imagesFrame.push_back(imageToSave);
    }


    imagesSavingThread->images_queue.Enqueue(imagesFrame);
    timestamps_.push_back(ts);

    //write to JSON file
    PawnSimApi* pawn = static_cast<PawnSimApi*>(vehicle_sim_api);
    FJsonFramePoseData trackedPoseData;
    trackedPoseData.WorldPose = FJsonMatrix4x4(pawn->getCamera("Left")->GetActorTransform());

    FJsonFrameData fdata;
    fdata.EpochTimeStamp = ts;

    std::ostringstream fileName;
    fileName // << "SBS_"
        << "_"
        << fdata.EpochTimeStamp 
        << "_" 
        << std::setw(5) 
        << std::setfill('0') 
        << frameIndex
        << ".png";
    fdata.ImageFileName = UTF8_TO_TCHAR(fileName.str().c_str());
    fdata.FrameIndex = frameIndex;
    fdata.TrackedPose = trackedPoseData;

    FJsonFrameDetections frameDetections;
    if (mask.empty()) {
        //UE_LOG(LogTemp, Warning, TEXT("mask empty"));
    }
    else {


        logDetections(pawn, responses[0].height, mask, detections, frameDetections); // Compute Detections and store them in the Struct. Assuming all cameras have the same res.
    }

    fdata.Detections = frameDetections;

    data.Frames.Add(fdata);

    frameIndex++;
}

void RecordingFile::appendSensorsData(msr::airlib::VehicleSimApiBase* vehicle_sim_api) 
{

    PawnSimApi* pawn = static_cast<PawnSimApi*>(vehicle_sim_api);

    if (msr::airlib::AirSimSettings::singleton().simmode_name == "Multirotor") {

        FJsonIMUData imuJsonData;
        FJsonGPSData gpsJsonData;
        FJsonBarometerData baroJsonData;
        FJsonMagnometerData magnetoJsonData;

        FTransform cam_pose = pawn->getCamera("Left")->GetActorTransform();

        UE_LOG(LogTemp, Warning, TEXT("get camera actor pose : %s"), *cam_pose.GetLocation().ToString());

        cam_pose.SetLocation(FVector::ZeroVector);

        MultirotorPawnSimApi* vehiclePawn = static_cast<MultirotorPawnSimApi*>(pawn);
        auto vehicleAPI = vehiclePawn->getVehicleApi();


        if (vehicleAPI->findSensorByName("Imu", msr::airlib::SensorBase::SensorType::Imu)) 
        {
            msr::airlib::ImuBase::Output imuData = vehicleAPI->getImuData("Imu");
            //UE_LOG(LogTemp, Warning, TEXT("IMU TS     %llu"), imuData.time_stamp);

            FVector angularVelocity = FVector(imuData.angular_velocity.x(), imuData.angular_velocity.y(), imuData.angular_velocity.z());
            imuJsonData.EpochTimeStamp = IS_MICROSECONDS ? (imuData.time_stamp / 1000) : imuData.time_stamp;
            imuJsonData.angularVelocity = convertFromUUToUnityCoordinateSystem(worldToCam(cam_pose, angularVelocity), false);

            FVector linearAcceleration = FVector(imuData.linear_acceleration.x(), imuData.linear_acceleration.y(), imuData.linear_acceleration.z());
            imuJsonData.linearAcceleration = convertFromUUToUnityCoordinateSystem(worldToCam(cam_pose, linearAcceleration), false);

            FQuat IMUOrientation = FQuat(imuData.orientation.x(), imuData.orientation.y(), imuData.orientation.z(), imuData.orientation.w());
            imuJsonData.orientation = convertFromUUToUnityCoordinateSystem(FTransform(worldToCam(cam_pose, IMUOrientation))).GetRotation();
        }

        if (vehicleAPI->findSensorByName("Barometer", msr::airlib::SensorBase::SensorType::Barometer)) 
        {
            msr::airlib::BarometerBase::Output barometerData = vehicleAPI->getBarometerData("Barometer");
            //UE_LOG(LogTemp, Warning, TEXT("Baro TS    %llu"), barometerData.time_stamp);

            baroJsonData.EpochTimeStamp = IS_MICROSECONDS ? (barometerData.time_stamp / 1000) : barometerData.time_stamp;
            baroJsonData.altitude = barometerData.altitude;
            baroJsonData.pressure = barometerData.pressure;
            baroJsonData.qnh = barometerData.qnh;
        }
        else {
            baroJsonData.EpochTimeStamp = 0;
            baroJsonData.altitude = 0;
            baroJsonData.pressure = 0;
            baroJsonData.qnh = 0;
        }

        if (vehicleAPI->findSensorByName("Magnetometer", msr::airlib::SensorBase::SensorType::Magnetometer)) 
        {
            msr::airlib::MagnetometerSimple::Output magnetometerData = vehicleAPI->getMagnetometerData("Magnetometer");
            UE_LOG(LogTemp, Warning, TEXT("Magneto TS %llu"), magnetometerData.time_stamp);

            FVector magneticBody = FVector(magnetometerData.magnetic_field_body.x(), magnetometerData.magnetic_field_body.y(), magnetometerData.magnetic_field_body.z());
            magnetoJsonData.EpochTimeStamp = IS_MICROSECONDS ? (magnetometerData.time_stamp / 1000) : magnetometerData.time_stamp;
            magnetoJsonData.magneticFieldBody = convertFromUUToUnityCoordinateSystem(worldToCam(cam_pose, magneticBody), false);

            //UE_LOG(LogTemp, Warning, TEXT("Size %i"), magnetometerData.magnetic_field_covariance.size());
            //for (int i = 0; i < magnetometerData.magnetic_field_covariance.size(); i++) {
            //    magnetoJsonData.magneticFieldCovariance.Add(magnetometerData.magnetic_field_covariance[i]);
            //}
        }

        if (vehicleAPI->findSensorByName("Gps", msr::airlib::SensorBase::SensorType::Gps))
        {
            msr::airlib::GpsSimple::Output gpsData = vehicleAPI->getGpsData("Gps");
            //UE_LOG(LogTemp, Warning, TEXT("GPS TS     %llu"), gpsData.time_stamp);
            //UE_LOG(LogTemp, Warning, TEXT("GPS UTC    %llu"), gpsData.gnss.time_utc);
        
            gpsJsonData.EpochTimeStamp = IS_MICROSECONDS ? (gpsData.time_stamp / 1000) : gpsData.time_stamp;
            gpsJsonData.eph = gpsData.gnss.eph;
            gpsJsonData.epv = gpsData.gnss.epv;
            gpsJsonData.velocity = FVector(gpsData.gnss.velocity.x(), gpsData.gnss.velocity.y(), gpsData.gnss.velocity.z());
            gpsJsonData.geoPoint.altitude = gpsData.gnss.geo_point.altitude;
            gpsJsonData.geoPoint.latitude = gpsData.gnss.geo_point.latitude;
            gpsJsonData.geoPoint.longitude = gpsData.gnss.geo_point.longitude;
        }

        data.Sensors.IMUData.Add(imuJsonData);
        data.Sensors.magnetometerData.Add(magnetoJsonData);
        data.Sensors.barometerData.Add(baroJsonData);
        data.Sensors.GPSData.Add(gpsJsonData);
    }
}

FJsonDataSet RecordingFile::getDataSet(){
    return data;
}

FJsonBoundingBox2DData RecordingFile::computeBbox2D(FJsonBoundingBox2DData raw_detection, cv::Mat mask, int id)
{
    int height = mask.rows;
    int width = mask.cols;

    int left = height;
    int top = width;
    int right = 0;
    int bot = 0;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (id == mask.at<unsigned char>(i, j)) {
                left = std::min(left, j);
                top = std::min(top, i);

                right = std::max(right, j);
                bot = std::max(bot, i);
            }
        }
    }

    FJsonBoundingBox2DData box2d;
    int bot_margin = 10;
    box2d.A = raw_detection.A + FVector2D(left, top);
    box2d.B = raw_detection.A + FVector2D(right, top);
    box2d.C = raw_detection.A + FVector2D(right, bot + bot_margin);
    box2d.D = raw_detection.A + FVector2D(left, bot + bot_margin);

    return box2d;
}

FJsonBoundingBox3DData RecordingFile::compute3DBoxesFrom2D(msr::airlib::AirSimSettings::CaptureSetting captureSettings, FTransform cam_pose, FJsonBoundingBox2DData box2D, msr::airlib::DetectionInfo_UU detection)
{
    cam_pose = convertFromUnityToImageCoordinateSystem(convertFromUUToUnityCoordinateSystem(cam_pose));
    FJsonBoundingBox3DData box_3d;

    float left = box2D.A.X;
    float top = box2D.A.Y;
    float right = box2D.C.X;
    float bot = box2D.C.Y;

    float root_depth = convertFromUUToImageCoordinateSystem(detection.relative_transform.GetTranslation()).Z;

    float __fx = captureSettings.fx;
    float __fy = captureSettings.fy;
    float __cx = captureSettings.cx;
    float __cy = captureSettings.cy;

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
    FTransform tf_gravity = cam_pose;

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

FJsonSkeletonData RecordingFile::retrieveSkeletonData(msr::airlib::AirSimSettings::CaptureSetting captureSettings, FTransform camPose, msr::airlib::DetectionInfo_UU& detection)
{
    FJsonSkeletonData skeleton_raw;
    TArray<FTransform> boneSpaceTransforms = detection.skeletal_mesh->GetBoneSpaceTransforms();
    FVector start = camPose.GetLocation();

    float fx = captureSettings.fx;
    float fy = captureSettings.fy;
    float cx = captureSettings.cx;
    float cy = captureSettings.cy;

    // Keypoint 34
    for (int idx = 0; idx < (int)BODY_PARTS_POSE_34::LAST; idx++) {

        if (targetBone_34[idx] == "not_found") {

            skeleton_raw.LocalPositionPerJoint.Add(FVector(INVALID_VALUE,INVALID_VALUE,INVALID_VALUE));
            skeleton_raw.LocalOrientationPerJoint.Add(FQuat(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));

            skeleton_raw.Keypoints3D_34.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
            skeleton_raw.Keypoints2D_34.Add(FVector2D(INVALID_VALUE, INVALID_VALUE));

            continue;
        }

        FVector keypoint;
        if (idx == (int)BODY_PARTS_POSE_34::PELVIS) { // The PELIVS kp is not at the same position in the SDK and in UE. Create a Fake kp at the correct position.
            keypoint = (detection.skeletal_mesh->GetBoneLocation(targetBone_34[(int)BODY_PARTS_POSE_34::LEFT_HIP], EBoneSpaces::WorldSpace) + detection.skeletal_mesh->GetBoneLocation(targetBone_34[(int)BODY_PARTS_POSE_34::RIGHT_HIP], EBoneSpaces::WorldSpace)) / 2;
        }
        else {
            keypoint = detection.skeletal_mesh->GetBoneLocation(targetBone_34[idx], EBoneSpaces::WorldSpace);
        }

        // RAY CAST FOR OCCLUSION
        FHitResult Hit;

        FVector end = keypoint;
        FCollisionQueryParams TraceParams(FName(TEXT("")), true);

        bool raycast = detection.actor->GetWorld()->LineTraceSingleByChannel(OUT Hit, start, end, ECollisionChannel::ECC_Visibility, TraceParams);

        // See what if anything has been hit and return what
        AActor* ActorHit = Hit.GetActor();

        float dist = FVector::Distance(keypoint, Hit.ImpactPoint);

        if (ActorHit && ActorHit->GetName() == detection.actor->GetName() && dist < occlusion_thresholds[idx]) { // if the ray cast hits the correct joint and the correct actor, then it's visible

            FVector local_position = boneSpaceTransforms[detection.skeletal_mesh->GetBoneIndex(targetBone_34[idx])].GetTranslation();
            FQuat local_orientation = boneSpaceTransforms[detection.skeletal_mesh->GetBoneIndex(targetBone_34[idx])].GetRotation();

            skeleton_raw.LocalPositionPerJoint.Add(convertFromUUToUnityCoordinateSystem(worldToCam(camPose, local_position)));
            skeleton_raw.LocalOrientationPerJoint.Add(convertFromUUToUnityCoordinateSystem(FTransform(worldToCam(camPose, local_orientation))).GetRotation()); /// ?????????

            skeleton_raw.Keypoints3D_34.Add(convertFromUUToUnityCoordinateSystem(worldToCam(camPose, keypoint)));

            FVector2D keypoint_2d = projectPoint_(convertFromUUToImageCoordinateSystem(keypoint), fx, fy, cx, cy);

            skeleton_raw.Keypoints2D_34.Add(keypoint_2d);
        }
        else { // it's occluded

            skeleton_raw.LocalPositionPerJoint.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
            skeleton_raw.LocalOrientationPerJoint.Add(FQuat(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));

            skeleton_raw.Keypoints3D_34.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));

            skeleton_raw.Keypoints2D_34.Add(FVector2D(INVALID_VALUE, INVALID_VALUE));
        }       
    }

    FQuat root_orientation = detection.skeletal_mesh->GetBoneQuaternion(targetBone_34[(int)BODY_PARTS_POSE_34::PELVIS], EBoneSpaces::WorldSpace);
    skeleton_raw.GlobalRootOrientation = convertFromUUToUnityCoordinateSystem(FTransform(worldToCam(camPose, root_orientation))).GetRotation();

    // Keypoints 18
    for (int idx = 0; idx < (int)BODY_PARTS_POSE_18::LAST; idx++) {
        if (targetBone_18[idx] == "not_found") {

            skeleton_raw.Keypoints3D.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
            skeleton_raw.Keypoints2D.Add(FVector2D(INVALID_VALUE, INVALID_VALUE));
            continue;
        }

        FVector keypoint = detection.skeletal_mesh->GetBoneLocation(targetBone_18[idx], EBoneSpaces::WorldSpace);
        // RAY CAST FOR OCCLUSION
        FHitResult Hit;

        FVector end = keypoint;
        FCollisionQueryParams TraceParams(FName(TEXT("")), true);

        bool raycast = detection.actor->GetWorld()->LineTraceSingleByChannel(OUT Hit, start, end, ECollisionChannel::ECC_Visibility, TraceParams);

        // See what if anything has been hit and return what
        AActor* ActorHit = Hit.GetActor();

        float dist = FVector::Distance(keypoint, Hit.ImpactPoint);

        if (ActorHit && ActorHit->GetName() == detection.actor->GetName() && dist < occlusion_thresholds[idx]) { // if the ray cast hits the correct joint and the correct actor, then it's visible
            skeleton_raw.Keypoints3D.Add(convertFromUUToUnityCoordinateSystem(worldToCam(camPose, keypoint)));

            FVector2D keypoint_2d = projectPoint_(convertFromUUToImageCoordinateSystem(keypoint), fx, fy, cx, cy);

            skeleton_raw.Keypoints2D.Add(keypoint_2d);
        }
        else { // it's occluded
            skeleton_raw.Keypoints3D.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
            skeleton_raw.Keypoints2D.Add(FVector2D(INVALID_VALUE, INVALID_VALUE));

        }       
    }
    return skeleton_raw;
}

void RecordingFile::logDetections(PawnSimApi* pawn, int image_height, cv::Mat& mask, std::vector<msr::airlib::DetectionInfo_UU>& detections, FJsonFrameDetections& frameDetections)
{
    FTransform cam_pose = pawn->getUUPose(); // assume left cam is at origin in vehicle frame
    auto height = image_height;
    int offset = 100;

    msr::airlib::AirSimSettings::CaptureSetting captureSettings = msr::airlib::AirSimSettings::singleton().getVehicleSetting(pawn->getVehicleName())->cameras.at("Left").capture_settings.at(0);

    float __fx = captureSettings.fx;
    float __fy = captureSettings.fy;
    float __cx = captureSettings.cx;
    float __cy = captureSettings.cy;

    for (int idx_detection = 0; idx_detection < detections.size(); idx_detection++) {
        auto detection = detections[idx_detection];
        std::string person_name = detections[idx_detection].name;
        int ObjectID = std::atoi(person_name.substr(7, person_name.size() - 7).c_str()); // person_XX

        FJsonSingleDetection singleDetection;
        singleDetection.ObjectID = ObjectID;
        singleDetection.ObjectType = 0; // Only compatible with Person for the moment.

        FVector position = (detection.box3D.max + detection.box3D.min) / 2;
        position.Z = detection.box3D.min.Z;
        singleDetection.Position3D_World_Floor = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, FVector(position)));

        FVector velocity = FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE); //TO DO
        singleDetection.Velocity3D_MPS = velocity; //convertFromUUToUnityCoordinateSystem(velocity);

        FJsonBoundingBox2DData bbox2D_raw;
        bbox2D_raw.A = FVector2D(detection.box2D.min.X, detection.box2D.min.Y);
        bbox2D_raw.B = FVector2D(detection.box2D.max.X, detection.box2D.min.Y);
        bbox2D_raw.C = FVector2D(detection.box2D.max.X, FMath::Clamp<int>(detection.box2D.max.Y + offset, detection.box2D.max.Y, height));
        bbox2D_raw.D = FVector2D(detection.box2D.min.X, FMath::Clamp<int>(detection.box2D.max.Y + offset, detection.box2D.max.Y, height));

        cv::Mat mask_id;
        cv::Mat(mask, cv::Rect(bbox2D_raw.A.X, bbox2D_raw.A.Y, (int)std::abs(bbox2D_raw.B.X - bbox2D_raw.A.X), (int)std::abs(bbox2D_raw.D.Y - bbox2D_raw.A.Y))).copyTo(mask_id);

        FJsonBoundingBox2DData bbox2D = computeBbox2D(bbox2D_raw, mask_id, ObjectID);
        singleDetection.BoundingBox2D = bbox2D;

        singleDetection.BoundingBox2D_Raw = bbox2D_raw;

        FVector max = detection.box3D.max;
        FVector min = detection.box3D.min;

        FVector dimensions = FVector(std::abs(max.X - min.X), std::abs(max.Y - min.Y), std::abs(max.Z - min.Z));
        singleDetection.Dimensions3D = convertFromUUToUnityCoordinateSystem(dimensions);

        FJsonBoundingBox3DData bbox3D_raw;
        bbox3D_raw.A = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, FVector(min.X, min.Y, max.Z)));
        bbox3D_raw.B = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, FVector(min.X, max.Y, max.Z)));
        bbox3D_raw.C = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, max));
        bbox3D_raw.D = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, FVector(max.X, min.Y, max.Z)));
        bbox3D_raw.E = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, min));
        bbox3D_raw.F = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, FVector(min.X, max.Y, min.Z)));
        bbox3D_raw.G = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, FVector(max.X, max.Y, min.Z)));
        bbox3D_raw.H = convertFromUUToUnityCoordinateSystem(camToWorld(cam_pose, FVector(max.X, min.Y, min.Z)));

        singleDetection.BoundingBox3D_World_Raw = bbox3D_raw;

        FJsonBoundingBox3DData bbox3D = compute3DBoxesFrom2D(captureSettings, cam_pose, bbox2D, detection); //bbox3D;

        singleDetection.BoundingBox3D_World = bbox3D;

        FJsonSkeletonData skeleton3D_raw = retrieveSkeletonData(captureSettings, cam_pose, detection);

        singleDetection.GlobalRootOrientation = skeleton3D_raw.GlobalRootOrientation;
        singleDetection.Keypoints2D = skeleton3D_raw.Keypoints2D;
        singleDetection.Keypoints2D_34 = skeleton3D_raw.Keypoints2D_34;
        singleDetection.Keypoints3D = skeleton3D_raw.Keypoints3D;
        singleDetection.Keypoints3D_34 = skeleton3D_raw.Keypoints3D_34;
        singleDetection.LocalOrientationPerJoint = skeleton3D_raw.LocalOrientationPerJoint;
        singleDetection.LocalPositionPerJoint = skeleton3D_raw.LocalPositionPerJoint;

        frameDetections.ObjectDetections.Push(singleDetection);
    }
}

void RecordingFile::appendColumnHeader(const std::string& header_columns)
{
    writeString(header_columns + "ImageFile" + "\n");
}

void RecordingFile::createFile(const std::string& file_path, const std::string& header_columns)
{
    try {
        closeFile();

        IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
        log_file_handle_ = platform_file.OpenWrite(*FString(file_path.c_str()));
        appendColumnHeader(header_columns);
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString(std::string("createFile Failed for ") + file_path, ex.what(), LogDebugLevel::Failure);
    }
}

void RecordingFile::createJsonFile(const std::string& file_path)
{
    try {
        closeFile();

        IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
        log_file_handle_ = platform_file.OpenWrite(*FString(file_path.c_str()));

        if (!imagesSavingThreadReady) {
            imagesSavingThread = new FSavingThread();
            imagesSavingThreadReady = true;
        }
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString(std::string("createJsonFile Failed for ") + file_path, ex.what(), LogDebugLevel::Failure);
    }
}

std::map<int64, std::vector<ImageToSave>> RecordingFile::getImageToSave()
{
    return imagesToSave_;
}

void RecordingFile::saveImages()
{
    UAirBlueprintLib::LogMessage(TEXT("Saving images: "), TEXT("Started"), LogDebugLevel::Success);
    int nb = 0;
    std::map<int64, std::vector<ImageToSave>>::iterator it;
    for (it = imagesToSave_.begin(); it != imagesToSave_.end(); it++) {
        std::vector<ImageToSave> imagesFrame = it->second;
        for (int i = 0; i < imagesFrame.size(); i++) {
            ImageToSave image = imagesFrame[i];
            if (image.image_type == (int)msr::airlib::ImageCaptureBase::ImageType::DepthPlanar) {
                cv::Mat depth_image = cv::Mat(image.height, image.width, CV_32FC1, (float*)(image.image_data_float.data()));
                cv::Mat depth_image_converted;
                depth_image.convertTo(depth_image_converted, CV_16UC1);

                if (!depth_image_converted.empty())
                    cv::imwrite(image.image_full_file_path, depth_image_converted);
                else
                    UE_LOG(LogTemp, Warning, TEXT("Error saving depth"));
                nb++;
            }
            else if (image.image_type == (int)msr::airlib::ImageCaptureBase::ImageType::Segmentation) {
                cv::Mat mask = image.mask;
                if (!mask.empty())
                    cv::imwrite(image.image_full_file_path, mask);
                else
                    UE_LOG(LogTemp, Warning, TEXT("Error saving mask"));
                nb++;
            }
            else if (image.image_type == (int)msr::airlib::ImageCaptureBase::ImageType::Scene) {
                cv::Mat scene = cv::Mat(image.height, image.width, CV_8UC3, (unsigned char*)image.image_data_uint8.data());
                if (!scene.empty()) {
                    cv::imwrite(image.image_full_file_path, scene);
                }
                else
                    UE_LOG(LogTemp, Warning, TEXT("Error saving image %s"), *image.image_full_file_path.c_str());


                nb++;
            }
           // UE_LOG(LogTemp, Warning, TEXT("Frame saved, %d / %d"), nb, imagesToSave_.size());
        }
    }
    UAirBlueprintLib::LogMessage(TEXT("Saving images: "), TEXT("Finished"), LogDebugLevel::Success);
    UE_LOG(LogTemp, Warning, TEXT("Images saved: finished"));
    clearImageToSave();
}

void RecordingFile::clearImageToSave()
{
    imagesToSave_.clear();
}

bool RecordingFile::isFileOpen() const
{
    return log_file_handle_ != nullptr;
}

void RecordingFile::closeFile()
{
    if (isFileOpen())
        delete log_file_handle_;

    log_file_handle_ = nullptr;
}

void RecordingFile::writeString(const std::string& str) const
{
    try {
        if (log_file_handle_) {
            FString line_f(str.c_str());
            log_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());
        }
        else
            UAirBlueprintLib::LogMessageString("Attempt to write to recording log file when file was not opened", "", LogDebugLevel::Failure);
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString(std::string("file write to recording file failed "), ex.what(), LogDebugLevel::Failure);
    }
}

RecordingFile::~RecordingFile()
{
    stopRecording(true);
}

std::string RecordingFile::getImagePath() {
    return image_path_;
}

void RecordingFile::startRecording(msr::airlib::VehicleSimApiBase* vehicle_sim_api, int64 sequence_id, const std::string& folder)
{
    try {
        std::string vehicle_folder_name = vehicle_sim_api->getVehicleName();
        std::string log_folderpath = common_utils::FileSystem::getLogFolderPath(true, folder) + "/" + vehicle_folder_name;
        image_path_ = log_folderpath;

        std::string log_filepath = common_utils::FileSystem::getLogFileNamePath(log_folderpath, record_filename, "", ".json", false);
        if (log_filepath != "") {

            frameIndex = 0;
            data = FJsonDataSet();
            imagesToSave_.clear();
            FJsonMetaData meta;
            meta.Bbox3dMinimumVolume = BBOX_MINIMUM_VOLUME;

            meta.ImageHeight = vehicle_sim_api->getVehicleSetting()->cameras.begin()->second.capture_settings.at(0).height;
            meta.ImageWidth = vehicle_sim_api->getVehicleSetting()->cameras.begin()->second.capture_settings.at(0).width;
            meta.InvalidValue = INVALID_VALUE;
            meta.IsMicroseconds = IS_MICROSECONDS;
            meta.TargetFPS = (int)(1 / AirSimSettings::singleton().recording_setting.record_interval);
            meta.ZEDSerialNumber = vehicle_sim_api->getVehicleSetting()->cameras.begin()->second.capture_settings.at(0).serial_number;

            meta.IsRealZED = false;
            meta.IsRectified = true;
            meta.SequenceID = sequence_id;

            data.Metadata = meta;

            data.InitialWorldPosition = FJsonMatrix4x4(static_cast<PawnSimApi*>(vehicle_sim_api)->getCamera("Left")->GetActorTransform());

            msr::airlib::AirSimSettings::CaptureSetting captureSettings = msr::airlib::AirSimSettings::singleton().getVehicleSetting(vehicle_sim_api->getVehicleName())->cameras.at("Left").capture_settings.at(0);
            FJsonCamerasConfiguration cameras_config;

            cameras_config.fx = captureSettings.fx;
            cameras_config.fy = captureSettings.fy;
            cameras_config.cx = captureSettings.cx;
            cameras_config.cy = captureSettings.cy;
            cameras_config.baseline = captureSettings.baseline / 1000.0f;

            data.CamerasConfiguration = cameras_config;

            if (msr::airlib::AirSimSettings::singleton().simmode_name == "Multirotor") {
#if SAVE_SENSOR_DATA
                msr::airlib::HomeGeoPoint geopoint = AirSimSettings::singleton().origin_geopoint;

                data.GeoPoint.altitude = geopoint.home_geo_point.altitude;
                data.GeoPoint.latitude = geopoint.home_geo_point.latitude;
                data.GeoPoint.longitude = geopoint.home_geo_point.longitude;

                MultirotorPawnSimApi* vehiclePawn = static_cast<MultirotorPawnSimApi*>(vehicle_sim_api);
                auto vehicleAPI = vehiclePawn->getVehicleApi();
                auto* imu = static_cast<const ImuSimple*>(vehicleAPI->findSensorByName("Imu", msr::airlib::SensorBase::SensorType::Imu));

                ImuSimpleParams imu_params = imu->getIMUParams();
                data.SensorsConfiguration.IMUConfiguration.AngularNoiseDensity = imu_params.gyro.arw;
                data.SensorsConfiguration.IMUConfiguration.GyroRandomWalk = imu->getGyroBiasStabilityNorm();
                data.SensorsConfiguration.IMUConfiguration.AccelNoiseDensity = imu_params.accel.vrw;
                data.SensorsConfiguration.IMUConfiguration.AccelRandomWalk = imu->getAccelBiasStabilityNorm();
#endif
            }

            createJsonFile(log_filepath);
        }
        else {
            UAirBlueprintLib::LogMessageString("Cannot start recording because path for log file is not available", "", LogDebugLevel::Failure);
            return;
        }

        if (isFileOpen()) {
            is_recording_ = true;

            UAirBlueprintLib::LogMessage(TEXT("Recording: "), TEXT("Started"), LogDebugLevel::Success);
        }
        else
            UAirBlueprintLib::LogMessageString("Error creating log file", log_filepath.c_str(), LogDebugLevel::Failure);
    }
    catch (...) {
        UAirBlueprintLib::LogMessageString("Error in startRecording", "", LogDebugLevel::Failure);
    }
}

void RecordingFile::stopRecording(bool ignore_if_stopped)
{
    is_recording_ = false;
    imagesSavingThread->Stop();

    if (!isFileOpen()) {
        if (ignore_if_stopped)
            return;

        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was not open"), LogDebugLevel::Failure);
    }
    else {
        writeString(std::string(TCHAR_TO_UTF8(*SerializeJson(data))));

        closeFile();
        UAirBlueprintLib::LogMessage(TEXT("Recording: "), TEXT("Stopped"), LogDebugLevel::Success);
        
        //saveImages();
    }

    UAirBlueprintLib::LogMessage(TEXT("Data saved to: "), FString(image_path_.c_str()), LogDebugLevel::Success);
}

bool RecordingFile::isRecording() const
{
    return is_recording_;
}
