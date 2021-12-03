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

    bool save_success = false;
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
        image_file_name << camera_name << "_" << ts << "_" << frameIndex;
        std::string extension = ".png";
        image_file_name << extension;

        std::string image_full_file_path = common_utils::FileSystem::combine(image_path_, image_file_name.str());

#if 0
        //write image file
        try {
            if (response.image_type == msr::airlib::ImageCaptureBase::ImageType::DepthPlanar) {
                cv::Mat depth_image = cv::Mat(response.height, response.width, CV_32FC1, (float*)(response.image_data_float.data()));
                cv::Mat depth_image_converted;
                depth_image.convertTo(depth_image_converted, CV_16UC1);
                cv::imwrite(image_full_file_path, depth_image_converted);

            }
            else if (response.image_type == msr::airlib::ImageCaptureBase::ImageType::Scene) {
                cv::Mat image = cv::Mat(response.height, response.width, CV_8UC3, (unsigned char*)response.image_data_uint8.data());
                cv::imwrite(image_full_file_path, image);


                /* std::ofstream file(image_full_file_path, std::ios::binary);
                file.write(reinterpret_cast<const char*>(response.image_data_uint8.data()), response.image_data_uint8.size());
                file.close();*/
            }

            save_success = true;
        }
        catch (std::exception& ex) {
            save_success = false;
            UAirBlueprintLib::LogMessage(TEXT("Image file save failed"), FString(ex.what()), LogDebugLevel::Failure);
        }

#else
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

#endif
    }

    imagesToSave_.insert(std::pair<int64, std::vector<ImageToSave>>(ts, imagesFrame));
    timestamps_.push_back(ts);

    //write to JSON file

    PawnSimApi* pawn = static_cast<PawnSimApi*>(vehicle_sim_api);
    FJsonFramePoseData trackedPoseData;
    trackedPoseData.WorldPose = FJsonMatrix4x4(convertFromUUToUnityCoordinateSystem(pawn->getCamera("Left")->GetActorTransform()));
    FJsonFrameData fdata;
    fdata.EpochTimeStamp = ts;

    std::ostringstream fileName;
    fileName // << "SBS_"
        << "_"
        << fdata.EpochTimeStamp << "_"
        << frameIndex
        << ".png";
    fdata.ImageFileName = UTF8_TO_TCHAR(fileName.str().c_str());
    fdata.FrameIndex = frameIndex;
    fdata.TrackedPose = trackedPoseData;

    FJsonFrameDetections frameDetections;

    if (mask.empty()) UE_LOG(LogTemp, Warning, TEXT("mask empty"));
    logDetections(pawn->getCamera("Left"), mask, detections, frameDetections); // Compute Detections and store them in the Struct
    fdata.Detections = frameDetections;

    data.Frames.Add(fdata);
    //UAirBlueprintLib::LogMessage(TEXT("Screenshot saved to:"), filePath, LogDebugLevel::Success);

    if (msr::airlib::AirSimSettings::singleton().simmode_name == "Multirotor") {
        auto vehicleAPI = static_cast<MultirotorPawnSimApi*>(pawn)->getVehicleApi();
        //msr::airlib::ImuBase::Output imuData = vehicleAPI->getImuData("Imu");
        // msr::airlib::BarometerBase::Output barometerData = vehicleAPI->getBarometerData("Barometer");
        //UE_LOG(LogTemp, Warning, TEXT("baro, %f"), barometerData.pressure);
    }
    frameIndex++;
}

FJsonDataSet RecordingFile::getDataSet(){
    return data;
}

FJsonBoundingBox2DData RecordingFile::ComputeBbox2D(FJsonBoundingBox2DData raw_detection, cv::Mat mask, int id)
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

FJsonSkeleton3DData RecordingFile::RetrieveSkeletonData(FTransform camPose, msr::airlib::DetectionInfo_UU& detection)
{

    FJsonSkeleton3DData skeleton_raw;

    for (int idx = 0; idx < (int)BODY_PARTS_POSE_34::LAST; idx++) {

        if (targetBone[idx] == "not_found") continue;

        FTransform local_joint_transform = detection.skeletal_mesh->GetBoneTransform(idx);

        FVector keypoint = detection.skeletal_mesh->GetBoneLocation(targetBone[idx], EBoneSpaces::WorldSpace);
        FVector local_position = detection.skeletal_mesh->GetBoneLocation(targetBone[idx], EBoneSpaces::ComponentSpace);
        FQuat local_orientation = detection.skeletal_mesh->GetBoneQuaternion(targetBone[idx], EBoneSpaces::ComponentSpace);

        skeleton_raw.local_position_per_joint.Add(convertFromUUToUnityCoordinateSystem(worldToCam(camPose, local_position)));
        skeleton_raw.local_orientation_per_joint.Add(convertFromUUToUnityCoordinateSystem(FTransform(worldToCam(camPose, local_orientation))).GetRotation()); /// ?????????
        //skeleton_raw.local_position_per_joint.Add(local_position);
        //skeleton_raw.local_orientation_per_joint.Add(local_orientation);
        skeleton_raw.keypoints.Add(convertFromUUToUnityCoordinateSystem(worldToCam(camPose, keypoint)));
    }

    skeleton_raw.global_root_orientation = convertFromUUToUnityCoordinateSystem(FTransform(detection.orientation)).GetRotation();
    return skeleton_raw;
}

void RecordingFile::logDetections(APIPCamera* camera, cv::Mat& mask, std::vector<msr::airlib::DetectionInfo_UU>& detections, FJsonFrameDetections& frameDetections)
{
    FTransform cam_pose = camera->GetActorTransform();
    auto height = 1080; // TO DO : retrieve the camera size;
    int offset = 100;
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

        FVector velocity = detection.relative_transform.GetTranslation(); //TO DO
        singleDetection.Velocity3D_MPS = convertFromUUToUnityCoordinateSystem(velocity);

        FJsonBoundingBox2DData bbox2D_raw;
        bbox2D_raw.A = FVector2D(detection.box2D.min.X, detection.box2D.min.Y);
        bbox2D_raw.B = FVector2D(detection.box2D.max.X, detection.box2D.min.Y);
        bbox2D_raw.C = FVector2D(detection.box2D.max.X, FMath::Clamp<int>(detection.box2D.max.Y + offset, detection.box2D.max.Y, height));
        bbox2D_raw.D = FVector2D(detection.box2D.min.X, FMath::Clamp<int>(detection.box2D.max.Y + offset, detection.box2D.max.Y, height));

        cv::Mat mask_id;
        cv::Mat(mask, cv::Rect(bbox2D_raw.A.X, bbox2D_raw.A.Y, (int)std::abs(bbox2D_raw.B.X - bbox2D_raw.A.X), (int)std::abs(bbox2D_raw.D.Y - bbox2D_raw.A.Y))).copyTo(mask_id);

        FJsonBoundingBox2DData bbox2D = ComputeBbox2D(bbox2D_raw, mask_id, ObjectID);
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

        FJsonBoundingBox3DData bbox3D = compute3DBoxesFrom2D(camera, bbox2D, detection); //bbox3D;

        singleDetection.BoundingBox3D_World = bbox3D;

        FJsonSkeleton3DData skeleton3D_raw = RetrieveSkeletonData(cam_pose, detection);

        singleDetection.Skeleton3D_Camera_Raw = skeleton3D_raw;

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
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString(std::string("createJsonFile Failed for ") + file_path, ex.what(), LogDebugLevel::Failure);
    }
}

std::vector<ImageToSave> RecordingFile::getImageToSave()
{
    std::vector<ImageToSave> out;
    if (timestamps_.size() > 0) {
        int64 ts = timestamps_[0];
        timestamps_.pop_front();

        out = imagesToSave_.at(ts);
    }

    return out;
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
            meta.TargetFPS = 15;
            meta.ZEDSerialNumber = 12345;
            meta.IsRealZED = false;
            meta.IsRectified = true;
            meta.SequenceID = sequence_id;

            data.Metadata = meta;

            data.InitialWorldPosition = FJsonMatrix4x4(static_cast<PawnSimApi*>(vehicle_sim_api)->getCamera("Left")->GetActorTransform());
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

    if (!isFileOpen()) {
        if (ignore_if_stopped)
            return;

        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was not open"), LogDebugLevel::Failure);
    }
    else {
        writeString(std::string(TCHAR_TO_UTF8(*SerializeJson(data))));

        closeFile();
        UAirBlueprintLib::LogMessage(TEXT("Recording: "), TEXT("Stopped"), LogDebugLevel::Success);
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
                    cv::imwrite(image.image_full_file_path, depth_image_converted);
                    nb++;
                }
                else if (image.image_type == (int)msr::airlib::ImageCaptureBase::ImageType::Segmentation) {
                    cv::Mat mask = image.mask;
                    cv::imwrite(image.image_full_file_path, mask);
                    nb++;
                }
                else if (image.image_type == (int)msr::airlib::ImageCaptureBase::ImageType::Scene) {
                    cv::Mat scene = cv::Mat(image.height, image.width, CV_8UC3, (unsigned char*)image.image_data_uint8.data());
                    cv::imwrite(image.image_full_file_path, scene);

                    nb++;
                }
                UE_LOG(LogTemp, Warning, TEXT("Frame saved, %d / %d"), nb, imagesToSave_.size() * 4);
            }
        }
        UAirBlueprintLib::LogMessage(TEXT("Saving images: "), TEXT("Finished"), LogDebugLevel::Success);
    }

    UAirBlueprintLib::LogMessage(TEXT("Data saved to: "), FString(image_path_.c_str()), LogDebugLevel::Success);
}

bool RecordingFile::isRecording() const
{
    return is_recording_;
}
