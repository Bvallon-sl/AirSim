#pragma once

#include "CoreMinimal.h"

#include <string>
#include "AirBlueprintLib.h"
#include "physics/Kinematics.hpp"
#include "HAL/FileManager.h"
#include "PawnSimApi.h"
#include "Vehicles/Multirotor/MultirotorPawnSimApi.h"
#include "common/AirSimSettings.hpp"
#include "Runtime/JsonUtilities/Public/JsonObjectConverter.h"

#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include "UtilityStructs.h"
#include "ImageSaverThread.h"

class RecordingFile
{
public:
    //RecordingFile();
    ~RecordingFile();

    void appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, std::vector<msr::airlib::DetectionInfo_UU>& detections, msr::airlib::VehicleSimApiBase* vehicle_sim_api, int64 timestamp);
    void appendSensorsData(msr::airlib::VehicleSimApiBase* vehicle_sim_api, int64 timestamp);
    void appendColumnHeader(const std::string& header_columns);
    void startRecording(msr::airlib::VehicleSimApiBase* vehicle_sim_api, int64 sequence_id, const std::string& folder = "");
    void stopRecording(bool ignore_if_stopped);
    bool isRecording() const;
	FJsonDataSet getDataSet();

    std::vector<ImageToSave> getImageToSave();
    std::string getImagePath();

    void saveImages();

private:
    void createFile(const std::string& file_path, const std::string& header_columns);
    void createJsonFile(const std::string& file_path);
    void closeFile();
    void writeString(const std::string& line) const;
    bool isFileOpen() const;

    void clearImageToSave();

    void logDetections(APIPCamera* camera, int image_height, cv::Mat& mask, std::vector<msr::airlib::DetectionInfo_UU>& detections, FJsonFrameDetections& frameDetections);
    FJsonBoundingBox2DData ComputeBbox2D(FJsonBoundingBox2DData raw_detection, cv::Mat mask, int id);
    FJsonSkeleton3DData RetrieveSkeletonData(FTransform camPose, msr::airlib::DetectionInfo_UU& detection);

private:
    std::string record_filename = "Data";
    std::string image_path_;
    bool is_recording_ = false;
    IFileHandle* log_file_handle_ = nullptr;
    //FImageSaverThread* imageSaveThread_ = nullptr;

    FJsonDataSet data;
    std::map<int64, std::vector<ImageToSave>> imagesToSave_;
    std::deque<int64> timestamps_;
    int frameIndex;
};