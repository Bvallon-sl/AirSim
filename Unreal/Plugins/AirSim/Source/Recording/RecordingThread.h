#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "Recording/RecordingFile.h"
#include "physics/Kinematics.hpp"
#include <memory>
#include "common/ClockFactory.hpp"
#include "common/AirSimSettings.hpp"
#include "common/WorkerThread.hpp"
#include "SimMode/SimModeBase.h"
#include "WorldSimApi.h"

#include <string>
#include <fstream>


class FRecordingThread : public FRunnable
{
public:
    typedef msr::airlib::AirSimSettings::RecordingSetting RecordingSetting;
    typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

public:
    FRecordingThread();
    virtual ~FRecordingThread();

    static void init();
    static void startRecording(WorldSimApi* simmode, const RecordingSetting& settings,
                               const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis);
    static void stopRecording();
    static void killRecording();
    static bool isRecording();

    static void toggleImagesSaving();

protected:
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

private:

    static void createMulticamJsonFile(std::vector<FJsonDataSet> data, std::string folder_path);
    static void createMulticamCalibFile(std::vector<FJsonDataSet> data, std::string folder_path);

    FThreadSafeCounter stop_task_counter_;

    static std::unique_ptr<FRecordingThread> running_instance_;
    static std::unique_ptr<FRecordingThread> finishing_instance_;
    static msr::airlib::WorkerThreadSignal finishing_signal_;
    static bool first_;

    static std::unique_ptr<FRecordingThread> instance_;

    std::unique_ptr<FRunnableThread> thread_;

    RecordingSetting settings_;
    //std::unique_ptr<RecordingFile> recording_file_;
    std::map <std::string, std::unique_ptr<RecordingFile>> recording_files_;
    common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;

    std::unordered_map<std::string, const ImageCaptureBase*> image_captures_;
    std::unordered_map<std::string, std::vector<msr::airlib::DetectionInfo_UU>> detections_;
    std::unordered_map<std::string, msr::airlib::Pose> last_poses_;

    msr::airlib::TTimePoint last_screenshot_on_;
    msr::airlib::TTimePoint last_screenshot_on_imu_;
    bool is_ready_;
    static WorldSimApi* world_sim_api_;

    static bool saving_;

    int counter = 0;
    int nb_frames_before_log = 2;
};