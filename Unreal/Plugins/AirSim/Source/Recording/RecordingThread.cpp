#include "RecordingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/RunnableThread.h"

#include <thread>
#include <mutex>
#include "RenderRequest.h"
#include "PIPCamera.h"

std::unique_ptr<FRecordingThread> FRecordingThread::running_instance_;
std::unique_ptr<FRecordingThread> FRecordingThread::finishing_instance_;
msr::airlib::WorkerThreadSignal FRecordingThread::finishing_signal_;
bool FRecordingThread::first_ = true;

WorldSimApi* FRecordingThread::world_sim_api_ = nullptr;


// copy in binary mode
bool copyFile(const char* SRC, const char* DEST)
{
    std::ifstream src(SRC, std::ios::binary);
    std::ofstream dest(DEST, std::ios::binary);
    dest << src.rdbuf();
    return src && dest;
}

FRecordingThread::FRecordingThread()
    : stop_task_counter_(0), /* recording_file_(nullptr),*/ is_ready_(false)
{
    counter = 0;
    for (auto& it : recording_files_) {
        it.second = nullptr;
    }
    thread_.reset(FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal)); // Windows default, possible to specify more priority
}

void FRecordingThread::startRecording(WorldSimApi* world_sim_api, const RecordingSetting& settings,
                                      const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis)
{
    stopRecording();
    world_sim_api_ = world_sim_api;
    //TODO: check FPlatformProcess::SupportsMultithreading()?
    assert(!isRecording());

    running_instance_.reset(new FRecordingThread());
    running_instance_->settings_ = settings;
    running_instance_->vehicle_sim_apis_ = vehicle_sim_apis;

    world_sim_api_->setSegmentationObjectID("[\\w\\W]*", 255, true);

    int i = 1;
    // create mask
    while (world_sim_api_->setSegmentationObjectID("person_" + std::to_string(i), i)) {
        i++;
    }

    int64 sequence_id = IS_MICROSECONDS == true ? (msr::airlib::ClockFactory::get()->nowNanos() / 1000) : msr::airlib::ClockFactory::get()->nowNanos();

    for (const auto& vehicle_sim_api : vehicle_sim_apis) {
        auto vehicle_name = vehicle_sim_api->getVehicleName();
        
         MultirotorPawnSimApi* vehiclePawn = static_cast<MultirotorPawnSimApi*>(vehicle_sim_api);

         SimpleFlightApi* simple_flight = static_cast<SimpleFlightApi*>(vehiclePawn->getVehicleApi());

        running_instance_->image_captures_[vehicle_name] = vehicle_sim_api->getImageCapture();
        running_instance_->last_poses_[vehicle_name] = msr::airlib::Pose();

#if SAVE_DETECTION
        CameraDetails camera_details("Left", vehicle_name, false);
        world_sim_api_->setDetectionFilterRadius(msr::airlib::ImageCaptureBase::ImageType::Scene, MAX_DISTANCE_METER * 100, camera_details);
        world_sim_api->addDetectionFilterMeshName(msr::airlib::ImageCaptureBase::ImageType::Scene, "person_*", camera_details);
#endif

        running_instance_->recording_files_.insert(std::pair<std::string, std::unique_ptr<RecordingFile>>(vehicle_name, std::make_unique<RecordingFile>()));
        running_instance_->recording_files_.at(vehicle_name)->startRecording(vehicle_sim_api, sequence_id, settings.folder);
    }

    running_instance_->last_screenshot_on_ = 0;

    // Set is_ready at the end, setting this before can cause a race when the file isn't open yet
    running_instance_->is_ready_ = true;
}

FRecordingThread::~FRecordingThread()
{
    if (this == running_instance_.get()) stopRecording();
}

void FRecordingThread::init()
{
    first_ = true;
}

bool FRecordingThread::isRecording()
{
    return running_instance_ != nullptr;
}

void FRecordingThread::stopRecording()
{
    if (running_instance_) {
        assert(finishing_instance_ == nullptr);
        finishing_instance_ = std::move(running_instance_);
        assert(!isRecording());
        finishing_instance_->Stop();
    }
}

void FRecordingThread::killRecording()
{
    if (first_) return;

    stopRecording();
    bool finished = finishing_signal_.waitForRetry(1, 5);
    if (!finished) {
        UE_LOG(LogTemp, Log, TEXT("killing thread"));
        finishing_instance_->thread_->Kill(false);
    }
}

void FRecordingThread::createMulticamCalibFile(std::vector<FJsonDataSet> data, std::string folder_path)
{
    IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
    // Calib file
    std::string calib_filepath = common_utils::FileSystem::getLogFileNamePath(folder_path, "../multicam_calib", "", ".json", false);
    FJsonMulticamCalib multicam_calib_file;

    if (data.size() > 0) {
    
        for (FJsonDataSet dataset : data) {

            FJsonSingleCalib calib;

            calib.inputPath = "";
            calib.inputType = 2;
            calib.serial = dataset.Metadata.ZEDSerialNumber;
            
            FTransform camera_transform = convertFromUnityToImageCoordinateSystem(dataset.InitialWorldPosition.toTransform());
            calib.tx = camera_transform.GetLocation().X /** 1000*/;
            calib.ty = camera_transform.GetLocation().Y /** 1000*/;
            calib.tz = camera_transform.GetLocation().Z /** 1000*/;

            FVector rot = convertMatrixToRot(camera_transform.ToMatrixWithScale());

            calib.rx = rot.X;
            calib.ry = rot.Y;
            calib.rz = rot.Z;

            multicam_calib_file.singleCalibs.Add(calib);
        }

        IFileHandle* calib_file_handle_ = platform_file.OpenWrite(*FString(calib_filepath.c_str()));
        FString line_f(std::string(TCHAR_TO_UTF8(*SerializeJson(multicam_calib_file))).c_str());
        calib_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());

        delete calib_file_handle_;
        calib_file_handle_ = nullptr;
    
    }
}

//create multicamera json  based on each single cam data
void FRecordingThread::createMulticamJsonFile(std::vector<FJsonDataSet> data, std::string folder_path)
{
    IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();

    // Fused GT json
    std::string log_filepath = common_utils::FileSystem::getLogFileNamePath(folder_path, "../multicam_data", "", ".json", false);
    FJsonDataSet multicam_file;

    if (data.size() > 0) {

        multicam_file.Metadata = data[0].Metadata;

        TMap<int, FJsonSingleDetection> detected_ids;
        for (int i = 0; i < data[0].Frames.Num(); i++) { //assuming all gt have the same number of frame
            detected_ids.Reset();

            FJsonFrameData fusedFrame;
            fusedFrame.EpochTimeStamp = data[0].Frames[i].EpochTimeStamp;
            fusedFrame.FrameIndex = data[0].Frames[i].FrameIndex;
            fusedFrame.ImageFileName = data[0].Frames[i].ImageFileName;

            FJsonFrameDetections fusedFrameDetection;
            for (FJsonDataSet dataset : data) { //foreach json
                for (int j = 0; j < dataset.Frames[i].Detections.ObjectDetections.Num(); j++){

                    FJsonSingleDetection singleDetection = dataset.Frames[i].Detections.ObjectDetections[j];
                    FTransform worlPose = dataset.Frames[i].TrackedPose.WorldPose.toTransform();

                    if (!detected_ids.Contains(singleDetection.ObjectID)) {
                        FJsonSingleDetection fusedFrameSingleDetection;
                        fusedFrameSingleDetection.ObjectID = singleDetection.ObjectID;
                        fusedFrameSingleDetection.ObjectType = singleDetection.ObjectType;
                        
                        fusedFrameSingleDetection.BoundingBox3D_World = singleDetection.BoundingBox3D_World;
                        fusedFrameSingleDetection.BoundingBox3D_World_Raw = singleDetection.BoundingBox3D_World_Raw;
                        fusedFrameSingleDetection.Position3D_World_Floor = singleDetection.Position3D_World_Floor;
                        fusedFrameSingleDetection.Velocity3D_MPS = singleDetection.Velocity3D_MPS;

                        fusedFrameSingleDetection.GlobalRootOrientation = camToWorld(worlPose, singleDetection.GlobalRootOrientation);                            

                        for (int idx = 0; idx < (int)BODY_PARTS_POSE_34::LAST; idx++) {
                            if (!isInvalidValue(singleDetection.Keypoints3D_34[idx])) {
                                fusedFrameSingleDetection.Keypoints3D_34.Add(camToWorld(worlPose, singleDetection.Keypoints3D_34[idx]));
                                fusedFrameSingleDetection.LocalPositionPerJoint.Add(camToWorld(worlPose, singleDetection.LocalPositionPerJoint[idx]));
                                fusedFrameSingleDetection.LocalOrientationPerJoint.Add(camToWorld(worlPose, singleDetection.LocalOrientationPerJoint[idx])); 

                            }
                            else {
                                fusedFrameSingleDetection.Keypoints3D_34.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
                                fusedFrameSingleDetection.LocalPositionPerJoint.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
                                fusedFrameSingleDetection.LocalOrientationPerJoint.Add(FQuat(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
                            }
                        }

                        for (int idx = 0; idx < (int)BODY_PARTS_POSE_18::LAST; idx++) {
                            if (!isInvalidValue(singleDetection.Keypoints3D[idx])) {
                                fusedFrameSingleDetection.Keypoints3D.Add(camToWorld(worlPose, singleDetection.Keypoints3D[idx]));
                            }
                            else {
                                fusedFrameSingleDetection.Keypoints3D.Add(FVector(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE));
                            }
                        }

                        detected_ids.Add(singleDetection.ObjectID, fusedFrameSingleDetection);
                        fusedFrameDetection.ObjectDetections.Add(fusedFrameSingleDetection);
                    }
                    else {
                        for (int idx = 0; idx < (int)BODY_PARTS_POSE_34::LAST; idx++) {
                            if (isInvalidValue(fusedFrameDetection.ObjectDetections[j].Keypoints3D_34[idx])) {
                                if (!isInvalidValue(singleDetection.Keypoints3D_34[idx])) {
                                    fusedFrameDetection.ObjectDetections[j].Keypoints3D_34[idx] = (camToWorld(worlPose, singleDetection.Keypoints3D_34[idx]));
                                    fusedFrameDetection.ObjectDetections[j].LocalPositionPerJoint[idx] = (camToWorld(worlPose, singleDetection.LocalPositionPerJoint[idx]));
                                    fusedFrameDetection.ObjectDetections[j].LocalOrientationPerJoint[idx] = (camToWorld(worlPose, singleDetection.LocalOrientationPerJoint[idx]));  
                                }
                            }
                        }

                        for (int idx = 0; idx < (int)BODY_PARTS_POSE_18::LAST; idx++) {
                            if (isInvalidValue(fusedFrameDetection.ObjectDetections[j].Keypoints3D[idx])) {
                                if (!isInvalidValue(singleDetection.Keypoints3D[idx])) {
                                    fusedFrameDetection.ObjectDetections[j].Keypoints3D[idx] = (camToWorld(worlPose, singleDetection.Keypoints3D[idx]));
                                }
                            }
                        }

                        fusedFrameDetection.ObjectDetections[j] = detected_ids[singleDetection.ObjectID];
                    }
                }
            }

            fusedFrame.Detections = fusedFrameDetection;
            multicam_file.Frames.Add(fusedFrame);
        }

        IFileHandle* log_file_handle_ = platform_file.OpenWrite(*FString(log_filepath.c_str()));
        FString line_f(std::string(TCHAR_TO_UTF8(*SerializeJson(multicam_file))).c_str());
        log_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());

        delete log_file_handle_;
        log_file_handle_ = nullptr;
    }
}

/*********************** methods for instance **************************************/

bool FRecordingThread::Init()
{
    if (first_) {
        first_ = false;
    }
    else {
        finishing_signal_.wait();
    }
    /* if (recording_file_) {
        UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread"), TEXT(""), LogDebugLevel::Success);
    }*/
    for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
        const auto& vehicle_name = vehicle_sim_api->getVehicleName();
        if (recording_files_.at(vehicle_name)) UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread"), TEXT(""), LogDebugLevel::Success);
    }

    chrono_save = 0;

    return true;
}

uint32 FRecordingThread::Run()
{
    while (stop_task_counter_.GetValue() == 0) {
        //make sure all vars are set up
        if (is_ready_) {
#if SAVE_SENSOR_DATA
            if (!sensorsThreadReady) {
                sensorsThread = new FSensorsThread(vehicle_sim_apis_);
                sensorsThreadReady = true;
            }
#endif     

            bool interval_elapsed = msr::airlib::ClockFactory::get()->elapsedSince(last_screenshot_on_) > settings_.record_interval;
            if (interval_elapsed) {

                last_screenshot_on_ = msr::airlib::ClockFactory::get()->nowNanos();

                if (vehicle_sim_apis_.mapSize() > 2)  world_sim_api_->pause(true);
                for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
                    const auto& vehicle_name = vehicle_sim_api->getVehicleName();

                    if (!settings_.record_on_move) {

                        std::vector<ImageCaptureBase::ImageResponse> responses;
                        FString name(vehicle_name.c_str());
                        image_captures_[vehicle_name]->getImages(settings_.requests[vehicle_name], responses);
#if SAVE_DETECTION
                        CameraDetails camera_details("Left", vehicle_name, false);
                        detections_[vehicle_name] = world_sim_api_->getDetections_UU(msr::airlib::ImageCaptureBase::ImageType::Scene, camera_details);
#endif
                        if (counter > nb_frames_before_log) {
                            recording_files_.at(vehicle_name)->appendRecord(responses, detections_[vehicle_name], vehicle_sim_api, last_screenshot_on_);
                        }
                    }
                }
                if (vehicle_sim_apis_.mapSize() > 2) world_sim_api_->pause(false);

                UAirBlueprintLib::LogMessageString("time : ",
                                                   Utils::stringf("%f", msr::airlib::ClockFactory::get()->elapsedSince(last_screenshot_on_), ClockFactory::get()->getTrueScaleWrtWallClock()),
                                                   LogDebugLevel::Informational);
                counter++;
            }
        }
    }
#if SAVE_SENSOR_DATA
    sensorsThread->Stop();

    for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
        const auto& vehicle_name = vehicle_sim_api->getVehicleName();
        recording_files_.at(vehicle_name)->data.Sensors = sensorsThread->jsonSensorsData[vehicle_name];
    }
#endif
  
	std::vector<FJsonDataSet> datasets;
    std::string folder_path;
	for (const auto& recording_file : recording_files_) {
            datasets.push_back(recording_file.second->getDataSet());

            folder_path = recording_file.second->getImagePath();
	}
    
    if (datasets.size() > 1) {
        createMulticamJsonFile(datasets, folder_path);
        createMulticamCalibFile(datasets, folder_path);
    }

    // copie settings file in folder
    std::string settings_path = "C:/Users/Benjamin/Documents/AirSim/settings.json";
    std::string settings_dest = folder_path + "/../settings.json";

     bool b = copyFile(settings_path.c_str(), settings_dest.c_str());

    if (!b) {
        FString filepath(settings_dest.c_str());
        UE_LOG(LogTemp, Log, TEXT("Fail copying settings.json at %s"), *filepath);
    }
 
    //recording_file_.reset();
    for (const auto& vehicle_sim_api : vehicle_sim_apis_)
    {
        const auto& vehicle_name = vehicle_sim_api->getVehicleName();

        recording_files_.at(vehicle_name).reset();
    }

    return 0;
}

void FRecordingThread::Stop()
{
    stop_task_counter_.Increment();
}

void FRecordingThread::Exit()
{
    assert(this == finishing_instance_.get());
    /* if (recording_file_)
        recording_file_.reset();*/

    for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
        const auto& vehicle_name = vehicle_sim_api->getVehicleName();
        if (recording_files_.at(vehicle_name)) recording_files_.at(vehicle_name).reset();
    }

    finishing_signal_.signal();
}
