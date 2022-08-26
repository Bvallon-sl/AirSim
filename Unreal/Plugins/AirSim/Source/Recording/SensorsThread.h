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

#include "UtilityStructs.h"


class FSensorsThread : public FRunnable
{
public:
    std::map<std::string, FJsonSensorsData> jsonSensorsData;
    bool isSaving = false;

public:
    FSensorsThread(common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> m_vehicles);
    virtual ~FSensorsThread();

    common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;

    bool Init() override;
    uint32 Run() override;
    void Stop() override;

private:

    void appendSensorsData(msr::airlib::VehicleSimApiBase* vehicle_sim_api);

    FRunnableThread* thread_;
    msr::airlib::TTimePoint last_sensors_screenshot_on_;
    bool bRunThread = true;

};