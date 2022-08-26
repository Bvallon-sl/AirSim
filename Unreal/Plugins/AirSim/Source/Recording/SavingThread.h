#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "physics/Kinematics.hpp"
#include <memory>
#include "common/ClockFactory.hpp"
#include "common/AirSimSettings.hpp"
#include "common/WorkerThread.hpp"

#include <string>
#include <fstream>

#include "UtilityStructs.h"


class FSavingThread : public FRunnable
{
public:
    TQueue<std::vector<ImageToSave>> images_queue;

public:
    FSavingThread();
    virtual ~FSavingThread();

    bool Init() override;
    uint32 Run() override;
    void Stop() override;

private:

    FRunnableThread* thread_;
    bool bRunThread = true;

    void saveImages(std::vector<ImageToSave> imagesFrame);
};