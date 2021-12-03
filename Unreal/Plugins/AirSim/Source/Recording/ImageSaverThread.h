#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

#include "UtilityStructs.h"
#include "Recording/RecordingFile.h"

class FImageSaverThread : public FRunnable
{
public:
    // Constructor, create the thread by calling this
    FImageSaverThread();

    // Destructor
    virtual ~FImageSaverThread() override;

    // Overriden from FRunnable
    bool Init() override; // setup here, allocate memory, ect.
    uint32 Run() override; // Main data processing happens here
    void Stop() override; // Clean up any memory you allocated here

    bool bInputReady = false;

    std::vector<ImageToSave> images;
private:
    // Thread handle. Control the thread using this, with operators like Kill and Suspend
    FRunnableThread* Thread;

    // Used to know when the thread should exit, changed in Stop(), read in Run()
    bool bRunThread;
};
