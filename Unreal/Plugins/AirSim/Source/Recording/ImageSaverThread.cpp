#include "ImageSaverThread.h"

#pragma region Main Thread Code
// This code will be run on the thread that invoked this thread (i.e. game thread)

FImageSaverThread::FImageSaverThread()
{
    // Constructs the actual thread object. It will begin execution immediately
    // If you've passed in any inputs, set them up before calling this.
    Thread = FRunnableThread::Create(this, TEXT("ImageSaverThread"));
}

FImageSaverThread::~FImageSaverThread()
{
    if (Thread) {
        // Kill() is a blocking call, it waits for the thread to finish.
        // Hopefully that doesn't take too long
        Thread->Kill();
        delete Thread;
    }
}

#pragma endregion
// The code below will run on the new thread.

bool FImageSaverThread::Init()
{
    UE_LOG(LogTemp, Warning, TEXT("My image saver thread has been initialized"))

    // Return false if you want to abort the thread
    return true;
}

uint32 FImageSaverThread::Run()
{
    // Peform your processor intensive task here. In this example, a neverending
    // task is created, which will only end when Stop is called.
    while (bRunThread) {

        if (bInputReady) {

            UAirBlueprintLib::LogMessage(TEXT("Saving images: "), TEXT("Started"), LogDebugLevel::Success);
            int nb = 0;

            for (auto& image : images) {
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
            }

            bInputReady = false;
            FPlatformProcess::Sleep(0.01f);
        }
    }

    return 0;
}

// This function is NOT run on the new thread!
void FImageSaverThread::Stop()
{
    // Clean up memory usage here, and make sure the Run() function stops soon
    // The main thread will be stopped until this finishes!

    // For this example, we just need to terminate the while loop
    // It will finish in <= 1 sec, due to the Sleep()
    bRunThread = false;
}
