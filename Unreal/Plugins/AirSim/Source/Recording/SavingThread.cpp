#include "SavingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/RunnableThread.h"

#include <thread>
#include <mutex>
#include "RenderRequest.h"
#include "PIPCamera.h"


FSavingThread::FSavingThread()
{ 
    thread_ = FRunnableThread::Create(this, TEXT("Images Saving thread"));
}

FSavingThread::~FSavingThread()
{
    if (thread_) {
        thread_->Kill();
        delete thread_;
    }
}

bool FSavingThread::Init()
{
    UE_LOG(LogTemp, Warning, TEXT("Images Saving has been initialized"))

    return true;
}

uint32 FSavingThread::Run()
{
    while (bRunThread) {
        while (!images_queue.IsEmpty()) {
            std::vector<ImageToSave> imgs;
            if (images_queue.Dequeue(imgs)) {
                saveImages(imgs);
            }
        }
    }
    return 0;
}

// This function is NOT run on the new thread!
void FSavingThread::Stop()
{
    bRunThread = false;
}


void FSavingThread::saveImages(std::vector<ImageToSave> imagesFrame)
{
    int nb = 0;

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
    }

}