#include "UnrealImageCapture.h"
#include "Engine/World.h"
#include "ImageUtils.h"

#include "RenderRequest.h"
#include "common/ClockFactory.hpp"

UnrealImageCapture::UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras)
    : cameras_(cameras)
{
    //TODO: explore screenshot option
    //addScreenCaptureHandler(camera->GetWorld());
}

UnrealImageCapture::~UnrealImageCapture()
{
}

void UnrealImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                   std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
{
    if (cameras_->valsSize() == 0) {
        for (unsigned int i = 0; i < requests.size(); ++i) {
            responses.push_back(ImageResponse());
            responses[responses.size() - 1].message = "camera is not set";
        }
    }
    else
        getSceneCaptureImage(requests, responses, false);
    
    //for (int i = 0; i < requests.size() && i < responses.size(); ++i)
    //    getImage(requests[i], responses[i]);
}

/*void UnrealImageCapture::getImage(const msr::airlib::ImageCaptureBase::ImageRequest& request, msr::airlib::ImageCaptureBase::ImageResponse& response) const
{
    getSceneCaptureImage(request.camera_name, request.image_type, response);
}

void UnrealImageCapture::getSceneCaptureImage(const std::string& camera_name, msr::airlib::ImageCaptureBase::ImageType image_type, msr::airlib::ImageCaptureBase::ImageResponse& response) const
{
    FString fcam_name(camera_name.c_str());
    APIPCamera* camera = cameras_->at(camera_name);
    camera->setCameraTypeEnabled(image_type, true);

    USceneCaptureComponent2D* capture = camera->getCaptureComponent(image_type, false);
    UTextureRenderTarget2D* textureTarget = capture->TextureTarget;

    RenderRequest render_request(BufferPool_, BufferPool_float_);
    render_request.fast_param_ = RenderRequest::RenderParams{ capture, textureTarget, false, false };
    render_request.FastScreenshot();

    response.camera_name = camera_name;
    response.time_stamp = render_request.latest_result_.time_stamp;
    response.width = render_request.latest_result_.width;
    response.height = render_request.latest_result_.height;
    response.image_type = image_type;
    response.pixels_as_float = render_request.latest_result_.pixels_as_float;

    if (!response.pixels_as_float)
        response.image_data_uint8 = std::move(render_request.latest_result_.pixels);
    else
        response.image_data_float = std::move(render_request.latest_result_.pixels_float);

    UE_LOG(LogTemp, Warning, TEXT("%s: stats: H: %d  W: %d  type: %d  px_format: %d"),
        *fcam_name, response.height, response.width, image_type, textureTarget->GetFormat());

    // Disable camera after capturing image, this reduces resource consumption when images are not being taken
    // Particulary when a high-resolution camera is used occasionally
    // camera->setCameraTypeEnabled(image_type, false);
}*/

void UnrealImageCapture::getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                              std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, bool use_safe_method) const
{
    std::vector<std::shared_ptr<RenderRequest::RenderParams>> render_params;
    std::vector<std::shared_ptr<RenderRequest::RenderResult>> render_results;

    bool visibilityChanged = false;
    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = cameras_->at(requests.at(i).camera_name);
        //TODO: may be we should have these methods non-const?
        visibilityChanged = const_cast<UnrealImageCapture*>(this)->updateCameraVisibility(camera, requests[i]) || visibilityChanged;
    }

    if (use_safe_method && visibilityChanged) {
        // We don't do game/render thread synchronization for safe method.
        // We just blindly sleep for 200ms (the old way)
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }

    UGameViewportClient* gameViewport = nullptr;
    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = cameras_->at(requests.at(i).camera_name);
        if (gameViewport == nullptr) {
            gameViewport = camera->GetWorld()->GetGameViewport();
        }

        responses.push_back(ImageResponse());
        ImageResponse& response = responses.at(i);

        UTextureRenderTarget2D* textureTarget = nullptr;
        USceneCaptureComponent2D* capture = camera->getCaptureComponent(requests[i].image_type, false);
        if (capture == nullptr) {
            response.message = "Can't take screenshot because none camera type is not active";
        }
        else if (capture->TextureTarget == nullptr) {
            response.message = "Can't take screenshot because texture target is null";
        }
        else
            textureTarget = capture->TextureTarget;

        render_params.push_back(std::make_shared<RenderRequest::RenderParams>(capture, textureTarget, requests[i].pixels_as_float, requests[i].compress));
    }

    if (nullptr == gameViewport) {
        return;
    }

    auto query_camera_pose_cb = [this, &requests, &responses]() {
        size_t count = requests.size();
        for (size_t i = 0; i < count; i++) {
            const ImageRequest& request = requests.at(i);
            APIPCamera* camera = cameras_->at(request.camera_name);
            ImageResponse& response = responses.at(i);
            auto camera_pose = camera->getUUPose();
            response.camera_position = camera_pose.GetLocation();
            response.camera_orientation = camera_pose.GetRotation();
        }
    };
    RenderRequest render_request{ gameViewport, std::move(query_camera_pose_cb) };

    render_request.getScreenshot(render_params.data(), render_results, render_params.size(), use_safe_method);

    for (unsigned int i = 0; i < requests.size(); ++i) {
        const ImageRequest& request = requests.at(i);
        ImageResponse& response = responses.at(i);

        response.camera_name = request.camera_name;
        response.time_stamp = render_results[i]->time_stamp;
        response.image_data_uint8 = std::vector<uint8_t>(render_results[i]->image_data_uint8.GetData(), render_results[i]->image_data_uint8.GetData() + render_results[i]->image_data_uint8.Num());
        response.image_data_float = std::vector<float>(render_results[i]->image_data_float.GetData(), render_results[i]->image_data_float.GetData() + render_results[i]->image_data_float.Num());

        if (use_safe_method) {
            // Currently, we don't have a way to synthronize image capturing and camera pose when safe method is used,
            APIPCamera* camera = cameras_->at(request.camera_name);
            auto camera_pose = camera->getUUPose();
            response.camera_position = camera_pose.GetLocation();
            response.camera_orientation = camera_pose.GetRotation();
        }
        response.pixels_as_float = request.pixels_as_float;
        response.compress = request.compress;
        response.width = render_results[i]->width;
        response.height = render_results[i]->height;
        response.image_type = request.image_type;
    }
}

bool UnrealImageCapture::updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request)
{
    bool visibilityChanged = false;
    if (!camera->getCameraTypeEnabled(request.image_type)) {
        camera->setCameraTypeEnabled(request.image_type, true);
        visibilityChanged = true;
    }

    return visibilityChanged;
}

bool UnrealImageCapture::getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng)
{
    FScreenshotRequest::RequestScreenshot(false); // This is an async operation
    return true;
}

void UnrealImageCapture::addScreenCaptureHandler(UWorld* world)
{
    static bool is_installed = false;

    if (!is_installed) {
        UGameViewportClient* ViewportClient = world->GetGameViewport();
        ViewportClient->OnScreenshotCaptured().Clear();
        ViewportClient->OnScreenshotCaptured().AddLambda(
            [this](int32 SizeX, int32 SizeY, const TArray<FColor>& Bitmap) {
                // Make sure that all alpha values are opaque.
                TArray<FColor>& RefBitmap = const_cast<TArray<FColor>&>(Bitmap);
                for (auto& Color : RefBitmap)
                    Color.A = 255;

                TArray<uint8_t> last_compressed_png;
                FImageUtils::CompressImageArray(SizeX, SizeY, RefBitmap, last_compressed_png);
                last_compressed_png_ = std::vector<uint8_t>(last_compressed_png.GetData(), last_compressed_png.GetData() + last_compressed_png.Num());
            });

        is_installed = true;
    }
}
