#include "ZEDConfigParser.h"

#include <iostream>
#include <fstream>


std::string ZEDConfigParser::folder_path = "C:/ProgramData/Stereolabs/settings/";

FString ZEDConfigParser::getResName(int image_width, bool left)
{

    FString out = "";
    switch (image_width) {

    case 2208:
        out = "2K";
        break;
    case 1920:
        out = "FHD";
        break;
    case 1280:
        out = "HD";
        break;
    case 672:
        out = "VGA";
        break;
    default:
        out = "CUSTOM";
        break;
    }


    return out;
}

FString ZEDConfigParser::getSectionName(int image_width, bool left) {

    FString out = left ? "[LEFT_CAM_" : "[RIGHT_CAM_";

    return out + getResName(image_width, left);
}

ZEDConfigParser::CalibrationProfile ZEDConfigParser::getConfigFromZEDProfile(std::string config_file, int image_width)
{
    std::string full_path = folder_path + config_file + ".conf";
    FString f_full_path(full_path.c_str());

    CalibrationProfile calib_profile = CalibrationProfile();

    std::ifstream infile(full_path);
    char line[256];

    FString section = getSectionName(image_width);
    UE_LOG(LogTemp, Warning, TEXT("Section name %s"), *section);

    while (infile) {
        infile.getline(line, 256);
        FString s_line(line);

        if (s_line.Contains(section)) {
            UE_LOG(LogTemp, Warning, TEXT("%s"), *s_line);

            // fx
            infile.getline(line, 256);
            s_line = FString(line);

            FString left = "";
            FString right = "";
            s_line.Split("=", &left, &right);
            
            calib_profile.fx = FCString::Atof(*right);
            UE_LOG(LogTemp, Warning, TEXT("fx:  %f"), calib_profile.fx);

            // fy
            infile.getline(line, 256);
            s_line = FString(line);

            left = "";
            right = "";
            s_line.Split("=", &left, &right);

            calib_profile.fy = FCString::Atof(*right);

            // cx
            infile.getline(line, 256);
            s_line = FString(line);

            left = "";
            right = "";
            s_line.Split("=", &left, &right);

            calib_profile.cx = FCString::Atof(*right);
            UE_LOG(LogTemp, Warning, TEXT("cx: %f"), calib_profile.cx);

             // cy
            infile.getline(line, 256);
            s_line = FString(line);

            left = "";
            right = "";
            s_line.Split("=", &left, &right);

            calib_profile.cy = FCString::Atof(*right);
            UE_LOG(LogTemp, Warning, TEXT("cy: %f"), calib_profile.cy);
        }

        if (s_line.Contains("[STEREO]")) {

            while (infile) {
                infile.getline(line, 256);
                s_line = FString(line);

                if (s_line.Contains("baseline")) {
                    FString left = "";
                    FString right = "";
                    s_line.Split("=", &left, &right);

                    calib_profile.baseline = FCString::Atof(*right);
                    UE_LOG(LogTemp, Warning, TEXT("baseline: %f"), calib_profile.baseline);
                }

                if (s_line.Contains("Baseline")) {
                    FString left = "";
                    FString right = "";
                    s_line.Split("=", &left, &right);

                    calib_profile.baseline = FCString::Atof(*right);
                    UE_LOG(LogTemp, Warning, TEXT("baseline: %f"), calib_profile.baseline);
                }
                else if (s_line.Contains("CV_" + getResName(image_width))) {
                    FString left = "";
                    FString right = "";
                    s_line.Split("=", &left, &right);

                    calib_profile.cv = FCString::Atof(*right);
                    UE_LOG(LogTemp, Warning, TEXT("cv: %f"), calib_profile.cv);
                }
                else if (s_line.Contains("RX_" + getResName(image_width))) {
                    FString left = "";
                    FString right = "";
                    s_line.Split("=", &left, &right);

                    calib_profile.rx = FCString::Atof(*right);
                    UE_LOG(LogTemp, Warning, TEXT("rx: %f"), calib_profile.rx);
                }
                else if (s_line.Contains("RZ_" + getResName(image_width))) {
                    FString left = "";
                    FString right = "";
                    s_line.Split("=", &left, &right);

                    calib_profile.rz = FCString::Atof(*right);
                    UE_LOG(LogTemp, Warning, TEXT("rz: %f"), calib_profile.rz);
                }
            }
        }
    }
    infile.close();

    return calib_profile;
}