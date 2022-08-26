#pragma once

class ZEDConfigParser
{
public:

	struct CalibrationProfile
    {
        float fx = 700;
        float fy = 700;
        float cx = 640;
        float cy = 360;
        float ty = 0;
        float tz = 0;
        float cv = 0;
        float rx = 0;
        float rz = 0;
        float k1 = 0;
        float k2 = 0;
        float k3 = 0;
        float p1 = 0;
        float p2 = 0;
        float baseline = 0.12f;
    };

    static FString getResName(int image_width, bool left = true);
    static FString getSectionName(int image_width, bool left = true);

	static CalibrationProfile getConfigFromZEDProfile(std::string filepath, int image_width);

private:

    static std::string folder_path;
    ZEDConfigParser() {}

};