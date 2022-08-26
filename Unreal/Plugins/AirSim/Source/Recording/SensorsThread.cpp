#include "SensorsThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/RunnableThread.h"

#include <thread>
#include <mutex>
#include "RenderRequest.h"
#include "PIPCamera.h"


FSensorsThread::FSensorsThread(common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> m_vehicles)
{ 
    vehicle_sim_apis_ = m_vehicles;
    thread_ = FRunnableThread::Create(this, TEXT("Sensors Data thread"));
}

FSensorsThread::~FSensorsThread()
{
    if (thread_) {
        thread_->Kill();
        delete thread_;
    }
}

bool FSensorsThread::Init()
{
    UE_LOG(LogTemp, Warning, TEXT("Sensors Data thread has been initialized"))

    last_sensors_screenshot_on_ = 0;

    return true;
}

uint32 FSensorsThread::Run()
{
    while (bRunThread) {
        if (!isSaving) {
            bool interval_elapsed_sensors = msr::airlib::ClockFactory::get()->elapsedSince(last_sensors_screenshot_on_) > (1.0f / SENSORS_FREQUENCY);
            if (interval_elapsed_sensors) {
                last_sensors_screenshot_on_ = msr::airlib::ClockFactory::get()->nowNanos();
                for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
                    appendSensorsData(vehicle_sim_api);
                }
            }
        }
    }
    return 0;
}

// This function is NOT run on the new thread!
void FSensorsThread::Stop()
{
    bRunThread = false;
}

void FSensorsThread::appendSensorsData(msr::airlib::VehicleSimApiBase* vehicle_sim_api)
{
    PawnSimApi* pawn = static_cast<PawnSimApi*>(vehicle_sim_api);

    if (msr::airlib::AirSimSettings::singleton().simmode_name == "Multirotor") {

        msr::airlib::AirSimSettings::CaptureSetting captureSettings = msr::airlib::AirSimSettings::singleton().getVehicleSetting(pawn->getVehicleName())->cameras.at("Left").capture_settings.at(0);

        float __fx = captureSettings.fx;
        float __fy = captureSettings.fy;
        float __cx = captureSettings.cx;
        float __cy = captureSettings.cy;

        FJsonIMUData imuJsonData;
        FJsonGPSData gpsJsonData;
        FJsonBarometerData baroJsonData;
        FJsonMagnometerData magnetoJsonData;

        FTransform cam_pose = pawn->getCamera("Left")->GetActorTransform();
        cam_pose.SetLocation(FVector::ZeroVector);

        MultirotorPawnSimApi* vehiclePawn = static_cast<MultirotorPawnSimApi*>(pawn);
        auto vehicleAPI = vehiclePawn->getVehicleApi();

        auto* imu = static_cast<const ImuSimple*>(vehicleAPI->findSensorByName("Imu", msr::airlib::SensorBase::SensorType::Imu));
        if (imu) {
            

            msr::airlib::ImuBase::Output imuData = vehicleAPI->getImuData("Imu");
            //UE_LOG(LogTemp, Warning, TEXT("IMU TS     %llu"), imuData.time_stamp);

            imuJsonData.EpochTimeStamp = IS_MICROSECONDS ? (imuData.time_stamp / 1000) : imuData.time_stamp;
                  
            FVector angularVelocityUU = vehiclePawn->getNedTransform().fromRelativeNed(imuData.angular_velocity);
            imuJsonData.angularVelocity = convertFromUUToImageCoordinateSystem(angularVelocityUU) / 100.0f; // ned -> UU fnct convert unit from meter to centimeter. / 100.0 to compensate it.

            FVector linearAccelerationUU = vehiclePawn->getNedTransform().fromRelativeNed(imuData.linear_acceleration);
            imuJsonData.linearAcceleration = convertFromUUToImageCoordinateSystem(linearAccelerationUU) / 100; // cm to m

            FQuat imuOrientationUU = vehiclePawn->getNedTransform().fromLocalNed(Pose(msr::airlib::Vector3r::Zero(), imuData.orientation)).GetRotation();
            imuJsonData.orientation = convertFromUUToUnityCoordinateSystem(FTransform(imuOrientationUU)).GetRotation();
        }
        else {
            imuJsonData.EpochTimeStamp = -1;
            imuJsonData.angularVelocity = FVector::ZeroVector;
            imuJsonData.linearAcceleration = FVector::ZeroVector;
            imuJsonData.orientation = FQuat::Identity;
        }

        if (vehicleAPI->findSensorByName("Barometer", msr::airlib::SensorBase::SensorType::Barometer)) {
            msr::airlib::BarometerBase::Output barometerData = vehicleAPI->getBarometerData("Barometer");
            //UE_LOG(LogTemp, Warning, TEXT("Baro TS    %llu"), barometerData.time_stamp);

            baroJsonData.EpochTimeStamp = IS_MICROSECONDS ? (barometerData.time_stamp / 1000) : barometerData.time_stamp;
            baroJsonData.altitude = barometerData.altitude;
            baroJsonData.pressure = barometerData.pressure;
            baroJsonData.qnh = barometerData.qnh;
        }
        else {
            baroJsonData.EpochTimeStamp = -1;
            baroJsonData.altitude = 0;
            baroJsonData.pressure = 0;
            baroJsonData.qnh = 0;
        }

        if (vehicleAPI->findSensorByName("Magnetometer", msr::airlib::SensorBase::SensorType::Magnetometer)) {
            msr::airlib::MagnetometerSimple::Output magnetometerData = vehicleAPI->getMagnetometerData("Magnetometer");
            //UE_LOG(LogTemp, Warning, TEXT("Magneto TS %llu"), magnetometerData.time_stamp);

            FVector magneticBody = FVector(magnetometerData.magnetic_field_body.x(), magnetometerData.magnetic_field_body.y(), magnetometerData.magnetic_field_body.z());
            magnetoJsonData.EpochTimeStamp = IS_MICROSECONDS ? (magnetometerData.time_stamp / 1000) : magnetometerData.time_stamp;
            magnetoJsonData.magneticFieldBody = convertFromUUToUnityCoordinateSystem(worldToCam(cam_pose, magneticBody), false);

            //UE_LOG(LogTemp, Warning, TEXT("Size %i"), magnetometerData.magnetic_field_covariance.size());
            //for (int i = 0; i < magnetometerData.magnetic_field_covariance.size(); i++) {
            //    magnetoJsonData.magneticFieldCovariance.Add(magnetometerData.magnetic_field_covariance[i]);
            //}
        }
        else
        {
            magnetoJsonData.EpochTimeStamp = -1;
            magnetoJsonData.magneticFieldBody = FVector::ZeroVector;
        }

        if (vehicleAPI->findSensorByName("Gps", msr::airlib::SensorBase::SensorType::Gps)) {
            msr::airlib::GpsSimple::Output gpsData = vehicleAPI->getGpsData("Gps");
            //UE_LOG(LogTemp, Warning, TEXT("GPS TS     %llu"), gpsData.time_stamp);
            //UE_LOG(LogTemp, Warning, TEXT("GPS UTC    %llu"), gpsData.gnss.time_utc);

            gpsJsonData.EpochTimeStamp = IS_MICROSECONDS ? (gpsData.time_stamp / 1000) : gpsData.time_stamp;
            gpsJsonData.eph = gpsData.gnss.eph;
            gpsJsonData.epv = gpsData.gnss.epv;
            gpsJsonData.velocity = FVector(gpsData.gnss.velocity.x(), gpsData.gnss.velocity.y(), gpsData.gnss.velocity.z());
            gpsJsonData.geoPoint.altitude = gpsData.gnss.geo_point.altitude;
            gpsJsonData.geoPoint.latitude = gpsData.gnss.geo_point.latitude;
            gpsJsonData.geoPoint.longitude = gpsData.gnss.geo_point.longitude;
        }
        else {
            gpsJsonData.EpochTimeStamp = -1;
            gpsJsonData.eph = 0;
            gpsJsonData.epv = 0;
            gpsJsonData.velocity = FVector::ZeroVector;
            gpsJsonData.geoPoint.altitude = 0;
            gpsJsonData.geoPoint.latitude = 0;
            gpsJsonData.geoPoint.longitude = 0;
        }

        jsonSensorsData[vehicle_sim_api->getVehicleName()].IMUData.Add(imuJsonData);
        jsonSensorsData[vehicle_sim_api->getVehicleName()].magnetometerData.Add(magnetoJsonData);
        jsonSensorsData[vehicle_sim_api->getVehicleName()].barometerData.Add(baroJsonData);
        jsonSensorsData[vehicle_sim_api->getVehicleName()].GPSData.Add(gpsJsonData);
    }
}