#include "VRDriver.hpp"
#include "HMDDevice.hpp"
#include "TrackerDevice.hpp"
#include "ControllerDevice.hpp"
#include "TrackingReferenceDevice.hpp"

#include <json.hpp>
#include <filesystem>
#include <fstream>
#include <vector>
#include <math.h>
#include <linalg.h>

#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES

#include <quaterniondatagram.h>

using namespace nlohmann;
using namespace MocapDriver;

vr::EVRInitError VRDriver::Init(vr::IVRDriverContext* pDriverContext)
{
    // Perform driver context initialisation
    if (vr::EVRInitError init_error = vr::InitServerDriverContext(pDriverContext); init_error != vr::EVRInitError::VRInitError_None) {
        return init_error;
    }
    
    LoadUniverseOrigin();

    // Create MVN stream source explicly
    // TODO: Load from config
    std::unique_ptr<MVNStreamSource> mvnStreamSrc = std::make_unique<MVNStreamSource>();
    mvnStreamSrc->init(this);
    mvnStreamSrc->PopulateTrackers();
    streamSources_.push_back(std::move(mvnStreamSrc));
  
	return vr::VRInitError_None;
}

void MocapDriver::VRDriver::LoadUniverseOrigin()
{
    // Load chaperone info from steam. THIS IS A DIRTY HACKY HACK HACK
    std::filesystem::path chaperone_config("C:/Program Files (x86)/Steam/config/chaperone_info.vrchap");
    if (std::filesystem::exists(chaperone_config)) {
        std::fstream file;
        file.open(chaperone_config, std::fstream::in);
        json chaperone_data;
        file >> chaperone_data;

        for (auto universe : chaperone_data["universes"]) {
            if (!universe.contains("standing")) {
                continue;
            }
            auto translation = universe["standing"]["translation"];

            // Translation is a directional vector from the lighthouse to the HMD, so invert the direction to get the offset
            // TODO: This doesn't need to be set for non-lighthouse headsets. I'm testing with a Meta Quest 3, but it's more likely that an MVN suit would be paired with lighthouse base stations.
            // Best that we check if we're using a lighthouse based tracking system in this area
            /*origin_.translation[0] = -translation[0].get<float>();
            origin_.translation[1] = -translation[1].get<float>();
            origin_.translation[2] = -translation[2].get<float>();*/

            // Same offset needed with the yaw, rotate it by PI radians
            origin_.yaw = universe["standing"]["yaw"].get<float>() + M_PI;
            Log("Chaperone Origin X: " + std::to_string(origin_.translation[0]) + " Origin Y: " + std::to_string(origin_.translation[1]) + " Origin Z: " + std::to_string(origin_.translation[2]) + " Origin Yaw: " + std::to_string(origin_.yaw) );
        }
    }
}

void VRDriver::Cleanup()
{
}

void VRDriver::RunFrame()
{
    //MessageBox(NULL,"hi", "Example Driver", MB_OK);
    // Collect events
    vr::VREvent_t event;
    std::vector<vr::VREvent_t> events;
    while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event)))
    {
        events.push_back(event);
    }
    this->openvr_events_ = events;

    // Update frame timing
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    this->frame_timing_ = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_frame_time_);
    this->last_frame_time_ = now;

    this->frame_timing_avg_ = this->frame_timing_avg_ * 0.9 + ((double)this->frame_timing_.count()) * 0.1;
    //MessageBox(NULL, std::to_string(((double)this->frame_timing_.count()) * 0.1).c_str(), "Example Driver", MB_OK);

    for (auto& device : this->devices_)
        device->Update();

}

bool VRDriver::ShouldBlockStandbyMode()
{
    return false;
}

void VRDriver::EnterStandby()
{
}

void VRDriver::LeaveStandby()
{
}

std::vector<std::shared_ptr<IVRDevice>> VRDriver::GetDevices()
{
    return this->devices_;
}

std::vector<vr::VREvent_t> VRDriver::GetOpenVREvents()
{
    return this->openvr_events_;
}

std::chrono::milliseconds VRDriver::GetLastFrameTime()
{
    return this->frame_timing_;
}

std::shared_ptr<IVRDevice> MocapDriver::VRDriver::CreateTrackerDevice(std::string serial, std::string role, IMocapStreamSource* motionSource, int segmentIndex)
{
    auto addtracker = std::make_shared<TrackerDevice>(serial, role);
    addtracker->SetMotionSource(motionSource);
    addtracker->SetSegmentIndex(segmentIndex);

    AddDevice(addtracker);
    addtracker->reinit(tracker_max_saved, tracker_max_time, tracker_smoothing, origin_);
    Log("Added tracker " + serial + " with role " + role);
    return addtracker;
}

bool VRDriver::AddDevice(std::shared_ptr<IVRDevice> device)
{
    vr::ETrackedDeviceClass openvr_device_class;
    // Remember to update this switch when new device types are added
    switch (device->GetDeviceType()) {
        case DeviceType::CONTROLLER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_Controller;
            break;
        case DeviceType::HMD:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_HMD;
            break;
        case DeviceType::TRACKER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker;
            break;
        case DeviceType::TRACKING_REFERENCE:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference;
            break;
        default:
            return false;
    }
    bool result = vr::VRServerDriverHost()->TrackedDeviceAdded(device->GetSerial().c_str(), openvr_device_class, device.get());
    if(result)
        this->devices_.push_back(device);
    return result;
}

SettingsValue VRDriver::GetSettingsValue(std::string key)
{
    vr::EVRSettingsError err = vr::EVRSettingsError::VRSettingsError_None;
    int int_value = vr::VRSettings()->GetInt32(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return int_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    float float_value = vr::VRSettings()->GetFloat(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return float_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    bool bool_value = vr::VRSettings()->GetBool(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return bool_value;
    }
    std::string str_value;
    str_value.reserve(1024);
    char* buf = (char*)malloc(sizeof(char) * 1024);
    vr::VRSettings()->GetString(settings_key_.c_str(), key.c_str(), buf, 1024, &err);
    str_value = std::string(buf);
    free(buf);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return str_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;

    return SettingsValue();
}

void VRDriver::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

UniverseOrigin MocapDriver::VRDriver::GetUniverseOrigin()
{
    return origin_;
}

vr::TrackedDevicePose_t MocapDriver::VRDriver::GetHMDPose()
{
    vr::TrackedDevicePose_t hmd_pose{};

    // GetRawTrackedDevicePoses expects an array.
    // We only want the hmd pose, which is at index 0 of the array so we can just pass the struct in directly, instead
    // of in an array
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, &hmd_pose, 1);

    return hmd_pose;
}

vr::IVRDriverInput* VRDriver::GetInput()
{
    return vr::VRDriverInput();
}

vr::CVRPropertyHelpers* VRDriver::GetProperties()
{
    return vr::VRProperties();
}

vr::IVRServerDriverHost* VRDriver::GetDriverHost()
{
    return vr::VRServerDriverHost();
}

//-----------------------------------------------------------------------------
// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdQuaternion_t VRDriver::GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdVector3_t VRDriver::GetPosition(vr::HmdMatrix34_t matrix) {
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}
