#pragma once

#include <chrono>
#include "cmath_fix.h"

#include <linalg.h>

#include "IVRDevice.hpp"
#include "DriverFactory.hpp"

#include <thread>
#include <sstream>
#include <iostream>
#include <string>
#include <thread>

namespace MocapDriver {
    class TrackerDevice : public IVRDevice {
        public:
            TrackerDevice(std::string serial, std::string role);
            ~TrackerDevice() = default;

            // Inherited via IVRDevice
            virtual std::string GetSerial() override;
            virtual void Update() override;
            //virtual void save_current_pose(double a, double b, double c, double qw, double qx, double qy, double qz, double time);
            //virtual int get_next_pose(double req_time, double pred[]);
            virtual vr::TrackedDeviceIndex_t GetDeviceIndex() override;
            virtual DeviceType GetDeviceType() override;
            virtual void Log(std::string message);
            virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
            virtual void Deactivate() override;
            virtual void EnterStandby() override;
            virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
            virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
            virtual vr::DriverPose_t GetPose() override;
            virtual void SubmitPose() override;

            virtual void reinit(int msaved, double mtime, double msmooth, UniverseOrigin origin = {});
            
            // Inherited via IVRDevice mocap additions
            // TODO: Put in seperate interface?
            virtual void SetMotionSource(IMocapStreamSource* motionSource);
            virtual void SetSegmentIndex(int segmentIndex) override;
            virtual int GetSegmentIndex() override;
            virtual IMocapStreamSource* GetMotionSource();
    private:
        vr::TrackedDeviceIndex_t device_index_ = vr::k_unTrackedDeviceIndexInvalid;
        std::string serial_;
        std::string role_;
        bool isSetup;

        std::chrono::high_resolution_clock::time_point _pose_timestamp;

        vr::DriverPose_t last_pose_ = IVRDevice::MakeDefaultPose();

        bool did_vibrate_ = false;
        float vibrate_anim_state_ = 0.f;

        vr::VRInputComponentHandle_t haptic_component_ = 0;

        vr::VRInputComponentHandle_t system_click_component_ = 0;
        vr::VRInputComponentHandle_t system_touch_component_ = 0;

        int max_saved = 10;
        std::vector<std::vector<double>> prev_positions; // prev_positions[:][0] je time since now (koliko cajta nazaj se je naredl, torej min-->max)
        double last_update = 0;
        double max_time = 1;
        double smoothing = 0;

        IMocapStreamSource* motionSource_;
        int segmentIndex_;
        double translation_origin[3] = {};
        vr::HmdQuaternion_t rotation_origin;

        // Update thread
        std::thread pose_submit_thread;
    };
};

void eulerToQuaternion(double eulerXYZ[], double* out_quat);
