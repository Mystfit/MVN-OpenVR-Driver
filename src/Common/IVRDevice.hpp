#pragma once

#include <variant>
#include <openvr_driver.h>
#include <DeviceType.hpp>

class IMocapStreamSource;

namespace MocapDriver {

    class IVRDevice : public vr::ITrackedDeviceServerDriver {
    public:
        /// <summary>
        /// Returns the serial string for this device
        /// </summary>
        /// <returns>Device serial</returns>
        virtual std::string GetSerial() = 0;

        /// <summary>
        /// Runs any update logic for this device.
        /// Called once per frame
        /// </summary>
        virtual void Update() = 0;

        /// <summary>
        /// Returns the OpenVR device index
        /// This should be 0 for HMDs
        /// </summary>
        /// <returns>OpenVR device index</returns>
        virtual vr::TrackedDeviceIndex_t GetDeviceIndex() = 0;

        /// <summary>
        /// Returns the segment of a mocap source this tracker represents
        /// </summary>
        /// <returns>Mocap segment index</returns>
        virtual int GetSegmentIndex() = 0;
        
        /// <summary>
        /// Returns which type of device this device is
        /// </summary>
        /// <returns>The type of device</returns>
        virtual DeviceType GetDeviceType() = 0;

        /// <summary>
        /// Returns the source of this trackers motion data
        /// </summary>
        /// <returns>The type of device</returns>
        virtual IMocapStreamSource* GetMotionSource() = 0;

        /// <summary>
        /// Set the source of this trackers motion data
        /// </summary>
        /// <returns>The type of device</returns>
        virtual void SetMotionSource(IMocapStreamSource* motionSource) = 0;

        /// <summary>
        /// Sets the segment of a mocap source this tracker represents
        /// </summary>
        virtual void SetSegmentIndex(int segmentIndex) = 0;

       /// <summary>
        /// Submits a new pose to the tracker
        /// </summary>
        virtual void SubmitPose() = 0;

        
        /// <summary>
        /// Makes a default device pose 
        /// </summary>
        /// <returns>Default initialised pose</returns>
        static inline vr::DriverPose_t MakeDefaultPose(bool connected = true, bool tracking = true) {
            vr::DriverPose_t out_pose = { 0 };

            out_pose.deviceIsConnected = connected;
            out_pose.poseIsValid = tracking;
            out_pose.result = tracking ? vr::ETrackingResult::TrackingResult_Running_OK : vr::ETrackingResult::TrackingResult_Running_OutOfRange;
            out_pose.willDriftInYaw = false;
            out_pose.shouldApplyHeadModel = false;
            out_pose.qDriverFromHeadRotation.w = out_pose.qWorldFromDriverRotation.w = out_pose.qRotation.w = 1.0;

            return out_pose;
        }

        // Inherited via ITrackedDeviceServerDriver
        virtual vr::EVRInitError Activate(uint32_t unObjectId) = 0;
        virtual void Deactivate() = 0;
        virtual void EnterStandby() = 0;
        virtual void* GetComponent(const char* pchComponentNameAndVersion) = 0;
        virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
        virtual vr::DriverPose_t GetPose() = 0;

        ~IVRDevice() = default;
    };
};