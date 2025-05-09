#pragma once
#include <memory>
#include <vector>
#include <chrono>
#include <variant>

#include "IVRDevice.hpp"

namespace MocapDriver {

    typedef std::variant<std::monostate, std::string, int, float, bool> SettingsValue;

    struct UniverseOrigin {
        double translation[3] = {};
        double yaw = 0.0;
    };

    class IVRDriver : protected vr::IServerTrackedDeviceProvider {
    public:

        /// <summary>
        /// Returns all devices being managed by this driver
        /// </summary>
        /// <returns>All managed devices</returns>
        virtual std::vector<std::shared_ptr<IVRDevice>> GetDevices() = 0;

        virtual std::shared_ptr<IVRDevice> CreateTrackerDevice(std::string serial, std::string role, IMocapStreamSource* motionSource = nullptr, int segmentIndex = -1) = 0;

        /// <summary>
        /// Returns all OpenVR events that happened on the current frame
        /// </summary>
        /// <returns>Current frame's OpenVR events</returns>
        virtual std::vector<vr::VREvent_t> GetOpenVREvents() = 0;

        /// <summary>
        /// Returns the milliseconds between last frame and this frame
        /// </summary>
        /// <returns>MS between last frame and this frame</returns>
        virtual std::chrono::milliseconds GetLastFrameTime() = 0;

        /// <summary>
        /// Adds a device to the driver
        /// </summary>
        /// <param name="device">Device instance</param>
        /// <returns>True on success, false on failure</returns>
        virtual bool AddDevice(std::shared_ptr<IVRDevice> device) = 0;

        /// <summary>
        /// Returns the value of a settings key
        /// </summary>
        /// <param name="key">The settings key</param>
        /// <returns>Value of the key, std::monostate if the value is malformed or missing</returns>
        virtual SettingsValue GetSettingsValue(std::string key) = 0;

        /// <summary>
        /// Gets the OpenVR VRDriverInput pointer
        /// </summary>
        /// <returns>OpenVR VRDriverInput pointer</returns>
        virtual vr::IVRDriverInput* GetInput() = 0;

        /// <summary>
        /// Gets the OpenVR VRDriverInput pointer
        /// </summary>
        /// <returns>OpenVR VRDriverInput pointer</returns>
        virtual UniverseOrigin GetUniverseOrigin() = 0;

        /// <summary>
        /// Gets the OpenVR VRDriverProperties pointer
        /// </summary>
        /// <returns>OpenVR VRDriverProperties pointer</returns>
        virtual vr::CVRPropertyHelpers* GetProperties() = 0;

        /// <summary>
        /// Gets the OpenVR VRServerDriverHost pointer
        /// </summary>
        /// <returns>OpenVR VRServerDriverHost pointer</returns>
        virtual vr::IVRServerDriverHost* GetDriverHost() = 0;

        /// <summary>
        /// Gets the OpenVR VRServerDriverHost pointer
        /// </summary>
        /// <returns>OpenVR VRServerDriverHost pointer</returns>
        virtual vr::TrackedDevicePose_t GetHMDPose() = 0;

        /// <summary>
        /// Writes a log message
        /// </summary>
        /// <param name="message">Message to log</param>
        virtual void Log(std::string message) = 0;

        virtual inline const char* const* GetInterfaceVersions() override {
            return vr::k_InterfaceVersions;
        };

        virtual ~IVRDriver() {}

    };
}

