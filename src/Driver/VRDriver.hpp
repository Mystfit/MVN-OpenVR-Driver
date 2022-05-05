#pragma once
#define NOMINMAX

#include <vector>
#include <memory>

#include <openvr_driver.h>

#include "IVRDriver.hpp"
#include "TrackerDevice.hpp"
#include "ControllerDevice.hpp"
#include "TrackingReferenceDevice.hpp"
#include "ControllerDevice.hpp"

namespace MocapDriver {
    class VRDriver : public IVRDriver {
    public:

        // Inherited via IVRDriver
        virtual std::vector<std::shared_ptr<IVRDevice>> GetDevices() override;
        virtual std::vector<vr::VREvent_t> GetOpenVREvents() override;
        virtual std::chrono::milliseconds GetLastFrameTime() override;
        virtual std::shared_ptr<IVRDevice> CreateTrackerDevice(std::string serial, std::string role, IMocapStreamSource* motionSource) override;
        virtual bool AddDevice(std::shared_ptr<IVRDevice> device) override;
        virtual SettingsValue GetSettingsValue(std::string key) override;
        virtual void Log(std::string message) override;
        virtual UniverseOrigin GetUniverseOrigin() override;

        virtual vr::IVRDriverInput* GetInput() override;
        virtual vr::CVRPropertyHelpers* GetProperties() override;
        virtual vr::IVRServerDriverHost* GetDriverHost() override;

        // Inherited via IServerTrackedDeviceProvider
        virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
        void LoadUniverseOrigin();
        virtual void Cleanup() override;
        virtual void RunFrame() override;
        virtual bool ShouldBlockStandbyMode() override;
        virtual void EnterStandby() override;
        virtual void LeaveStandby() override;
        virtual ~VRDriver() = default;

    private:        
        //void PopulateTrackers();

        // TODO: Move into MVNMocapSource implementation
        /*std::string GetSettingsSegmentTarget(Segment segment);
        std::unique_ptr<UdpServer> mvn_udp_server;
        void ReceiveMVNData(StreamingProtocol, const Datagram*);*/


        std::string version = "0.0.1";

        std::shared_ptr<MocapDriver::ControllerDevice> fakemove_;
        std::vector<std::shared_ptr<IVRDevice>> devices_;
        std::vector<std::shared_ptr<TrackingReferenceDevice>> stations_;
        std::vector<vr::VREvent_t> openvr_events_;
        std::chrono::milliseconds frame_timing_ = std::chrono::milliseconds(16);
        double frame_timing_avg_ = 16;
        std::chrono::system_clock::time_point last_frame_time_ = std::chrono::system_clock::now();
        std::string settings_key_ = "MVN";

        vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
        vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);
        void PipeThread();

        int pipeNum = 1;
        double smoothFactor = 0.2;
        
        UniverseOrigin origin_;

        int tracker_max_saved = 10;
        double tracker_max_time = 1;
        double tracker_smoothing = 0;
    };
};
