#pragma once

#include <memory>
#include <linalg.h>
#include <unordered_map>
#include <IVRDriver.hpp>

struct SegmentSample {
	linalg::mat<float, 4, 4> localTransform;
	linalg::mat<float, 4, 4> worldTransform;
	double translation[3];
	double rotation_quat[4];
	double velocity[3];
};

struct PoseSample {
	int32_t pose_id;
	std::unordered_map<int, SegmentSample> segments;
};

// Forwards 
//namespace MocapDriver {
//	class IVRDriver;
//}

class IMocapStreamSource {
public:
	virtual void init(MocapDriver::IVRDriver* owning_driver) {
		driver_ = owning_driver;
	}
	virtual std::string GetRenderModelPath(int segmentIndex) = 0;
	virtual void PopulateTrackers() = 0;
	virtual void Close() = 0;
	virtual MocapDriver::IVRDriver* GetDriver() { return driver_; }

	virtual void QueuePose(const PoseSample& pose) = 0;
	virtual PoseSample GetNextPose() = 0;

	vr::TrackedDevicePose_t GetHMDPose()
	{
		vr::TrackedDevicePose_t poses[1];
		GetDriver()->GetDriverHost()->GetRawTrackedDevicePoses(0, poses, 1);
		if (poses[0].bDeviceIsConnected && poses[0].bPoseIsValid) {
			return poses[0];
		}

		return vr::TrackedDevicePose_t();
	}

	vr::TrackedDevicePose_t GetHMDPoseCorrected()
	{
		auto manager = vr::VRDriverManager();
		//GetDriverHandle

		return vr::TrackedDevicePose_t();
	}


	

private:
	MocapDriver::IVRDriver* driver_;
};