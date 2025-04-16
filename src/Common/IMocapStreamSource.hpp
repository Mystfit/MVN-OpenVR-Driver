#pragma once

#include <memory>
#include <chrono>

struct SegmentSample {
	double translation[3];
	double rotation_quat[4];
	double velocity[3];
	double angular_velocity[3];
};

struct PoseSample {
	int32_t pose_id;
	std::vector<SegmentSample> segments;
	std::chrono::high_resolution_clock::time_point timestamp;
};

// Forwards 
namespace MocapDriver {
	class IVRDriver;
}

class IMocapStreamSource {
public:
	virtual void init(MocapDriver::IVRDriver* owning_driver) = 0;
	virtual std::string GetRenderModelPath(int segmentIndex) = 0;
	virtual void PopulateTrackers() = 0;
	virtual MocapDriver::IVRDriver* GetDriver() = 0;
	virtual void WaitForPose(PoseSample& outPose) = 0;
	virtual void QueuePose(const PoseSample& pose, const std::chrono::high_resolution_clock::time_point& timestamp) = 0;
	virtual PoseSample GetNextPose() = 0;
};