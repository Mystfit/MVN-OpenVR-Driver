#pragma once

#include <memory>

struct SegmentSample {
	double translation[3];
	double rotation_quat[4];
	double velocity[3];
};

struct PoseSample {
	int32_t pose_id;
	std::vector<SegmentSample> segments;
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
	virtual void QueuePose(const PoseSample& pose) = 0;
	virtual PoseSample GetNextPose() = 0;
};