#pragma once

#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <IVRDriver.hpp>
#include <IMocapStreamSource.hpp>
#include <udpserver.h>
#include <concurrentqueue.h>

#include "segments.h"

class MVNStreamSource : public IMocapStreamSource {
public:
	virtual void init(MocapDriver::IVRDriver* owning_driver) override;
	virtual void PopulateTrackers() override;
	virtual void Close() override;
	virtual MocapDriver::IVRDriver* GetDriver() override;
	virtual PoseSample GetNextPose() override;
	virtual void QueuePose(const PoseSample& pose, const std::chrono::high_resolution_clock::time_point& timestamp);
	virtual std::string GetRenderModelPath(int segmentIndex);

private:
	void ReceiveMVNData(StreamingProtocol, const Datagram*);
	void WaitForPose(PoseSample& outPose);
	double ClampVelocity(double velocity, double max_value);

	std::string GetSettingsSegmentTarget(Segment segment);
	bool GetSettingsParentToHMD();
	MocapDriver::IVRDriver* driver_;
	std::unordered_map<Segment, std::shared_ptr<MocapDriver::IVRDevice>> trackers_;
	std::unique_ptr<UdpServer> mvn_udp_server_;
	
	std::mutex pose_update_mtx;
	std::mutex pose_submit_mtx;
	std::condition_variable pose_available;
	bool pose_processed;
	std::map <int32_t, PoseSample> incomplete_poses_;
	PoseSample completed_pose_;

	bool parent_to_HMD_ = true;
 };
