#pragma once

#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <IVRDriver.hpp>
#include <IMocapStreamSource.hpp>
#include <udpserver.h>
#include <concurrentqueue.h>

#include "segments.h"

#define MVN_STREAM "MVN"

class MVNStreamSource : public IMocapStreamSource {
public:
	virtual void Init(MocapDriver::IVRDriver* owning_driver) override;
	virtual bool Connect() override;
	virtual void Update() override {};
	virtual void PopulateTrackers() override;
	virtual MocapDriver::IVRDriver* GetDriver() override;
	virtual PoseSample GetNextPose() override;
	virtual void QueuePose(const PoseSample& pose);
	virtual std::string GetRenderModelPath(int segmentIndex);

private:
	void ReceiveMVNData(StreamingProtocol, const Datagram*);

	std::string GetSegmentRole(MVNSegment segment);
	MocapDriver::IVRDriver* driver_;
	std::unordered_map<MVNSegment, std::shared_ptr<MocapDriver::IVRDevice>> trackers_;
	std::unique_ptr<UdpServer> mvn_udp_server_;

	std::mutex pose_update_mtx;
	std::map <int32_t, PoseSample> incomplete_poses_;
	PoseSample completed_pose_;
 };
