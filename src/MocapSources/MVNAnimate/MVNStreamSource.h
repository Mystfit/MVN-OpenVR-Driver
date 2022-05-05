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

class MVNStreamSource : public IMocapStreamSource {
public:
	virtual void init(std::shared_ptr<MocapDriver::IVRDriver> owning_Driver) override;
	virtual void PopulateTrackers() override;
	virtual std::shared_ptr<MocapDriver::IVRDriver> GetDriver() override;
	virtual PoseSample GetNextPose() override;

private:

	void ReceiveMVNData(StreamingProtocol, const Datagram*);

	std::string GetSettingsSegmentTarget(Segment segment);
	std::shared_ptr<MocapDriver::IVRDriver> driver_;
	std::unordered_map<Segment, std::shared_ptr<MocapDriver::IVRDevice>> trackers_;
	std::unique_ptr<UdpServer> mvn_udp_server_;

	std::map <int32_t, PoseSample> incomplete_poses_;
	
	std::mutex pose_update_mtx;
	PoseSample completed_pose_;
	//moodycamel::ConcurrentQueue< PoseSample > completed_poses_;
 };
