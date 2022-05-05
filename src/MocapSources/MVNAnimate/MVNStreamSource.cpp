#include <linalg.h>

#include "MVNStreamSource.h"
#include "quaterniondatagram.h"
#include <PoseMath.hpp>
#include <linearsegmentkinematicsdatagram.h>

void MVNStreamSource::init(std::shared_ptr<MocapDriver::IVRDriver> owning_Driver)
{
	driver_ = owning_Driver;

    int port = 9763;
    std::string hostDestinationAddress = "localhost";
    mvn_udp_server_ = std::make_unique<UdpServer>(
        hostDestinationAddress,
        (uint16_t)port,
        [this](StreamingProtocol protocol, const Datagram* message) {
            this->ReceiveMVNData(protocol, message);
        });
    GetDriver()->Log("Created MVN listen server");
}

void MVNStreamSource::PopulateTrackers()
{
    for (auto segment : SegmentName) {
        std::string segment_hint = GetSettingsSegmentTarget(segment.first);
        if (segment_hint.compare("disabled")) {
            std::string name = SegmentName.at(segment.first);
            std::string role = segment.second;

            auto tracker = GetDriver()->CreateTrackerDevice(name, role);

            std::string rendermodel = std::string("{MVN}/rendermodels/XSens/") + tracker->GetSerial();
            GetDriver()->Log("Tracker rendermodel path: " + rendermodel);

            trackers_.emplace(segment.first, tracker);
        }
    }
}

std::shared_ptr<MocapDriver::IVRDriver> MVNStreamSource::GetDriver()
{
    return driver_;
}

PoseSample MVNStreamSource::GetNextPose()
{
    std::scoped_lock<std::mutex> lock(pose_update_mtx);
    return PoseSample(completed_pose_);
}


std::string MVNStreamSource::GetSettingsSegmentTarget(Segment segment)
{
    std::string prefix = "Role_";
    std::string key = prefix + SegmentName.at(segment);
    return std::get<std::string>(GetDriver()->GetSettingsValue(key));
}

void MVNStreamSource::ReceiveMVNData(StreamingProtocol protocol, const Datagram* message)
{
    int32_t msg_id = message->sampleCounter();
    bool pose_is_complete = true;

    // Create a new pose if it isn't already being filled
    if (incomplete_poses_.find(msg_id) == incomplete_poses_.end()) {
        incomplete_poses_.emplace(msg_id, PoseSample{ msg_id, std::vector<SegmentSample>(SegmentName.size()) });
        pose_is_complete = false;
    } 

    for (auto tracker_pair : trackers_) {
        auto segment = tracker_pair.first;

        // Find incomplete pose segment for us to fill
        auto segment_it = incomplete_poses_[msg_id].segments.begin() + segment;
    
        if (protocol == StreamingProtocol::SPPoseQuaternion) {
            const QuaternionDatagram* quat_msg = static_cast<const QuaternionDatagram*>(message);
            auto segment_data = quat_msg->GetSegmentData(segment);
            
            if (segment_data.segmentId > -1){
                // Parse MVN formatted data into a matrix to perform coordinate system conversions
                linalg::vec <float, 4> segment_quat(segment_data.orientation[1], segment_data.orientation[2], segment_data.orientation[3], segment_data.orientation[0]);
                linalg::vec <float, 4> segment_quat_normalized = linalg::normalize(segment_quat);
                auto transformMatrix = linalg::pose_matrix(
                    segment_quat_normalized,
                    linalg::vec<float, 3>(segment_data.position[0], segment_data.position[1], segment_data.position[2])
                );

                // Convert MVN Animate Z-up to OpenVR Y-up
                linalg::mat<float, 4, 4> vrMatrix = ConvertZtoYUp(transformMatrix);

                // Pull quaternion out of segment transformation matrix
                linalg::vec<float, 4> rotQuat = linalg::rotation_quat(GetRotationMatrixFromTransform(vrMatrix));

                segment_it->translation[0] = vrMatrix.w.x;
                segment_it->translation[1] = vrMatrix.w.y;
                segment_it->translation[2] = vrMatrix.w.z;
                segment_it->rotation_quat[0] = rotQuat.w;
                segment_it->rotation_quat[1] = rotQuat.x;
                segment_it->rotation_quat[2] = rotQuat.y;
                segment_it->rotation_quat[3] = rotQuat.z;
            }
        }
        else if (protocol == StreamingProtocol::SPLinearSegmentKinematics) {
            const LinearSegmentKinematicsDatagram* linear_kinematics_msg = static_cast<const LinearSegmentKinematicsDatagram*>(message);
            auto segment_data = linear_kinematics_msg->GetSegmentData(segment);
            
            if (segment_data.segmentId > -1) {
                auto velocity_matrix = GetTransformMatrixFromVector(
                    linalg::vec<float, 3>{segment_data.velocity[0], segment_data.velocity[1], segment_data.velocity[2]},
                    linalg::vec<float, 3>{0.0, 0.0, 1.0}
                );

                // Convert MVN Animate Z-up to OpenVR Y-up
                linalg::mat<float, 4, 4> vrMatrix = ConvertZtoYUp(velocity_matrix);

                segment_it->velocity[0] = vrMatrix.x.z;
                segment_it->velocity[1] = vrMatrix.y.z;
                segment_it->velocity[2] = vrMatrix.z.z;
            }
        }
    }
    
    // Save stored pose
    if (pose_is_complete) {
        {
            std::scoped_lock<std::mutex> lock(pose_update_mtx);
            completed_pose_ = incomplete_poses_[msg_id];
        }
        incomplete_poses_.erase(msg_id);
    }
}