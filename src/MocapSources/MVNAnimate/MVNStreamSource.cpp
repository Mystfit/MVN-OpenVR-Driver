#include <linalg.h>
#include <chrono>

#include "MVNStreamSource.h"
#include "quaterniondatagram.h"
#include <PoseMath.hpp>
#include <linearsegmentkinematicsdatagram.h>
#include <algorithm>

void MVNStreamSource::init(MocapDriver::IVRDriver* owning_driver)
{
    IMocapStreamSource::init(owning_driver);

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

void MVNStreamSource::Close()
{
    mvn_udp_server_->stopThread();
}

void MVNStreamSource::PopulateTrackers()
{
    for (auto segment : SegmentName) {
        std::string segment_hint = GetSettingsSegmentTarget(segment.first);
        if (segment_hint.compare("disabled")) {
            std::string name = SegmentName.at(segment.first);
            std::string role = segment.second;

            auto tracker = GetDriver()->CreateTrackerDevice(name, role, this, segment.first);

            std::string rendermodel = std::string("{Mocap}/rendermodels/XSens/") + tracker->GetSerial();
            GetDriver()->Log("Tracker rendermodel path: " + rendermodel);

            trackers_.emplace(segment.first, tracker);
        }
    }
}

PoseSample MVNStreamSource::GetNextPose()
{
    std::scoped_lock<std::mutex> lock(pose_update_mtx);
    return completed_pose_;
}

void MVNStreamSource::QueuePose(const PoseSample& pose, const std::chrono::high_resolution_clock::time_point& timestamp)
{
    {
        std::scoped_lock<std::mutex> lock(pose_update_mtx);
        completed_pose_ = pose;
        completed_pose_.timestamp = timestamp;
    }

    // Update all tracker threads
    /*std::unique_lock<std::mutex> lck(pose_submit_mtx);
    pose_processed = true;
    lck.unlock();
    pose_available.notify_all();
    lck.lock();
    pose_processed = false;*/
    for (auto pair : trackers_) {
        pair.second->SubmitPose();
    }
}

std::string MVNStreamSource::GetRenderModelPath(int segmentIndex)
{
    // Relative to "{Mocap}/rendermodels"
    return std::string("XSens/") + SegmentName.at((Segment)segmentIndex); //"{htc}/rendermodels/vr_tracker_vive_1_0"; 
}


std::string MVNStreamSource::GetSettingsSegmentTarget(Segment segment)
{
    std::string prefix = "Role_";
    std::string key = prefix + SegmentName.at(segment);

    vr::EVRSettingsError err = vr::EVRSettingsError::VRSettingsError_None;
    char* buf = (char*)malloc(sizeof(char) * 1024);
    vr::VRSettings()->GetString("MVN", key.c_str(), buf, 1024, &err);
    std::string str_value(buf);
    free(buf);

    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return str_value;
    }

    return "";
}

void MVNStreamSource::ReceiveMVNData(StreamingProtocol protocol, const Datagram* message)
{
    if (protocol != StreamingProtocol::SPPoseQuaternion)
        return;

    int32_t msg_id = message->sampleCounter();
    bool pose_is_complete = false;

    // Dirty pose so trackers have to wait
    pose_processed = false;

    // Create a new pose if it isn't already being filled
    if (incomplete_poses_.find(msg_id) == incomplete_poses_.end()) {
        incomplete_poses_.emplace(msg_id, PoseSample{ msg_id, std::vector<SegmentSample>(SegmentName.size()) });
    }

    // Improved timestamp handling - store exact message arrival time
    std::chrono::high_resolution_clock::time_point current_pose_time =
        std::chrono::high_resolution_clock::now();

    // Store time delta in seconds with message - improved precision with double
    double pose_delta_time;

    // Use a minimum delta time to avoid division by very small numbers which can cause velocity spikes
    const double MIN_DELTA_TIME = 0.001; // 1ms minimum to prevent huge velocities

    if (completed_pose_.timestamp.time_since_epoch().count() > 0) {
        double delta_time = std::chrono::duration<double>((current_pose_time - completed_pose_.timestamp)).count();
        pose_delta_time = max(MIN_DELTA_TIME, delta_time);
    }
    else {
        // First pose - can't calculate velocity yet
        pose_delta_time = 0.0;
    }

    for (auto tracker_pair : trackers_) {
        Segment segment = tracker_pair.first;

        auto segment_it = incomplete_poses_[msg_id].segments.begin() + segment;

        if (protocol == StreamingProtocol::SPPoseQuaternion) {
            const QuaternionDatagram* quat_msg = static_cast<const QuaternionDatagram*>(message);
            auto segment_data = quat_msg->GetSegmentData(segment);

            if (segment_data.segmentId > -1) {
                // Parse MVN formatted data into a matrix to perform coordinate system conversions
                linalg::vec <float, 4> segment_quat(segment_data.orientation[1], segment_data.orientation[2], segment_data.orientation[3], segment_data.orientation[0]);
                linalg::vec <float, 4> segment_quat_normalized = linalg::normalize(segment_quat);
                auto transformMatrix = linalg::pose_matrix(
                    segment_quat_normalized,
                    linalg::vec<float, 3>(segment_data.position[0], segment_data.position[1], segment_data.position[2])
                );
            }

#ifndef MVN_SUPPORTS_LINEAR_KINEMATICS 
            pose_is_complete = true;
#endif
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

                // Get velocity values from the converted matrix
                double raw_vel_x = vrMatrix.x.z;
                double raw_vel_y = vrMatrix.y.z;
                double raw_vel_z = vrMatrix.z.z;

                // Apply smoothing to these velocities too if we have previous values
                const float alpha = 0.3f; // Lower = more smoothing
                const double MAX_VELOCITY = 5.0; // 5 meters per second max velocity

                if (segment < completed_pose_.segments.size()) {
                    segment_it->velocity[0] = ClampVelocity(alpha * raw_vel_x + (1.0f - alpha) * completed_pose_.segments[segment].velocity[0], MAX_VELOCITY);
                    segment_it->velocity[1] = ClampVelocity(alpha * raw_vel_y + (1.0f - alpha) * completed_pose_.segments[segment].velocity[1], MAX_VELOCITY);
                    segment_it->velocity[2] = ClampVelocity(alpha * raw_vel_z + (1.0f - alpha) * completed_pose_.segments[segment].velocity[2], MAX_VELOCITY);
                }
                else {
                    segment_it->velocity[0] = ClampVelocity(raw_vel_x, MAX_VELOCITY);
                    segment_it->velocity[1] = ClampVelocity(raw_vel_y, MAX_VELOCITY);
                    segment_it->velocity[2] = ClampVelocity(raw_vel_z, MAX_VELOCITY);
                }
            }
        }
        else {
            GetDriver()->Log("Received unknown MVN packet " + std::to_string(protocol));
        }
    }

    // Save stored pose
    if (pose_is_complete) {
        QueuePose(incomplete_poses_[msg_id], current_pose_time);
        incomplete_poses_.erase(msg_id);
    }
}

// Helper function to clamp velocity values
double MVNStreamSource::ClampVelocity(double velocity, double max_value) {
    if (velocity > max_value) return max_value;
    if (velocity < -max_value) return -max_value;
    return velocity;
}

void MVNStreamSource::WaitForPose(PoseSample& outPose)
{
    std::unique_lock<std::mutex> lk(pose_submit_mtx);
    pose_available.wait(lk, [this]{ return pose_processed; });
    outPose = GetNextPose();
}