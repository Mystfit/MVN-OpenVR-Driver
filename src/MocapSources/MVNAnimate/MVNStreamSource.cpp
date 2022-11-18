#include <linalg.h>

#include "MVNStreamSource.h"
#include "quaterniondatagram.h"
#include <PoseMath.hpp>
#include <linearsegmentkinematicsdatagram.h>
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES

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
    return completed_pose_;
}

void MVNStreamSource::QueuePose(const PoseSample& pose)
{
    std::scoped_lock<std::mutex> lock(pose_update_mtx);
    completed_pose_ = pose;
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
    bool pose_is_complete = true;

    // Create a new pose if it isn't already being filled
    if (incomplete_poses_.find(msg_id) == incomplete_poses_.end()) {
        incomplete_poses_.emplace(msg_id, PoseSample{ msg_id });
        
#ifdef MVN_SUPPORTS_LINEAR_KINEMATICS 
        pose_is_complete = false;
#endif
    }

    //linalg::mat<float, 4, 4> HMDHeadLocalOffset;
    linalg::vec<float, 3> HMDLocationOffset;
    linalg::vec<float, 4> HMDRotationOffset;
    linalg::mat<float, 4, 4> HMDHeadLocalOffset;
    linalg::mat<float, 4, 4> HeadRootWorldTransform;
    linalg::mat<float, 4, 4> HeadOrigWorldTransform;

    // Try to get HMD pose relative to steamvr space

    // Dump driver names
    //auto manager = vr::VRDriverManager();
    //for (size_t idx = 0; idx < manager->GetDriverCount(); idx++) {
    //    char name[255];
    //    manager->GetDriverName(idx, name, 255);
    //    GetDriver()->Log("Driver name: " + std::string(name));
    //}
    //auto hmd_driver = manager->GetDriverHandle("oculus");

    auto HMDPose = GetHMDPose();
    linalg::mat<float, 4, 4> HMDTransform{
       {HMDPose.mDeviceToAbsoluteTracking.m[0][0], HMDPose.mDeviceToAbsoluteTracking.m[0][1], HMDPose.mDeviceToAbsoluteTracking.m[0][2], HMDPose.mDeviceToAbsoluteTracking.m[0][3]},
       {HMDPose.mDeviceToAbsoluteTracking.m[1][0], HMDPose.mDeviceToAbsoluteTracking.m[1][1], HMDPose.mDeviceToAbsoluteTracking.m[1][2], HMDPose.mDeviceToAbsoluteTracking.m[1][3]},
       {HMDPose.mDeviceToAbsoluteTracking.m[2][0], HMDPose.mDeviceToAbsoluteTracking.m[2][1], HMDPose.mDeviceToAbsoluteTracking.m[2][2], HMDPose.mDeviceToAbsoluteTracking.m[2][3]},
       {0.0f, 0.0f, 0.0f, 1.0f}
    };
    linalg::vec<float, 4> HMDRotation = linalg::rotation_quat(GetRotationMatrixFromTransform(HMDTransform));
    linalg::vec<float, 3> HMDLocation = linalg::vec<float, 3>(HMDTransform.x.w, HMDTransform.y.w, HMDTransform.z.w);

    for (auto tracker_pair : trackers_) {
        Segment segment = tracker_pair.first;

        // Find incomplete pose segment for us to fill
        incomplete_poses_[msg_id].segments.emplace(segment, SegmentSample{});
        auto& segment_it = incomplete_poses_[msg_id].segments.at(segment);
        
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
            linalg::mat<float, 4, 4> segmentWorldTransform = ConvertZtoYUp(transformMatrix);

            // Save original worldspace transform for this segment
            segment_it.worldTransform = segmentWorldTransform;

            if (segment == Segment::Head) {
                HeadOrigWorldTransform = segmentWorldTransform;

                auto HeadDeltaTransform = linalg::mul(HMDTransform, linalg::inverse(segment_it.worldTransform));
                auto HeadDeltaRotation = GetRotationMatrixFromTransform(HeadDeltaTransform);
                auto HeadDeltaLocation = linalg::vec<float, 3>(HeadDeltaTransform.x.w, HeadDeltaTransform.y.w, HeadDeltaTransform.z.w);
                //HMDHeadLocalOffset = HeadDeltaTransform;

                auto yaw_only_hmd_rot = GetYAxisRotation(linalg::rotation_quat(HeadDeltaRotation));
                auto rotate_yaw_quat = linalg::mul(linalg::rotation_matrix(CreateFromAxisAngle(0.0, 1.0, 0.0, M_PI)), linalg::rotation_matrix(yaw_only_hmd_rot));
                auto inverted_yaw = linalg::inverse(rotate_yaw_quat);
                HMDHeadLocalOffset = inverted_yaw;//linalg::pose_matrix(inverted_yaw, HeadDeltaLocation);
                HMDHeadLocalOffset.w.x = HeadDeltaLocation.x;
                HMDHeadLocalOffset.w.y = HeadDeltaLocation.y;
                HMDHeadLocalOffset.w.z = HeadDeltaLocation.z;
                
                   
                auto VerifyLoc = GetTranslationFromTransformMatrix(HeadRootWorldTransform);
                GetDriver()->Log("HMDLoc: " + std::to_string(HeadDeltaLocation.x) + "Y: " + std::to_string(HeadDeltaLocation.y) + "Z: " + std::to_string(HeadDeltaLocation.z));



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

                segment_it.velocity[0] = vrMatrix.x.z;
                segment_it.velocity[1] = vrMatrix.y.z;
                segment_it.velocity[2] = vrMatrix.z.z;
            }
        }
    }


    /*auto HMDRotation = GetRotation(HMDPose.mDeviceToAbsoluteTracking);
    GetDriver()->Log("X: " + std::to_string(HMDRotation.x) + "Y: " + std::to_string(HMDRotation.y) + "Z: " + std::to_string(HMDRotation.z) + "W: " + std::to_string(HMDRotation.w));*/

    
    // Save stored pose
    if (pose_is_complete) {

        linalg::mat<float, 4, 4> HeadNewWorldTransform;

        // Build local space transforms for segments relative to the head
        for (auto tracker_pair : trackers_) {
            auto& segment = incomplete_poses_[msg_id].segments[tracker_pair.second->GetSegmentIndex()];

            if (tracker_pair.first == Segment::Head) {
                // For reference, the following is reference for getting back to the correct pose but wrong offset 
                // HeadNewWorldTransform = segment.worldTransform;
                HeadNewWorldTransform = linalg::mul(HMDHeadLocalOffset, segment.worldTransform);
                //HeadNewWorldTransform = segment.worldTransform;//HMDHeadLocalOffset; // linalg::mul(HMDHeadLocalOffset, HMDTransform);
                //HeadNewWorldTransform.x.w -= HMDLocationOffset.x;
                //HeadNewWorldTransform.y.w -= HMDLocationOffset.y;
                //HeadNewWorldTransform.z.w -= HMDLocationOffset.z;
                //segment.worldTransform = segment.worldTransform;// HeadNewWorldTransform;
                segment.localTransform = linalg::mat<float, 4, 4>();
            }
            else {
                // Segment worldspace matrix to head-relative matrix
                auto localTransform = linalg::mul(segment.worldTransform, linalg::inverse(HeadOrigWorldTransform));
                segment.localTransform = localTransform;
            }
        }


        for (auto tracker_pair : trackers_) {
            auto& segment = incomplete_poses_[msg_id].segments.at(tracker_pair.second->GetSegmentIndex());

            // Head-relative local to world
            // The following line is for reference for how to convert from head relative to world space
            // linalg::mul(segment.localTransform, HeadOrigWorldTransform)

            linalg::mat<float, 4, 4> SegmentWorldTransform;
            if (tracker_pair.first == Segment::Head) {
                SegmentWorldTransform = HeadNewWorldTransform;//segment.worldTransform;
            }
            else {
                SegmentWorldTransform = linalg::mul(segment.localTransform, HeadNewWorldTransform);
            }

            auto rotation_quat = linalg::rotation_quat(GetRotationMatrixFromTransform(SegmentWorldTransform));
            auto location = GetTranslationFromTransformMatrix(SegmentWorldTransform);

            segment.rotation_quat[0] = rotation_quat.w;
            segment.rotation_quat[1] = rotation_quat.x;
            segment.rotation_quat[2] = rotation_quat.y;
            segment.rotation_quat[3] = rotation_quat.z;
            segment.translation[0] = location.x;
            segment.translation[1] = location.y;
            segment.translation[2] = location.z;

            /*segment.translation[0] -= HMDLocationOffset.x;
            segment.translation[1] -= HMDLocationOffset.y;
            segment.translation[2] -= HMDLocationOffset.z;*/
        }
    }

    QueuePose(incomplete_poses_[msg_id]);
    incomplete_poses_.erase(msg_id);
}