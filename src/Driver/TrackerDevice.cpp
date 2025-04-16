#include "TrackerDevice.hpp"
#include <PoseMath.hpp>

using namespace MocapDriver;

TrackerDevice::TrackerDevice(std::string serial, std::string role):
    serial_(serial),
    role_(role),
    motionSource_(nullptr),
    rotation_origin(),
    segmentIndex_(-1)
{
    this->last_pose_ = MakeDefaultPose();
    this->isSetup = false;

    // Create thread to handle pose updates
    //pose_submit_thread = std::thread([this]() {this->SubmitPose(); });
}

std::string TrackerDevice::GetSerial()
{
    return this->serial_;
}

void TrackerDevice::reinit(int msaved, double mtime, double msmooth, UniverseOrigin origin)
{
    if (msaved < 5)     //prevent having too few values to calculate linear interpolation, and prevent crash on 0
        msaved = 5;

    if (msmooth < 0)
        msmooth = 0;
    else if (msmooth > 0.99)
        msmooth = 0.99;

    max_saved = msaved;
    std::vector<std::vector<double>> temp(msaved, std::vector<double>(8,-1));
    prev_positions = temp;
    max_time = mtime;
    smoothing = msmooth;

    translation_origin[0] = origin.translation[0];
    translation_origin[1] = origin.translation[1];
    translation_origin[2] = origin.translation[2];

    double euler_rot_origin[3] = { 0.0, origin.yaw, 0.0 };
    double quat_rot_origin[4] = {};
    eulerToQuaternion(euler_rot_origin, quat_rot_origin);
    rotation_origin.w = quat_rot_origin[3];
    rotation_origin.x = quat_rot_origin[0];
    rotation_origin.y = quat_rot_origin[1];
    rotation_origin.z = quat_rot_origin[2];

    //Log("Settings changed! " + std::to_string(msaved) + " " + std::to_string(mtime));
}

void MocapDriver::TrackerDevice::SetMotionSource(IMocapStreamSource* motionSource)
{
    motionSource_ = motionSource;
}

void MocapDriver::TrackerDevice::SetSegmentIndex(int segmentIndex)
{
    segmentIndex_ = segmentIndex;
}

int MocapDriver::TrackerDevice::GetSegmentIndex()
{
    return segmentIndex_;
}

IMocapStreamSource* MocapDriver::TrackerDevice::GetMotionSource()
{
    return motionSource_;
}

void MocapDriver::TrackerDevice::SubmitPose()
{
    //while (true) {
        auto source = GetMotionSource();
        if (source) 
        {
            // Block until pose is ready
            PoseSample pose = source->GetNextPose();
            //source->WaitForPose(pose);

            // Setup pose for this frame
            vr::DriverPose_t tracker_pose = MakeDefaultPose();

            // Update time delta (for working out velocity)
            std::chrono::high_resolution_clock::time_point  current_time;
            double real_pose_delta_seconds = 0.0;

            // Get segment
            auto segmentIndex = GetSegmentIndex();
            if (segmentIndex < 0 || !pose.segments.size()) {
                return;
            }

            tracker_pose.vecPosition[0] = pose.segments[segmentIndex].translation[0];
            tracker_pose.vecPosition[1] = pose.segments[segmentIndex].translation[1];
            tracker_pose.vecPosition[2] = pose.segments[segmentIndex].translation[2];

            tracker_pose.qRotation.w = pose.segments[segmentIndex].rotation_quat[0];
            tracker_pose.qRotation.x = pose.segments[segmentIndex].rotation_quat[1];
            tracker_pose.qRotation.y = pose.segments[segmentIndex].rotation_quat[2];
            tracker_pose.qRotation.z = pose.segments[segmentIndex].rotation_quat[3];


            // Set world origin from universe standing position
            tracker_pose.vecWorldFromDriverTranslation[0] = translation_origin[0];
            tracker_pose.vecWorldFromDriverTranslation[1] = translation_origin[1];
            tracker_pose.vecWorldFromDriverTranslation[2] = translation_origin[2];

            // Set world rotation from universe yaw 
            tracker_pose.qWorldFromDriverRotation = rotation_origin;

            // Set velocity
            //tracker_pose.vecVelocity[0] = pose.segments[segmentIndex].velocity[0];
            //tracker_pose.vecVelocity[1] = pose.segments[segmentIndex].velocity[1];
            //tracker_pose.vecVelocity[2] = pose.segments[segmentIndex].velocity[2];
            /* tracker_pose.vecAngularVelocity[0] = pose.segments[segmentIndex].angular_velocity[0];
            tracker_pose.vecAngularVelocity[1] = pose.segments[segmentIndex].angular_velocity[1];
            tracker_pose.vecAngularVelocity[2] = pose.segments[segmentIndex].angular_velocity[2];*/

            // Set pose time offset
            current_time = std::chrono::high_resolution_clock::now();
            real_pose_delta_seconds = std::chrono::duration<double>(current_time - pose.timestamp).count();

            if (real_pose_delta_seconds > 0)            //unless we get two pose updates at the same time, update velocity so steamvr can do some interpolation
            {
                // Update velocity using last submitted pose as a reference
                /*tracker_pose.vecVelocity[0] = (tracker_pose.vecPosition[0] - this->last_pose_.vecPosition[0]) / pose_time_delta_seconds;
                tracker_pose.vecVelocity[1] = (tracker_pose.vecPosition[1] - this->last_pose_.vecPosition[1]) / pose_time_delta_seconds;
                tracker_pose.vecVelocity[2] = (tracker_pose.vecPosition[2] - this->last_pose_.vecPosition[2]) / pose_time_delta_seconds;*/
                /*tracker_pose.vecVelocity[0] = 0.8 * tracker_pose.vecVelocity[0] + 0.2 * (tracker_pose.vecPosition[0] - previous_position[0]) / pose_time_delta_seconds;
                tracker_pose.vecVelocity[1] = 0.8 * tracker_pose.vecVelocity[1] + 0.2 * (tracker_pose.vecPosition[1] - previous_position[1]) / pose_time_delta_seconds;
                tracker_pose.vecVelocity[2] = 0.8 * tracker_pose.vecVelocity[2] + 0.2 * (tracker_pose.vecPosition[2] - previous_position[2]) / pose_time_delta_seconds;*/
            }

            tracker_pose.poseTimeOffset = real_pose_delta_seconds;
            //Log("Segment: " + std::to_string(segmentIndex) + " Pose time offset : " + std::to_string(tracker_pose.poseTimeOffset));

            // Post pose
            GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, tracker_pose, sizeof(vr::DriverPose_t));
        }
    //}
}

void TrackerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Check if this device was asked to be identified
    auto events = GetDriver()->GetOpenVREvents();
    for (auto event : events) {
        // Note here, event.trackedDeviceIndex does not necissarily equal this->device_index_, not sure why, but the component handle will match so we can just use that instead
        //if (event.trackedDeviceIndex == this->device_index_) {
        if (event.eventType == vr::EVREventType::VREvent_Input_HapticVibration) {
            if (event.data.hapticVibration.componentHandle == this->haptic_component_) {
                this->did_vibrate_ = true;
            }
        }
        //}
    }

    // Check if we need to keep vibrating
    if (this->did_vibrate_) {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count() / 1000.f);
        if (this->vibrate_anim_state_ > 1.0f) {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }
}

void TrackerDevice::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

DeviceType TrackerDevice::GetDeviceType()
{
    return DeviceType::TRACKER;
}

vr::TrackedDeviceIndex_t TrackerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

vr::EVRInitError TrackerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating tracker " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 3);
    
    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "Mocap_tracker");

    // Opt out of hand selection
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);

    // Set up a render model path
    std::string rendermodel = "{Mocap}/rendermodels/" + motionSource_->GetRenderModelPath(GetSegmentIndex());
    Log("Mocap segment rendermodel path: " + rendermodel);
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, rendermodel.c_str());

    // Set controller profile
    //GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{Mocap}/input/example_tracker_bindings.json");

    // Set the icon
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, "{Mocap}/icons/tracker_ready.png");

    if (this->serial_.find("Mocap") == std::string::npos)
    {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{Mocap}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{Mocap}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{Mocap}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{Mocap}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{Mocap}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{Mocap}/icons/tracker_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{Mocap}/icons/tracker_not_ready.png");
    }
    else
    {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{Mocap}/icons/MVN_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{Mocap}/icons/MVN_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{Mocap}/icons/MVN_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{Mocap}/icons/MVN_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{Mocap}/icons/MVN_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{Mocap}/icons/MVN_not_ready.png");
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{Mocap}/icons/MVN_not_ready.png");
    }

    //set role, role hint and everything else to ensure trackers are detected as trackers and not controllers
    std::string rolehint = "vive_tracker";
    if (role_ == "TrackerRole_LeftFoot")
        rolehint = "vive_tracker_left_foot";
    else if (role_ == "TrackerRole_RightFoot")
        rolehint = "vive_tracker_right_foot";
    else if (role_ == "TrackerRole_Waist")
        rolehint = "vive_tracker_waist";

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, role_.c_str());

    vr::VRProperties()->SetInt32Property(props, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
    vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerHandSelectionPriority_Int32, -1);

    std::string l_registeredDevice("/devices/Mocap/");
    l_registeredDevice.append(serial_);

    Log("Setting role " + role_ + " to " + l_registeredDevice);

    if(role_ != "vive_tracker")
        vr::VRSettings()->SetString(vr::k_pch_Trackers_Section, l_registeredDevice.c_str(), role_.c_str());

    return vr::EVRInitError::VRInitError_None;
}

void TrackerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void TrackerDevice::EnterStandby()
{
}

void* TrackerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void TrackerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t TrackerDevice::GetPose()
{
    return last_pose_;
}
