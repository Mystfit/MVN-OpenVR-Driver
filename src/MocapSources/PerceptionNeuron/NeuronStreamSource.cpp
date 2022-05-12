#include <sstream>
#include <IVRDriver.hpp>
#include <PoseMath.hpp>
#include <filesystem>
#include "NeuronStreamSource.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <cassert>
#undef _USE_MATH_DEFINES

//NeuronStreamSource::NeuronStreamSource() {
//	m_firstWrite = true;
//	m_initialFrame = -1;
//	m_dataCount = -1;
//}
//
//
//NeuronStreamSource::~NeuronStreamSource() {
//
//}

void NeuronStreamSource::Init(MocapDriver::IVRDriver* owning_driver)
{
	owning_driver_ = owning_driver;

	vr::ETrackedPropertyError tpeError;
	std::filesystem::path installDir = vr::VRProperties()->GetStringProperty(vr::VRDriverContext()->GetDriverHandle(), vr::Prop_InstallPath_String, &tpeError);

	//(installDir / std::filesystem::path("resources/skeletons/axisneuron_hierarchy.bvh")).string()

	//char hierarchy_path[vr::k_unMaxPropertyStringSize];
	//vr::VRSettings()->GetResourceFullPath("axisneuron_hierarchy.bvh", "{Mocap}/skeletons", hierarchy_path, vr::k_unMaxPropertyStringSize);
	bvhLoader.load_file((installDir / "resources" / "skeletons" / "axisneuron_hierarchy.bvh"));
}

bool NeuronStreamSource::Connect()
{
	// Query Application Interface [1/8/2021 brian.wang]
	MocapApi::IMCPApplication* mcpApplication = nullptr;
	MocapApi::EMCPError mcpError = MocapApi::MCPGetGenericInterface(MocapApi::IMCPApplication_Version,
		reinterpret_cast<void**>(&mcpApplication));
	//check(mcpError == MocapApi::Error_None);

	// Create Communication Application With PNS [1/8/2021 brian.wang]
	mcpError = mcpApplication->CreateApplication(&_application);
	assert(mcpError == MocapApi::EMCPError::Error_None);

	{
		// Query Settings Interface [1/8/2021 brian.wang]
		MocapApi::IMCPSettings* mcpSettings = nullptr;
		mcpError = MocapApi::MCPGetGenericInterface(MocapApi::IMCPSettings_Version,
			reinterpret_cast<void**>(&mcpSettings));
		assert(mcpError == MocapApi::EMCPError::Error_None);

		// Create Settings Object [1/8/2021 brian.wang]
		MocapApi::MCPSettingsHandle_t mcpSettingsHandle = 0;
		mcpError = mcpSettings->CreateSettings(&mcpSettingsHandle);
		assert(mcpError == MocapApi::EMCPError::Error_None);

		// Follow settings value setting is same sa PNS Settings [1/8/2021 brian.wang]
		{
			/*
			* [1/8/2021 brian.wang]
			*   This settings same as "Bvh Broadcasting"-> "Frame Format" -> "Type", only support MocapApi::BvhDataType_Binary,
				other MocapApi::BvhDataType_Mask_LegacyHumanHierarchy with MocapApi::BvhDataType_Binary Group same as "Skeleton"->"Perception Neuron/ Perception Neuron Pro" in PNS.
			*/
			mcpError = mcpSettings->SetSettingsBvhData(MocapApi::BvhDataType_Binary, mcpSettingsHandle);
			assert(mcpError == MocapApi::EMCPError::Error_None);
			// this settings same as "BVH Broadcasting"->"Bvh Format"->Rotation in pns [1/8/2021 brian.wang]
			mcpError = mcpSettings->SetSettingsBvhRotation(MocapApi::BvhRotation_YXZ, mcpSettingsHandle);
			assert(mcpError == MocapApi::EMCPError::Error_None);
			// this settings same as pns [1/8/2021 brian.wang]
			mcpError = mcpSettings->SetSettingsBvhTransformation(MocapApi::BvhTransformation_Enable, mcpSettingsHandle);
			assert(mcpError == MocapApi::EMCPError::Error_None);
#if 1   //  same as "Bvh Broadcasting" -> "Protocol" -> "UDP" -> "Port" in PNS Setting
			mcpError = mcpSettings->SetSettingsUDP(7004, mcpSettingsHandle);
#else   //  same as "Bvh Broadcasting" -> "Protocol" -> "TCP" -> "Local IP" -> "IP" and "Port" in PNS Setting
			mcpError = mcpSettings->SetSettingsTCP("127.0.0.1", 7002, mcpSettingsHandle);
#endif
			assert(mcpError == MocapApi::EMCPError::Error_None);
		}

		// set Settings for Application [1/8/2021 brian.wang]
		mcpError = mcpApplication->SetApplicationSettings(mcpSettingsHandle, _application);
		assert(mcpError == MocapApi::EMCPError::Error_None);
		// Destroy Settings, if not used [1/8/2021 brian.wang]
		//mcpError = mcpSettings->DestroySettings(mcpSettingsHandle);
		//assert(mcpError == MocapApi::EMCPError::Error_None);
	}

	{
		// Query RenderSettings Interface [1/8/2021 brian.wang]
		MocapApi::IMCPRenderSettings* renderSettings = nullptr;
		mcpError = MocapApi::MCPGetGenericInterface(MocapApi::IMCPRenderSettings_Version,
			reinterpret_cast<void**>(&renderSettings));
		assert(mcpError == MocapApi::EMCPError::Error_None);

		MocapApi::MCPRenderSettingsHandle_t renderSettingsHandle = 0;

		// Get Pre-Defined RenderSetting suport PreDefinedRenderSettings_UnrealEngine, or PreDefinedRenderSettings_Unity3D [1/8/2021 brian.wang]
		renderSettings->GetPreDefRenderSettings(MocapApi::PreDefinedRenderSettings_Default, &renderSettingsHandle);// MUST NOT Destroy this [1/8/2021 brian.wang]

		// Set Render Settings for Application [1/8/2021 brian.wang]
		mcpError = mcpApplication->SetApplicationRenderSettings(renderSettingsHandle, _application);
		assert(mcpError == MocapApi::EMCPError::Error_None);
	}

	mcpError = mcpApplication->OpenApplication(_application);
	assert(mcpError == MocapApi::EMCPError::Error_None);

	// Failed
	return true;
}

void NeuronStreamSource::Update()
{
	// Query Application Interface [1/8/2021 brian.wang]
	MocapApi::IMCPApplication* mcpApplication = nullptr;
	MocapApi::EMCPError mcpError = MocapApi::MCPGetGenericInterface(MocapApi::IMCPApplication_Version,
		reinterpret_cast<void**>(&mcpApplication));

	std::vector<MocapApi::MCPEvent_t> events;
	uint32_t unEvent = 0;

	// poll next event in application event queue [1/8/2021 brian.wang]
	mcpError = mcpApplication->PollApplicationNextEvent(nullptr, &unEvent, _application);
	if (unEvent > 0) {
		events.resize(unEvent);
		for (auto& e : events) {
			e.size = sizeof(MocapApi::MCPEvent_t);
		}
		mcpError = mcpApplication->PollApplicationNextEvent(events.data(), &unEvent, _application);
	}
	if (unEvent > 0) {
		// update avatar posture [1/8/2021 brian.wang]
		for (const auto& e : events) {
			if (e.eventType == MocapApi::MCPEvent_AvatarUpdated) {
				UpdateAvatarTransform(e.eventData.motionData.avatarHandle);
			}
		}
	}
}

void NeuronStreamSource::PopulateTrackers()
{
	for (auto segment : NeuronSegmentName) {
		std::string segment_hint = GetSegmentRole(segment.first);
		if (segment_hint.compare("disabled") && !segment_hint.empty()) {
			std::string name = NeuronSegmentName.at(segment.first);
			std::string role = segment_hint;

			auto tracker = GetDriver()->CreateTrackerDevice(name, role, this, (int)segment.first);
			trackers_.emplace(segment.first, tracker);
		}
	}
}

MocapDriver::IVRDriver* NeuronStreamSource::GetDriver()
{
	return owning_driver_;
}

PoseSample NeuronStreamSource::GetNextPose()
{
	return completed_pose_;
}

void NeuronStreamSource::QueuePose(const PoseSample& pose)
{
	std::scoped_lock<std::mutex> lock(pose_update_mtx);
	completed_pose_ = pose;
}

std::string NeuronStreamSource::GetRenderModelPath(int segmentIndex)
{
	//TODO

	// Relative to "{Mocap}/rendermodels"
// 
	//return std::string("Neuron/") + SegmentName.at((MVNSegment)segmentIndex); //"{htc}/rendermodels/vr_tracker_vive_1_0"; 
	//return std::string("PerceptionNeuron/locator");
	return std::string("PerceptionNeuron/generic_tracker");
}


void NeuronStreamSource::UpdateAvatarTransform(MocapApi::MCPAvatarHandle_t avatar)
{
	MocapApi::EMCPError mcpError;

	// Query Avatar Interface [1/8/2021 brian.wang]
	MocapApi::IMCPAvatar* avatarMgr = nullptr;
	mcpError = MocapApi::MCPGetGenericInterface(MocapApi::IMCPAvatar_Version,
		reinterpret_cast<void**>(&avatarMgr));
	assert(mcpError == MocapApi::EMCPError::Error_None);

	// Get Root Joint at Avatar [1/8/2021 brian.wang]
	MocapApi::MCPJointHandle_t joint = 0;
	mcpError = avatarMgr->GetAvatarRootJoint(&joint, avatar);
	assert(mcpError == MocapApi::EMCPError::Error_None);

	MocapApi::MCPJointHandle_t joints;
	uint32_t numJoints = 0;
	mcpError = avatarMgr->GetAvatarJoints(&joints, &numJoints, avatar);
	assert(mcpError == MocapApi::EMCPError::Error_None);

	PoseSample pose = PoseSample{ 0, std::vector<SegmentSample>(MocapApi::EMCPJointTag::JointTag_JointsCount) };

	// Update joint posture recursived [1/8/2021 brian.wang]
	UpdateJointsTransform(joint, pose);

	QueuePose(pose);
}

void NeuronStreamSource::UpdateJointsTransform(MocapApi::MCPJointHandle_t joint, PoseSample& outPose)
{
	MocapApi::EMCPError mcpError;

	// Get Joint Interface [1/8/2021 brian.wang]
	MocapApi::IMCPJoint* jointMgr = nullptr;
	mcpError = MocapApi::MCPGetGenericInterface(MocapApi::IMCPJoint_Version,
		reinterpret_cast<void**>(&jointMgr));
	assert(mcpError == MocapApi::EMCPError::Error_None);

	// Get Joint Name [1/8/2021 brian.wang]
	const char* name = nullptr;
	jointMgr->GetJointName(&name, joint);
	assert(mcpError == MocapApi::EMCPError::Error_None);

	MocapApi::EMCPJointTag jointTag = MocapApi::EMCPJointTag::JointTag_Invalid;
	mcpError = jointMgr->GetJointTag(&jointTag, joint);
	assert(mcpError == MocapApi::EMCPError::Error_None);
	int jointIdx = (int)jointTag;

	MocapApi::IMCPBodyPart* bodyPartMgr = nullptr;
	mcpError = MocapApi::MCPGetGenericInterface(MocapApi::IMCPBodyPart_Version,
		reinterpret_cast<void**>(&bodyPartMgr));
	assert(mcpError == MocapApi::EMCPError::Error_None);

	MocapApi::MCPBodyPartHandle_t bodyPart;
	mcpError = jointMgr->GetJointBodyPart(&bodyPart, joint);
	assert(mcpError == MocapApi::EMCPError::Error_None);

	float xloc, yloc, zloc;
	linalg::vec<float, 3> loc;
	linalg::vec<float, 4> rot;

	mcpError = jointMgr->GetJointLocalPosition(&loc.x, &loc.y, &loc.z, joint);
	mcpError = jointMgr->GetJointLocalRotation(&rot.y, &rot.x , &rot.z, &rot.w, joint);
	//mcpError = bodyPartMgr->GetJointPosition(&x, &y, &z, bodyPart);
	assert(mcpError == MocapApi::EMCPError::Error_None);


	linalg::mat<float, 4, 4> worldPose = linalg::pose_matrix<float>(rot, loc / 100.0f);
	linalg::vec<float, 4> quat = linalg::rotation_quat(GetRotationMatrixFromTransform(worldPose));

	outPose.segments[jointIdx].translation[0] = worldPose.w.x;
	outPose.segments[jointIdx].translation[1] = worldPose.w.y;
	outPose.segments[jointIdx].translation[2] = worldPose.w.z;
	outPose.segments[jointIdx].rotation_quat[0] = quat.w;
	outPose.segments[jointIdx].rotation_quat[1] = quat.y;
	outPose.segments[jointIdx].rotation_quat[2] = quat.y;
	outPose.segments[jointIdx].rotation_quat[3] = quat.z;

	//jointMgr->GetJointLocalPosition(&v.X, &v.Y, &v.Z, joint)
	//jointMgr->GetJointLocalRotation(&q.X, &q.Y, &q.Z, &q.W, joint);
	
	uint32_t numberOfChildren = 0;
	jointMgr->GetJointChild(nullptr, &numberOfChildren, joint);
	if (numberOfChildren > 0) {
		std::vector<MocapApi::MCPJointHandle_t> joints;
		joints.resize(numberOfChildren);
		jointMgr->GetJointChild(&joints[0], &numberOfChildren, joint);

		// Update joint posture recursived [1/8/2021 brian.wang]
		for (auto j : joints) {
			UpdateJointsTransform(j, outPose);
		}
	}
}


//void NeuronStreamSource::BvhExport(BvhDataHeader* header, float* data) {
//
//	// Initial connection :
//	// We get the initial offsets and the hierarchy
//	m_perJoint = 6;
//
//	if (!header->WithDisp) {
//		GetDriver()->Log("Received BVH frame from Axis Studio with no displacement data. Enable displacement in Axis studio BVH broadcast.");
//		return;
//	}
//
//	m_currentFrame = header->FrameIndex;
//
//	PoseSample pose = PoseSample{ 0, std::vector<SegmentSample>(NeuronSegmentName.size()) };
//	std::vector<linalg::vec<float, 3>> localLocations(NeuronSegmentName.size());
//	std::vector<linalg::vec<float, 4>> localRotations(NeuronSegmentName.size());
//
//	// Only need to calculate each worldspace joint once
//	std::unordered_map<int, linalg::mat<float, 4, 4>> worldspace_joint_poses = { {0, linalg::pose_matrix<float>(localRotations[0], localLocations[0])} };
//
//	for (unsigned int i = 0; i < NeuronSegmentName.size(); ++i) {
//		// Get locations and convert to meters
//		float xloc = data[(i * 6) + 0] / 100.0;
//		float yloc = data[(i * 6) + 1] / 100.0;
//		float zloc = data[(i * 6) + 2] / 100.0;
//
//		// Get rotations and convert to radians
//		float yrot = data[(i * 6) + 3] * M_PI / 180; //yaw
//		float xrot = data[(i * 6) + 4] * M_PI / 180; //pitch
//		float zrot = data[(i * 6) + 5] * M_PI / 180; //roll
//
//		// Convert rotations to quaternions
//		linalg::vec<float, 3> localpos = {xloc, yloc, zloc};
//		linalg::vec<float, 4> localrot = linalg::normalize(eulerToQuaternion(linalg::vec<float, 3>{ yrot, xrot, zrot }));
//		linalg::mat<float, 4, 4> localPose = linalg::pose_matrix<float>(localrot, localpos);
//
//		if (i == 0) {
//			// Hips
//			//GetDriver()->Log("Saving worldspace pose for joint " + NeuronSegmentName.at((NeuronSegment)i));
//			worldspace_joint_poses[i] = linalg::pose_matrix<float>(localrot, localpos);
//		}
//
//		auto jointChain = bvhLoader.GetBoneChainToTarget(i);
//		auto start_joint = *jointChain.begin();
//		for (auto joint : jointChain) {
//			if (joint->parent) {
//				if (worldspace_joint_poses.find(joint->index) == worldspace_joint_poses.end()) {
//					auto parent_world_pose = worldspace_joint_poses.find(joint->parent->index);
//					//GetDriver()->Log("Saving worldspace pose for joint " + NeuronSegmentName.at((NeuronSegment)i));
//					worldspace_joint_poses[joint->index] = linalg::mul(parent_world_pose->second, localPose);
//				}
//			}
//		}
//
//		linalg::vec<float, 4> rotQuat = linalg::rotation_quat(GetRotationMatrixFromTransform(worldspace_joint_poses[i]));
//
//		pose.segments[i].translation[0] = worldspace_joint_poses[i].w.x;
//		pose.segments[i].translation[1] = worldspace_joint_poses[i].w.y;
//		pose.segments[i].translation[2] = worldspace_joint_poses[i].w.z;
//		pose.segments[i].rotation_quat[0] = rotQuat.w;
//		pose.segments[i].rotation_quat[1] = rotQuat.x;
//		pose.segments[i].rotation_quat[2] = rotQuat.y;
//		pose.segments[i].rotation_quat[3] = rotQuat.z;
//	}
//
//	QueuePose(pose);
//}

std::string NeuronStreamSource::GetSegmentRole(NeuronSegment segment)
{
	std::string prefix = "Role_";
	std::string key = prefix + NeuronSegmentName.at(segment);

	vr::EVRSettingsError err = vr::EVRSettingsError::VRSettingsError_None;
	char* buf = (char*)malloc(sizeof(char) * 1024);
	vr::VRSettings()->GetString("AxisNeuron", key.c_str(), buf, 1024, &err);
	std::string str_value(buf);
	free(buf);

	if (err == vr::EVRSettingsError::VRSettingsError_None) {
		return str_value;
	}

	return "";
}
