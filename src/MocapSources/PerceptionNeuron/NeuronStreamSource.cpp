#include <sstream>
#include <IVRDriver.hpp>
#include <PoseMath.hpp>
#include <filesystem>
#include "NeuronStreamSource.h"

#define _USE_MATH_DEFINES
#include <math.h>
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
	// If a connection has already been established
	if (sockUDPRef) {
		// close socket
		BRCloseSocket(sockUDPRef);
		sockUDPRef = 0;
	}

	// Try to connect to the Neuron
	const char* address = "127.0.0.1";
	//sockTCPRef = BRConnectTo(const_cast<char*>(address), 7003);
	sockUDPRef = BRStartUDPServiceAt(7004);

	// If the connection is ok
	if (sockUDPRef) {

		//calback register used to receive data
		// (we assign BvhFrameDataReceived to the FrameData callback,
		//  and CalcFrameDataReceive to the CalculationData callback)
		BRRegisterFrameDataCallback(this, BvhFrameDataReceived);

		// Unused
		// TODO: check if it works if we delete this
		BRRegisterCalculationDataCallback(this, CalcFrameDataReceive);

		return true;
	}

	// Failed
	return false;
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

void NeuronStreamSource::KillConnection() {
	BRCloseSocket(sockUDPRef);
}

void __stdcall NeuronStreamSource::BvhFrameDataReceived(void* customedObj, SOCKET_REF sender, BvhDataHeader* header, float* data) {
	NeuronStreamSource* pthis = (NeuronStreamSource*)customedObj;
	pthis->ShowBvhBoneInfo(sender, header, data);
}

// Called, but unused
void __stdcall NeuronStreamSource::CalcFrameDataReceive(void* customedObj, SOCKET_REF sender, CalcDataHeader* header, float* data) {
	NeuronStreamSource* pthis = (NeuronStreamSource*)customedObj;
	pthis->ShowBvhCalcInfo(sender, header, data);
}

void NeuronStreamSource::ShowBvhBoneInfo(SOCKET_REF sender, BvhDataHeader* header, float* data) {
	if (m_initialFrame == -1)
		m_initialFrame = header->FrameIndex;
	
	if(m_dataCount == -1)
		m_dataCount = header->DataCount;

	BvhExport(header, data);
}

void NeuronStreamSource::ShowBvhCalcInfo(SOCKET_REF sender, CalcDataHeader* header, float* data) {
	// Cool callback bro
}

void NeuronStreamSource::BvhExport(BvhDataHeader* header, float* data) {

	// Initial connection :
	// We get the initial offsets and the hierarchy
	m_perJoint = 6;

	if (!header->WithDisp) {
		GetDriver()->Log("Received BVH frame from Axis Studio with no displacement data. Enable displacement in Axis studio BVH broadcast.");
		return;
	}

	m_currentFrame = header->FrameIndex;

	PoseSample pose = PoseSample{ 0, std::vector<SegmentSample>(NeuronSegmentName.size()) };
	std::vector<linalg::vec<float, 3>> relativeLocations(NeuronSegmentName.size());
	std::vector<linalg::vec<float, 4>> relativeRotations(NeuronSegmentName.size());

	for (unsigned int i = 0; i < NeuronSegmentName.size(); i++) {
		// Get locations and convert to meters
		float xloc = data[(i * 6) + 0] / 100.0;
		float yloc = data[(i * 6) + 1] / 100.0;
		float zloc = data[(i * 6) + 2] / 100.0;

		// Get rotations and convert to radians
		float yrot = data[(i * 6) + 3] * M_PI / 180; //yaw
		float xrot = data[(i * 6) + 4] * M_PI / 180; //pitch
		float zrot = data[(i * 6) + 5] * M_PI / 180; //roll

		//// Min euler
		//yrot = (yrot < 0) ? yrot + 360.0 : yrot;
		//xrot = (xrot < 0) ? xrot + 360.0 : xrot;
		//zrot = (zrot < 0) ? zrot + 360.0 : zrot;

		//// Max euler
		//yrot = (yrot >= 360.0) ? yrot - 360.0 : yrot;
		//xrot = (xrot >= 360.0) ? xrot - 360.0 : xrot;
		//zrot = (zrot >= 360.0) ? zrot - 360.0 : zrot;

		// Store relative locations for conversion to absolute locations


		// Convert rotations to quaternions
		linalg::vec<float, 3> pos = {xloc, yloc, zloc};
		linalg::vec<float, 3> euler = { yrot, xrot, zrot };
		linalg::vec<float, 4> quat = eulerToQuaternion(euler);
		quat = linalg::normalize(quat);

		//linalg::mat<float, 4, 4> trans_mat = linalg::pose_matrix<float>(quat, pos);
		//trans_mat = ConvertLeftToRightHanded(trans_mat);
		//linalg::vec<float, 4> rotQuat = linalg::rotation_quat(GetRotationMatrixFromTransform(trans_mat));

		// Create pose
		relativeLocations[i][0] = pos.x;
		relativeLocations[i][1] = pos.y;
		relativeLocations[i][2] = pos.z;
		relativeRotations[i][0] = quat.w;
		relativeRotations[i][1] = quat.x;
		relativeRotations[i][2] = quat.y;
		relativeRotations[i][3] = quat.z;
	}

	// We need to convert relative joint positions to absolute so use the cached BVH hierarchy
	for (unsigned int joint_idx = 0; joint_idx < NeuronSegmentName.size(); joint_idx++) {
		auto jointChain = bvhLoader.GetBoneChainToTarget(joint_idx);
		
		// Sum relative joint locations to get the absolute translation
		auto start_joint = *jointChain.begin();
		linalg::mat<float, 4, 4> abs_joint_pose = linalg::pose_matrix<float>(relativeRotations[start_joint->index], relativeLocations[start_joint->index]);
		for(auto joint : jointChain){
			if (joint != *jointChain.begin()) {
				// Multiply child against each parent joint to build absolute position
				linalg::vec<float, 3> offset_pos = relativeLocations[joint->index];//{ joint->offset.x / 100.0f, joint->offset.y / 100.0f, joint->offset.z / 100.0f };
				abs_joint_pose = linalg::mul(abs_joint_pose, linalg::pose_matrix<float>(relativeRotations[joint->index], offset_pos));
			}
		}

		// Pull quaternion out of segment transformation matrix
		linalg::vec<float, 4> rotQuat = linalg::rotation_quat(GetRotationMatrixFromTransform(abs_joint_pose));

		pose.segments[joint_idx].translation[0] = abs_joint_pose.w.x;
		pose.segments[joint_idx].translation[1] = abs_joint_pose.w.y;
		pose.segments[joint_idx].translation[2] = abs_joint_pose.w.z;
		pose.segments[joint_idx].rotation_quat[0] = rotQuat.w;
		pose.segments[joint_idx].rotation_quat[1] = rotQuat.x;
		pose.segments[joint_idx].rotation_quat[2] = rotQuat.y;
		pose.segments[joint_idx].rotation_quat[3] = rotQuat.z;
	}

	QueuePose(pose);
}

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
