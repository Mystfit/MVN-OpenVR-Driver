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
	bvhLoader.load_file("F:/Code/MVN-OpenVR-Driver/src/MocapSources/PerceptionNeuron/bvh_header_template2.bvh");
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

	for (unsigned int i = 0; i < NeuronSegmentName.size(); i++) {
		// Get locations and convert to meters
		float xloc = data[(i * 6) + 0] / 100.0;
		float yloc = data[(i * 6) + 1] / 100.0;
		float zloc = data[(i * 6) + 2] / 100.0;

		// Get rotations and convert to radians
		float yrot = data[(i * 6) + 3] * M_PI / 180;
		float xrot = data[(i * 6) + 4] * M_PI / 180;
		float zrot = data[(i * 6) + 5] * M_PI / 180;

		// Convert rotations to quaternions
		double euler[3] = { xrot, yrot, zrot };
		double quat[4];
		eulerToQuaternion(euler, quat);

		// Create pose
		pose.segments[i].translation[0] = xloc;
		pose.segments[i].translation[1] = yloc;
		pose.segments[i].translation[2] = zloc;
		pose.segments[i].rotation_quat[0] = quat[0];
		pose.segments[i].rotation_quat[1] = quat[1];
		pose.segments[i].rotation_quat[2] = quat[2];
		pose.segments[i].rotation_quat[3] = quat[3];

		try {
			/*if ((NeuronSegment)i == NeuronSegment::Hips) {
				GetDriver()->Log(
					"Neuron segment: " + NeuronSegmentName.at((NeuronSegment)i) +
					" Xpos: " + std::to_string(xloc) +
					" Ypos: " + std::to_string(yloc) +
					" Zpos: " + std::to_string(zloc) +
					" Yrot: " + std::to_string(yrot) +
					" Xrot: " + std::to_string(xrot) +
					" Zrot: " + std::to_string(zrot)
				);
			}*/
		}
		catch (std::out_of_range e) {
			std::string segmentIdx = std::to_string(i);
			GetDriver()->Log("Neuron segment index out of range" + segmentIdx);
		}
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
