#pragma once

#include <unordered_map>
#include <string>

enum class MVNSegment {
	Pelvis = 0,
	L5= 1,
	L3 = 2,
	T12 = 3,
	T8 = 4,
	Neck = 5,
	Head = 6,
	RightShoulder = 7,
	RightUpperArm = 8,
	RightForearm = 9,
	RightHand = 10,
	LeftShoulder = 11,
	LeftUpperArm = 12,
	LeftForearm = 13,
	LeftHand = 14,
	RightUpperLeg = 15,
	RightLowerLeg = 16,
	RightFoot = 17,
	RightToe = 18,
	LeftUpperLeg = 19,
	LeftLowerLeg = 20,
	LeftFoot = 21,
	LeftToe = 22
	//Prop1 = 24,
	//Prop2 = 25,
	//Prop3 = 26,
	//Prop4 = 27
};

const std::unordered_map<MVNSegment, std::string> MVNSegmentName = {
	{MVNSegment::Pelvis, "Pelvis"},
	{MVNSegment::L5, "L5"},
	{MVNSegment::L3, "L3"},
	{MVNSegment::T12, "T12"},
	{MVNSegment::T8, "T8"},
	{MVNSegment::Neck, "Neck"},
	{MVNSegment::Head, "Head"},
	{MVNSegment::RightShoulder, "RightShoulder"},
	{MVNSegment::RightUpperArm, "RightUpperArm"},
	{MVNSegment::RightForearm, "RightForearm"},
	{MVNSegment::RightHand, "RightHand"},
	{MVNSegment::LeftShoulder, "LeftShoulder"},
	{MVNSegment::LeftUpperArm, "LeftUpperArm"},
	{MVNSegment::LeftForearm, "LeftForearm"},
	{MVNSegment::LeftHand, "LeftHand"},
	{MVNSegment::RightUpperLeg, "RightUpperLeg"},
	{MVNSegment::RightLowerLeg, "RightLowerLeg"},
	{MVNSegment::RightFoot, "RightFoot"},
	{MVNSegment::RightToe, "RightToe"},
	{MVNSegment::LeftUpperLeg, "LeftUpperLeg"},
	{MVNSegment::LeftLowerLeg, "LeftLowerLeg"},
	{MVNSegment::LeftFoot, "LeftFoot"},
	{MVNSegment::LeftToe, "LeftToe"}
	//{MVNSegment::Prop1, "Prop1"},
	//{MVNSegment::Prop2, "Prop2"},
	//{MVNSegment::Prop3, "Prop3"},
	//{MVNSegment::Prop4, "Prop4"}
};