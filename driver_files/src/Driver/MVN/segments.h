#pragma once

#include <unordered_map>
#include <string>

enum Segment {
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
	LeftToe = 22,
	Prop1 = 24,
	Prop2 = 25,
	Prop3 = 26,
	Prop4 = 27
};

const std::unordered_map<Segment, std::string> SegmentName = {
	{Pelvis, "Pelvis"},
	{L5, "L5"},
	{L3, "L3"},
	{T12, "T12"},
	{T8, "T8"},
	{Neck, "Neck"},
	{Head, "Head"},
	{RightShoulder, "RightShoulder"},
	{RightUpperArm, "RightUpperArm"},
	{RightForearm, "RightForearm"},
	{RightHand, "RightHand"},
	{LeftShoulder, "LeftShoulder"},
	{LeftUpperArm, "LeftUpperArm"},
	{LeftForearm, "LeftForearm"},
	{LeftHand, "LeftHand"},
	{RightUpperLeg, "RightUpperLeg"},
	{RightLowerLeg, "RightLowerLeg"},
	{RightFoot, "RightFoot"},
	{RightToe, "RightToe"},
	{LeftUpperLeg, "LeftUpperLeg"},
	{LeftLowerLeg, "LeftLowerLeg"},
	{LeftFoot, "LeftFoot"},
	{LeftToe, "LeftToe"},
	{Prop1, "Prop1"},
	{Prop2, "Prop2"},
	{Prop3, "Prop3"},
	{Prop4, "Prop4"}
};