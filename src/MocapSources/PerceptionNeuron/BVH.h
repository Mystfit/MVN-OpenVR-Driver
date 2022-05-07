// Adapted from https://github.com/JaninaAlthoefer/MoCapViewer

#pragma once

#include <vector>
#include <stack>
#include <string.h>
#include <fstream>


struct Offset 
{
	float x, y, z;
};

enum Channels
{
	XPOSITION, YPOSITION, ZPOSITION,
	XROTATION, YROTATION, ZROTATION
};

struct Joint 
{
	std::string name;
	Joint *parent;
	Offset offset;
	bool end;
	unsigned int numChannels;
	unsigned int offsetChannel;
	std::vector<Channels> channelEnums;
	std::vector<Joint*> children;

};

struct Motion
{
	unsigned int numFrames;
	float frameTime;
	float **data;
};

class BVH {

public: 
	BVH();
	~BVH();

	bool isLoaded();
	float getFrameTime();
	bool load_stream(std::stringstream& stream);
	bool load_file(const std::filesystem::path& path);

	std::vector<std::string> splitString(std::string &str, char delimiter);
	bool loadHierarchy(std::stringstream *stream, Joint *parent, Joint *out);
	bool loadMotion(std::stringstream*stream, Motion *out);

	void clearHierarchy(Joint *joint);
	void clear();

private: 
	std::ifstream file;
	std::vector<std::string> vec;

	Joint* root;
	Motion* motion;

	bool loaded;

};
