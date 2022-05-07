#include "BVH.h"
#include <iostream>
#include <sstream>
#include <filesystem>

static unsigned int channelOffsetNum = 0;
 
BVH::BVH()
{
	root = new Joint();
	motion = new Motion();

	//loaded = load_file(name);
}

BVH::~BVH()
{
	clear();

	loaded = false;
}

bool BVH::load_stream(std::stringstream& stream)
{
	std::string line;

	getline(stream, line);
	vec = splitString(line, ' ');

	if (vec[0] != "HIERARCHY")
	{
		//file.close();
		//return false;
	}

	getline(stream, line);
	vec = splitString(line, ' ');

	//root = new Joint();

	bool res = loadHierarchy(&stream, NULL, root);

	if (res == false)
	{
		//file.close();
		//return false;
	}

	getline(file, line);
	vec = splitString(line, ' ');

	if (vec[0] != "MOTION")
	{
		//file.close();
		//return false;
	}

	//motion = new Motion();

	res = loadMotion(&stream, motion);

	if (res == false)
	{
		//file.close();
		//return false;
	}

	//std::cout << "loading success" << std::endl;

	//file.close();//*/

	return true;
}

bool BVH::load_file(const std::filesystem::path& path)
{
	file.open(path, std::ios::in);
	if (file.is_open() == 0)  return false;

	std::stringstream stream;
	stream << file.rdbuf();
	file.close();

	load_stream(stream);
}

bool BVH::loadHierarchy(std::stringstream*stream, Joint *parent, Joint *out)
{
	bool isEnd = false;
	
	std::string line;

	if (vec[0] == "JOINT" || vec[0] == "ROOT")
	{
		out->name = vec[1];
	}

	if (vec[0] == "End")
	{
		isEnd = true;
		out->name = parent->name + "End"; 
	}

	out->parent = parent;
	out->end = isEnd;

	getline(*stream, line);
	getline(*stream, line);
	vec = splitString(line, ' ');

	if (vec[0] != "OFFSET") return false;

	out->offset.x = atof(vec[1].c_str());
	out->offset.y = atof(vec[2].c_str());
	out->offset.z = atof(vec[3].c_str());

	getline(*stream, line);
	vec = splitString(line, ' ');

	if (isEnd)
	{
		return true;
	}

	if(vec[0] != "CHANNELS") return false;
	
	out->numChannels = atoi(vec[1].c_str()); 
	
	out->offsetChannel = channelOffsetNum;
	channelOffsetNum += out->numChannels;

	for (int k =0; k<out->numChannels; k++)
	{
		if (vec[k+2] == "Xposition")
			out->channelEnums.push_back(XPOSITION);
		else if (vec[k+2] == "Yposition")
			out->channelEnums.push_back(YPOSITION);
		else if (vec[k+2] == "Zposition")
			out->channelEnums.push_back(ZPOSITION);
		else if (vec[k+2] == "Zrotation")
			out->channelEnums.push_back(ZROTATION);
		else if (vec[k+2] == "Yrotation")
			out->channelEnums.push_back(YROTATION);
		else if (vec[k+2] == "Xrotation")
			out->channelEnums.push_back(XROTATION);
	}//*/

	getline(*stream, line);
	vec = splitString(line, ' ');

	while (vec[0] == "JOINT" || vec[0] == "End")
	{
		Joint *   newJoint = new Joint();
	
		out->children.push_back(newJoint);

		bool res = loadHierarchy(stream, out, newJoint);
		
		if (res == false) return false;

		getline(*stream, line); // this is working for end site
		vec = splitString(line, ' ');
	}//*/

	//getline(*stream, line); // doesn't work here i think...
	//vec = splitString(line, ' ');


	/*for(int i = 0; i < 7; i++)
	{
		getline(*stream, line);
		std::vector<std::string> vec = splitString(line, ' ');

		for (int j = 0; j < vec.size(); j++)
			std::cout << vec[j].c_str() << std::endl;

		std::cout << std::endl;
	}//*/
	return true;
}

bool BVH::loadMotion(std::stringstream*stream, Motion *out)
{
	std::string line;

	getline(*stream, line);
	vec = splitString(line, ' ');

	if(vec[0] != "Frames:") return false;

	out->numFrames = atoi(vec[1].c_str());
	
	getline(*stream, line);
	vec = splitString(line, ' ');

	if(vec[0] != "Frame") return false;
	if(vec[1] != "Time:") return false;

	out->frameTime = atof(vec[2].c_str());

	out->data = new float *[out->numFrames];
	for (int i = 0; i < out->numFrames; i++)
	{
		out->data[i] = new float[channelOffsetNum];
	}

	for (int i = 0; i<out->numFrames; i++)
	{
		getline(*stream, line);
		vec = splitString(line, ' ');

		for (int j = 0; j < channelOffsetNum; j++)
		{
			out->data[i][j] = atof(vec[j].c_str());
		}
		//std::cout << "Line: " << i << std::endl;
	}//*/

	return true;
}

void BVH::clear()
{
	for (int i = 0; i < motion->numFrames; i++)
	{
		delete[] motion->data[i];
	}
	delete[] motion->data;

	motion->numFrames = 0;
	motion->frameTime = 0.0f;
	channelOffsetNum = 0;

	delete motion;

	clearHierarchy(root);

}

void BVH::clearHierarchy(Joint *joint)
{
	for(int i = joint->children.size()-1; i >= 0; i--)
	{
		clearHierarchy(joint->children[i]);
	}

	delete joint;
}

std::vector<std::string> BVH::splitString(std::string &str, char delimiter)
{
	std::vector<std::string> words;
    std::string word;
    std::stringstream stream(str);

    while( getline(stream, word, delimiter) )
	{
		while(word.front() == '\t')
			word.erase(word.begin());

        words.push_back(word);
	}

	return words;
}

bool BVH::isLoaded()
{
	return loaded;
}

float BVH::getFrameTime()
{
	return motion->frameTime;
}

/*
void BVH::Render(unsigned int frameNo)
{
	unsigned int frame = frameNo % motion->numFrames;

	RenderFigure(root, motion->data[frame]);
}

void BVH::RenderFigure(Joint* joint, float* data)
{
	//matrixStack.push(currentMatrix);

	//do stuff
	glPushMatrix();

	if ( joint->parent == NULL )
	{
		//is root -> first three are position
		glTranslatef(data[0], data[1], data[2]);
	}
	else
	{
		glTranslatef(joint->offset.x, joint->offset.y , joint->offset.z);
	}

	for (int i=0; i<joint->channelEnums.size(); i++)
	{
		Channels channel = joint->channelEnums[i];
		if (channel == XROTATION )
			glRotatef( data[joint->offsetChannel+i], 1.0f, 0.0f, 0.0f );
		else if (channel == YROTATION)
			glRotatef( data[joint->offsetChannel+i], 0.0f, 1.0f, 0.0f );
		else if ( channel == ZROTATION )
			glRotatef( data[joint->offsetChannel+i], 0.0f, 0.0f, 1.0f );
	}

	if (joint->children.size() == 0)
	{
		RenderLine(joint->offset.x, joint->offset.y, joint->offset.z);
	}
	else
	{
		for (int i = 0; i < joint->children.size(); i++)
		{
			Joint *child = joint->children[i];
			RenderLine(child->offset.x, child->offset.y, child->offset.z);
		}
	}

	for (int j = 0; j < joint->children.size(); j++)
	{
		RenderFigure(joint->children[j], data);
	}

	glPopMatrix();
	//matrixStack.pop();

}

void BVH::RenderLine(float x, float y, float z)
{
	//matrixStack.push(currentMatrix);
	glPushMatrix();

	//do stuff
	float  dirX = x;
	float  dirY = y;
	float  dirZ = z;
	float  lineLength = sqrt((dirX * dirX) + (dirY * dirY) + (dirZ * dirZ));

	float  length = lineLength;
	
	if ( length < 0.0001 ) 
	{ 
		dirX = 0.0; 
		dirY = 0.0; 
		dirZ = 1.0;  
		length = 1.0;
	}
	dirX /= length; 
	dirY /= length; 
	dirZ /= length;

	float upX = 0.0;
	float upY = 1.0;
	float upZ = 0.0;

	float perpX = upY * dirZ - upZ * dirY;
	float perpY = upZ * dirX - upX * dirZ;
	float perpZ = upX * dirY - upY * dirX;
	
	length = sqrt((perpX * perpX) + (perpY * perpY) + (perpZ * perpZ));
	if ( length < 0.0001 ) 
	{ 
		perpX = 1.0; 
		perpY = 0.0; 
		perpZ = 0.0;  
		length = 1.0;
	}
	perpX /= length; 
	perpY /= length; 
	perpZ /= length;

	float m[16] = {perpX, perpY, perpZ, 0.0,
	               upX,   upY,   upZ,   0.0,
	               dirX,  dirY,  dirZ,  0.0,
	               0.0,   0.0,   0.0,   1.0};
	glMultMatrixf(m);

	if (quadObj == NULL)
		quadObj = gluNewQuadric();

	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);

	gluCylinder(quadObj, 0.1f, 0.1f, lineLength, 12.0f, 9.0f); 

	glPopMatrix();
	//matrixStack.pop();
}
*/

/*int  main( int argc, char ** argv )
{
	wchar_t test2[] = L"01_01.bvh";

	BVH* bvh = new BVH(test2);

	if (bvh)
		delete bvh;
	
	//std::cout << "Konstruktor: " << numConst << " Destruktor: " << numDestr << std::endl;

	int a; 
	std::cin >> a;
	
	return 0;
}*/