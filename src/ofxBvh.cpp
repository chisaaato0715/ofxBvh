#include "ofxBvh.h"


static inline void billboard();

ofxBvh::~ofxBvh()
{
	unload();
}

void ofxBvh::load(string path)
{
	path = ofToDataPath(path);
	
	string data = ofBufferFromFile(path).getText();
	
	const size_t HIERARCHY_BEGIN = data.find("HIERARCHY", 0);
	const size_t MOTION_BEGIN = data.find("MOTION", 0);
	
	if (HIERARCHY_BEGIN == string::npos
		|| MOTION_BEGIN == string::npos)
	{
		ofLogError("ofxBvh", "invalid bvh format");
		return;
	}
	
	parseHierarchy(data.substr(HIERARCHY_BEGIN, MOTION_BEGIN));
	parseMotion(data.substr(MOTION_BEGIN));
	
	currentFrame = frames[0];
	
	int index = 0;
	updateJoint(index, currentFrame, root);
	
	frame_new = false;
}

void ofxBvh::unload()
{
	for (int i = 0; i < joints.size(); i++)
		delete joints[i];
	
	joints.clear();
	
	root = NULL;
	
	frames.clear();
	currentFrame.clear();
	
	num_frames = 0;
	frame_time = 0;
	
	rate = 1;
	play_head = 0;
	playing = false;
	loop = false;
	
	need_update = false;
}

void ofxBvh::play()
{
	playing = true;
}

void ofxBvh::stop()
{
	playing = false;
}

bool ofxBvh::isPlaying()
{
	return playing;
}

void ofxBvh::setLoop(bool yn)
{
	loop = yn;
}

bool ofxBvh::isLoop() { return loop; }

void ofxBvh::setRate(float rate)
{
	this->rate = rate;
}

void ofxBvh::updateJoint(int& index, const FrameData& frame_data, ofxBvhJoint *joint)
{
	ofVec3f translate;
	ofQuaternion rotate;
	
	for (int i = 0; i < joint->channel_type.size(); i++)
	{
		float v = frame_data[index++];
		ofxBvhJoint::CHANNEL t = joint->channel_type[i];
		
		if (t == ofxBvhJoint::X_POSITION)
			translate.x = v;
		else if (t == ofxBvhJoint::Y_POSITION)
			translate.y = v;
		else if (t == ofxBvhJoint::Z_POSITION)
			translate.z = v;
		else if (t == ofxBvhJoint::X_ROTATION)
			rotate = ofQuaternion(v, ofVec3f(1, 0, 0)) * rotate;
		else if (t == ofxBvhJoint::Y_ROTATION)
			rotate = ofQuaternion(v, ofVec3f(0, 1, 0)) * rotate;
		else if (t == ofxBvhJoint::Z_ROTATION)
			rotate = ofQuaternion(v, ofVec3f(0, 0, 1)) * rotate;
	}
	
	translate += joint->initial_offset;
	
	joint->matrix.makeIdentityMatrix();
	joint->matrix.glTranslate(translate);
	joint->matrix.glRotate(rotate);
	
	joint->global_matrix = joint->matrix;
	joint->offset = translate;
	
	if (joint->parent)
	{
		joint->global_matrix.postMult(joint->parent->global_matrix);
	}
	
	for (int i = 0; i < joint->children.size(); i++)
	{
		updateJoint(index, frame_data, joint->children[i]);
	}

}

void ofxBvh::update()
{
	frame_new = false;
	
	if (playing && ofGetFrameNum() > 1)
	{
		int last_index = getFrame();
		
		play_head += ofGetLastFrameTime() * rate;
		int index = getFrame();
		
		if (index != last_index)
		{
			need_update = true;
			
			currentFrame = frames[index];
			
			if (index >= frames.size())
			{
				if (loop)
					play_head = 0;
				else
					playing = false;
			}
			
			if (play_head < 0)
				play_head = 0;
		}
	}
	
	if (need_update)
	{
		need_update = false;
		frame_new = true;
		
		int index = 0;
		updateJoint(index, currentFrame, root);
	}
}

void ofxBvh::draw()
{	
	ofPushStyle();
	ofFill();
	
	for (int i = 0; i < joints.size(); i++)
	{
		ofxBvhJoint *o = joints[i];
		glPushMatrix();
		glMultMatrixf(o->getGlobalMatrix().getPtr());
		
		if (o->isSite())
		{
			ofSetColor(ofColor::yellow);
			billboard();
			ofCircle(0, 0, 6);
			//ofDrawCone(6, 6);
			//ofDrawAxis(5);
		}
		else if (o->getChildren().size() == 1)
		{
			ofSetColor(ofColor::white);		
			billboard();
			ofCircle(0, 0, 2);
			//ofDrawCone(6, 6);
			//ofDrawAxis(5);

		}
		else if (o->getChildren().size() > 1)
		{
			if (o->isRoot())
				ofSetColor(ofColor::cyan);
			else
				ofSetColor(ofColor::green);
			
			billboard();
			ofCircle(0, 0, 4);
			//ofDrawCone(6, 6);
			//ofDrawAxis(5);
		}
		
		glPopMatrix();
	}
	
	ofPopStyle();
}

bool ofxBvh::isFrameNew()
{
	return frame_new;
}

void ofxBvh::setFrame(int index)
{
	if (ofInRange(index, 0, frames.size()) && getFrame() != index)
	{
		currentFrame = frames[index];
		play_head = (float)index * frame_time;
		
		need_update = true;
	}
}

int ofxBvh::getFrame()
{
	return floor(play_head / frame_time);
}

void ofxBvh::setPosition(float pos)
{
	setFrame((float)frames.size() * pos);
}

float ofxBvh::getPosition()
{
	return play_head / (float)frames.size();
}

float ofxBvh::getDuration()
{
	return (float)frames.size() * frame_time;
}

void ofxBvh::parseHierarchy(const string& data)
{
	vector<string> tokens;
	string token;
	
	total_channels = 0;
	num_frames = 0;
	frame_time = 0;
	
	for (int i = 0; i < data.size(); i++)
	{
		char c = data[i];
		
		if (isspace(c))
		{
			if (!token.empty()) tokens.push_back(token);
			token.clear();
		}
		else
		{
			token.push_back(c);
		}
	}
	
	int index = 0;
	while (index < tokens.size())
	{
		if (tokens[index++] == "ROOT")
		{
			root = parseJoint(index, tokens, NULL);
		}
	}
}

ofxBvhJoint* ofxBvh::parseJoint(int& index, vector<string> &tokens, ofxBvhJoint *parent)
{
	string name = tokens[index++];
	ofxBvhJoint *joint = new ofxBvhJoint(name, parent);
	if (parent) parent->children.push_back(joint);
	
	joint->bvh = this;
	
	joints.push_back(joint);
	jointMap[name] = joint;
	
	while (index < tokens.size())
	{
		string token = tokens[index++];
		
		if (token == "OFFSET")
		{
			joint->initial_offset.x = ofToFloat(tokens[index++]);
			joint->initial_offset.y = ofToFloat(tokens[index++]);
			joint->initial_offset.z = ofToFloat(tokens[index++]);
			
			joint->offset = joint->initial_offset;
		}
		else if (token == "CHANNELS")
		{
			int num = ofToInt(tokens[index++]);
			
			joint->channel_type.resize(num);
			total_channels += num;
			
			for (int i = 0; i < num; i++)
			{
				string ch = tokens[index++];
				
				char axis = tolower(ch[0]);
				char elem = tolower(ch[1]);
				
				if (elem == 'p')
				{
					if (axis == 'x')
						joint->channel_type[i] = ofxBvhJoint::X_POSITION;
					else if (axis == 'y')
						joint->channel_type[i] = ofxBvhJoint::Y_POSITION;
					else if (axis == 'z')
						joint->channel_type[i] = ofxBvhJoint::Z_POSITION;
					else
					{
						ofLogError("ofxBvh", "invalid bvh format");
						return NULL;
					}
				}
				else if (elem == 'r')
				{
					if (axis == 'x')
						joint->channel_type[i] = ofxBvhJoint::X_ROTATION;
					else if (axis == 'y')
						joint->channel_type[i] = ofxBvhJoint::Y_ROTATION;
					else if (axis == 'z')
						joint->channel_type[i] = ofxBvhJoint::Z_ROTATION;
					else
					{
						ofLogError("ofxBvh", "invalid bvh format");
						return NULL;
					}
				}
				else
				{
					ofLogError("ofxBvh", "invalid bvh format");
					return NULL;
				}
			}
		}
		else if (token == "JOINT"
				 || token == "End")
		{
			parseJoint(index, tokens, joint);
		}
		else if (token == "}")
		{
			break;
		}
	}
	
	return joint;
}

void ofxBvh::parseMotion(const string& data)
{
	vector<string> lines = ofSplitString(data, "\n", true, true);
	
	int index = 0;
	
	while (index < lines.size())
	{
		string line = lines[index];
		
		if (line.empty())
		{
			index++;
			continue;
		}
		
		if (line.find("MOTION") != string::npos) {}
		else if (line.find("Frames:") != string::npos)
		{
			num_frames = ofToInt(ofSplitString(line, ":")[1]);
		}
		else if (line.find("Frame Time:") != string::npos)
		{
			frame_time = ofToFloat(ofSplitString(line, ":")[1]);
		}
		else break;
		
		index++;
	}
	
	while (index < lines.size())
	{
		string line = lines[index];
		vector<string> channels = ofSplitString(line, " ");

		if (channels.size() != total_channels)
		{
			ofLogError("ofxBvh", "channel size mismatch");
			return;
		}
		
		char buf[64];
		FrameData data;
		for (int i = 0; i < channels.size(); i++)
		{
			float v;
			sscanf(channels[i].c_str(), "%f", &v);
			data.push_back(v);
		}
		
		frames.push_back(data);
		
		index++;
	}
	
	if (num_frames != frames.size())
		ofLogWarning("ofxBvh", "frame size mismatch");
}

const ofxBvhJoint* ofxBvh::getJoint(int index)
{
	return joints.at(index);
}

const ofxBvhJoint* ofxBvh::getJoint(string name)
{
	return jointMap[name];
}

static inline void billboard()
{
	GLfloat m[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, m);
	
	float inv_len;
	
	m[8] = -m[12];
	m[9] = -m[13];
	m[10] = -m[14];
	inv_len = 1. / sqrt(m[8] * m[8] + m[9] * m[9] + m[10] * m[10]);
	m[8] *= inv_len;
	m[9] *= inv_len;
	m[10] *= inv_len;
	
	m[0] = -m[14];
	m[1] = 0.0;
	m[2] = m[12];
	inv_len = 1. / sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
	m[0] *= inv_len;
	m[1] *= inv_len;
	m[2] *= inv_len;
	
	m[4] = m[9] * m[2] - m[10] * m[1];
	m[5] = m[10] * m[0] - m[8] * m[2];
	m[6] = m[8] * m[1] - m[9] * m[0];
	
	glLoadMatrixf(m);
}




//=====    追加    ======================================================================

//HIERARCHYを表示する
void  ofxBvh::getjoint(const ofxBvhJoint * joint, int z, stringstream& sstr) {

	string jointgroup;

	if (joint->isSite())jointgroup = "End";
	else if (joint->isRoot())jointgroup = "ROOT";
	else jointgroup = "JOINT";

	getSpace(z, sstr); sstr << jointgroup << " " << joint->getName() << endl;
	getSpace(z, sstr); sstr << "{" << endl;
	if (joint->isRoot()) { getSpace(z + 1, sstr); sstr << "OFFSET" << "\ " << "0.000000 0.000000 0.000000" << endl; }
	else { getSpace(z + 1, sstr); sstr << "OFFSET" << " " << fixed << setprecision(6) << joint->getOffset().x << " " << joint->getOffset().y << " " << joint->getOffset().z << endl; }

	if (joint->isSite()) {}
	else if (joint->isRoot()) {
		getSpace(z + 1, sstr); sstr << "CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation" << endl;
	}
	else { getSpace(z + 1, sstr); sstr << "CHANNELS 3 Yrotation Xrotation Zrotation" << endl; }

	for (int i = 0; i < joint->children.size(); i++) {
		getjoint(joint->children[i], z + 1, sstr);
	}

	getSpace(z, sstr); sstr << "}" << endl;

}


//新しくBvhファイルを生成する関数
void ofxBvh::newBvhdata()
{
	stringstream sstr;
	sstr.str("");

	//HIERARCYの表示
	sstr << "HIERARCHY" << endl;
	getjoint(joints[0], 0, sstr);

	//MOTIONデータの表示
	sstr << "MOTION" << endl
		<< "Frames:"" " << num_frames << endl
		<< "Frame Time:"" " << frame_time << endl;

	for (int i = 0; i < num_frames; i++) {
		for (int j = 0; j < total_channels; j++)
		{
			sstr << fixed << setprecision(6) << frames.at(i).at(j) << " ";
		}
		sstr << endl;
	}

	ofstream ofs("data/newdata1.bvh"); //ファイルオープン
	ofs << sstr.str();

}


//z個スペースを表示する関数
void ofxBvh::getSpace(int z, stringstream& sstr)
{
	for (int i = 0; i < z; i++)
	{
		sstr << " ";
	}
}


void ofxBvh::updateIK(ofxBvhJoint *joint) {

	ofxBvhJoint *endjoint = joint;

	for (int i = 0; joint != joints[0]; i++) 
	{

		ofMatrix4x4 worldToBone = joint->getParent()->getGlobalMatrix().getInverse();

		ofVec3f localTarget = target * worldToBone;
		ofVec3f localEffector = endjoint -> getPosition() * worldToBone;

		ofQuaternion m1;
		m1.makeRotate(localTarget, localEffector);

		ofVec3f axis;
		float angle;

		ofQuaternion m2;
		m2 = joint->getParent()->getMatrix().getRotate() *  m1;

		m2.getRotate(angle, axis);

		m2.makeRotate(ofClamp(angle, 170, 190), axis);

		ofMatrix4x4 mat;
		mat.setTranslation(joint->getParent()->getMatrix().getTranslation());
		mat.setRotate(m2);
		joint->getParent()->setMatrix(mat);

		joint = joint->getParent();

		updateFK(joints[0]);

	}

}


void ofxBvh::updateFK(ofxBvhJoint *joint) {

	if (joint->isRoot()) {}
	else { joint->setGlobalMatrix(joint->getMatrix() * joint->getParent()->getGlobalMatrix());}

	for (int i = 0; i < joint->getChildren().size(); i++)
	{
		updateFK(joint->getChildren().at(i));
	}

}

int ofxBvh::selectJoint(ofVec3f target) {

	ofVec3f distance;
	for (int i = 0; i < getNumJoints(); i++) {

		distance = target - joints[i]->getPosition();

		if (-2 < distance.x > 2 && -2 < distance.y > 2) {
			selectjoint = i;
		}

	}

	return selectjoint;
}
