#include "MLAFrame.h"

Frame::Frame() {

}

Frame::~Frame() {
	if (!m_joints.empty())
		for (unsigned int i=0 ; i<m_joints.size() ; i++)
			delete (m_joints[i]);
}

Frame::Frame(const Frame& frame) {
	m_joints = frame.m_joints;
	m_names = frame.m_names;
}

Frame& Frame::operator=(const Frame& frame) {
	m_joints = frame.m_joints;
	m_names = frame.m_names;
	return *this;
}

void Frame::setJoints(const std::vector<Joint*>& joints) {
	m_joints = joints;
}

const std::vector<Joint*>& Frame::getJoints() const {
	return m_joints;
}

void Frame::insertJoint(Joint* joint) {
	m_joints.push_back(joint);
	m_names[joint->getJointName()] = m_joints.size()-1; 
}


Joint* Frame::getJoint(const std::string& jointName) {
	if (m_names.count(jointName) != 0) {
		return m_joints[m_names[jointName]];
	}
	else {
		return 0;
	}
}

Joint* Frame::getJoint(const unsigned int idx) {
	if(idx < m_joints.size()) {
		return m_joints[idx];
	}
	else {
		return 0;
	}
}

void Frame::setNames(const std::map<std::string, unsigned int>& names) {
	m_names = names;
}

const std::map<std::string, unsigned int>& Frame::getNames() const {
	return m_names;
}

Frame* Frame::duplicateFrame() {
	Frame* copied_frame = new Frame();

	for (unsigned int i = 0; i < m_joints.size(); i++) {
		Joint* new_joint = new Joint(*m_joints[i]);

		new_joint->setParent(0);

		if (this->getJoints()[i]->getParent()) {
			std::string parentName = m_joints[i]->getParent()->getJointName();
			new_joint->setParent(copied_frame->getJoint(parentName));
			new_joint->getParent()->addChild(new_joint);
		}

		copied_frame->insertJoint(new_joint);
	}

	return copied_frame;
}