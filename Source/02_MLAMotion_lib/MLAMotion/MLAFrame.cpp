#include "MLAFrame.h"

Frame::Frame() {

}

Frame::~Frame() {
	if (!m_joints.empty())
		for (unsigned int i=0 ; i<m_joints.size() ; i++)
			free(m_joints.at(i));
}

Frame::Frame(const Frame& frame) {
	m_joints = frame.m_joints;
	m_names = frame.m_names;
	m_roots = frame.m_roots;
}

Frame& Frame::operator=(const Frame& frame) {
	m_joints = frame.m_joints;
	m_names = frame.m_names;
	m_roots = frame.m_roots;
	return *this;
}

void Frame::setJoints(const std::vector<Joint*>& joints) {
	m_joints = joints;
}

const std::vector<Joint*>& Frame::getJoints() const {
	return m_joints;
}

void Frame::setRoots(const std::vector<unsigned int>& roots) {
	m_roots = roots;
}

const std::vector<unsigned int>& Frame::getRoots() const {
	return m_roots;
}

void Frame::insertJoint(Joint* joint) {
	m_joints.push_back(joint);
	m_names[joint->getJointName()] = m_joints.size()-1; 
}


Joint* Frame::getJoint(const std::string jointName) {
	if (m_names.find(jointName) != m_names.end()) {
		return m_joints.at(m_names[jointName]);
	}
	else {
		return 0;
	}
}

void Frame::addRoot(unsigned int idx) {
	m_roots.push_back(idx);
}

void Frame::setNames(const std::map<std::string, unsigned int>& names) {
	m_names = names;
}

const std::map<std::string, unsigned int> Frame::getNames() const {
	return m_names;
}