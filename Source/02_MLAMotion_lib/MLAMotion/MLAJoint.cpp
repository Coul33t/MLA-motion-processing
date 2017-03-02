#include "MLAJoint.h"

Joint::Joint() {
	m_parent = nullptr;
}

Joint::~Joint() {
	free(m_parent);
	for(unsigned int i=0 ; i<m_child.size() ; i++)
		free(m_child.at(i));
}

Joint::Joint(const Joint& joint) {
	m_jointName = joint.m_jointName;
	m_positions = joint.m_positions;
	m_orientations = joint.m_orientations;
}

Joint& Joint::operator=(const Joint& joint) {
	m_jointName = joint.m_jointName;
	m_positions = joint.m_positions;
	m_orientations = joint.m_orientations;
	return *this;
}

void Joint::setJointName(const std::string& jointName) {
	m_jointName = jointName;
}

void Joint::setPositions(const glm::dvec3& positions) {
	m_positions = positions;
}

void Joint::setOrientations(const glm::quat& orientations) {
	m_orientations = orientations;
}

const std::string& Joint::getJointName() const {
	return m_jointName;
}

const glm::dvec3& Joint::getPositions() const {
	return m_positions;
}

const glm::quat& Joint::getOrientations() const {
	return m_orientations;
}

void Joint::setParent(Joint* parent) {
	m_parent = parent;
}

void Joint::addChild(Joint* child) {
	m_child.push_back(child);
}

std::vector<Joint*> Joint::getChilds() {
	return m_child;
}

Joint* Joint::getParent() {
	return m_parent;
}