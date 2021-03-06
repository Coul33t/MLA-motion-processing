#include "MLAJoint.h"

Joint::Joint() {
	m_parent = nullptr;
}

Joint::~Joint() {
}

Joint::Joint(const Joint& joint) {
	m_joint_name = joint.m_joint_name;
	m_positions = joint.m_positions;
	m_orientations = joint.m_orientations;
}

Joint& Joint::operator=(const Joint& joint) {
	m_joint_name = joint.m_joint_name;
	m_positions = joint.m_positions;
	m_orientations = joint.m_orientations;
	return *this;
}

void Joint::setName(const std::string& joint_name) {
	m_joint_name = joint_name;
}

void Joint::setPositions(const glm::dvec3& positions) {
	m_positions = positions;
}

void Joint::setOrientations(const glm::dquat& orientations) {
	m_orientations = orientations;
}

const std::string& Joint::getName() const {
	return m_joint_name;
}

const glm::dvec3& Joint::getPositions() const {
	return m_positions;
}

const glm::dquat& Joint::getOrientations() const {
	return m_orientations;
}

void Joint::setParent(Joint* parent) {
	m_parent = parent;
}

void Joint::addChild(Joint* child) {
	m_child.push_back(child);
}

const std::vector<Joint*>& Joint::getChilds() const {
	return m_child;
}

Joint* Joint::getParent() const {
	return m_parent;
}