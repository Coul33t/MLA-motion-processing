#include "MLAJoint.h"

Joint::Joint() {

}

Joint::Joint(const Joint& joint) {
	m_jointName = joint.m_jointName;
	m_offsets = joint.m_offsets;
	m_orientations = joint.m_orientations;
}

Joint& Joint::operator=(const Joint& joint) {
	m_jointName = joint.m_jointName;
	m_offsets = joint.m_offsets;
	m_orientations = joint.m_orientations;
	return *this;
}

void Joint::setJointName(const std::string& jointName) {
	m_jointName = jointName;
}

void Joint::setOffsets(const glm::vec3& offsets) {
	m_offsets = offsets;
}

void Joint::setOrientations(const glm::quat& orientations) {
	m_orientations = orientations;
}

const std::string& Joint::getJointName() const {
	return m_jointName;
}

const glm::vec3& Joint::getOffsets() const {
	return m_offsets;
}

const glm::quat& Joint::getOrientations() const {
	return m_orientations;
}