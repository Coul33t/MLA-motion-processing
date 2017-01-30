#include "MLAFrame.h"

Frame::Frame() {

}

Frame::Frame(const Frame& frame) {
	m_joints = frame.m_joints;
}

Frame& Frame::operator=(const Frame& frame) {
	m_joints = frame.m_joints;
	return *this;
}

void Frame::setJoints(const MlaGraph<Joint>& joints) {
	m_joints = joints;
}

const MlaGraph<Joint>& Frame::getJoints() const {
	return m_joints;
}