#include "MLAFrame.h"

Frame::Frame() {

}

Frame::~Frame() {
	if (!m_joints.empty())
		for (unsigned int i = 0; i < m_joints.size(); i++)
			delete (m_joints[i]);
}

Frame::Frame(const Frame& frame) {
	m_joints = frame.m_joints;
}

Frame& Frame::operator=(const Frame& frame) {
	m_joints = frame.m_joints;
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
}

Joint* Frame::getRoot() const {
	return m_root;
}

void Frame::setRoot(Joint* root) {
	m_root = root;
}

Joint* Frame::getJoint(const std::string& joint_name) {
	auto it = find_if(m_joints.begin(), m_joints.end(), [&joint_name](const Joint* obj) {return obj->getName() == joint_name; });
	if (it != m_joints.end()) {
		return *it;
	}
	else {
		return nullptr;
	}
}

Joint* Frame::getJoint(const unsigned int idx) {
	if(idx < m_joints.size()) {
		return m_joints[idx];
	}
	else {
		return nullptr;
	}
}

Frame* Frame::duplicateFrame() {
	Frame* copied_frame = new Frame();

	for (unsigned int i = 0; i < m_joints.size(); i++) {
		Joint* new_joint = new Joint(*m_joints[i]);

		new_joint->setParent(nullptr);

		// If the original joint has at least a parent
		if (this->getJoints()[i]->getParent()) {
			std::string parentName = m_joints[i]->getParent()->getName();
			new_joint->setParent(copied_frame->getJoint(parentName));
			new_joint->getParent()->addChild(new_joint);
		}

		// else, it's the root
		else {
			copied_frame->setRoot(new_joint);
		}

		copied_frame->insertJoint(new_joint);
	}

	return copied_frame;
}