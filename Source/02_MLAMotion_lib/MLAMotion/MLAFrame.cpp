/**
* \file MLAFrame.cpp
* \brief This file contains the Frame class, representing a single frame for a motion.
* \author Quentin.C
* \version 0.1
* \date 12 February 2019
*/

#include "MLAFrame.h"

/** Constructor
*/
Frame::Frame() {
	m_root = nullptr;
}

/** Destructor
*/
Frame::~Frame() {
	// No need to delete m_root since it will be deleted by this
	/*if (!m_joints.empty())
		for (unsigned int i = 0; i < m_joints.size(); i++)
			delete (m_joints[i]);*/
}

// Error? Need a deep copy of the other frame m_joints?
/**
* @param frame The frame to copy
*/
Frame::Frame(const Frame& frame) {
	m_joints = frame.m_joints;
}

/*Frame::Frame(const std::shared_ptr<Frame> frame) {
	m_joints = frame->m_joints;
}*/

/** Equal operator overload
* @param frame The frame to copy
*/
Frame& Frame::operator=(const Frame& frame) {
	m_joints = frame.m_joints;
	return *this;
}

/*std::shared_ptr<Frame> Frame::operator=(const std::shared_ptr<Frame> frame) {
	m_joints = frame->m_joints;
	return std::make_shared<Frame>(this);
}*/

void Frame::setJoints(const std::vector<std::shared_ptr<Joint>>& joints) {
	m_joints = joints;
}

const std::vector<std::shared_ptr<Joint>>& Frame::getJoints() const {
	return m_joints;
}

void Frame::insertJoint(std::shared_ptr<Joint> joint) {
	m_joints.push_back(joint);
}

std::shared_ptr<Joint> Frame::getRoot() const {
	return m_root;
}

void Frame::setRoot(std::shared_ptr<Joint> root) {
	m_root = root;
}

/** Returns a vector containing all of the frame joints' names.
* @return A vector of joints' names for the current frame
*/
std::vector<std::string> Frame::getJointsName() const {
	std::vector<std::string> joint_names;

	for (auto it = m_joints.begin(); it != m_joints.end(); ++it) {
		joint_names.push_back((*it)->getName());
	}

	return joint_names;
}

/** Returns a joint data from its name.
* @param joint_name Name of the desired joint
* @return The joint if it was found, else a nullptr
*/
std::shared_ptr<Joint> Frame::getJoint(const std::string& joint_name) {
	auto it = find_if(m_joints.begin(), m_joints.end(), [&joint_name](const std::shared_ptr<Joint> obj) {return obj->getName() == joint_name; });
	if (it != m_joints.end()) {
		return *it;
	}
	else {
		return nullptr;
	}
}

/** Returns a joint data from its index.
* @param idx Index of the desired joint
* @return The joint if the index is inferior to the joints' vector size, else a nullptr
*/
std::shared_ptr<Joint> Frame::getJoint(const unsigned int idx) {
	if(idx < m_joints.size()) {
		return m_joints[idx];
	}
	else {
		return nullptr;
	}
}

/** Duplicates the current frame.
* @return A copy of the current frame
*/
std::shared_ptr<Frame> Frame::duplicateFrame() {
	std::shared_ptr<Frame> copied_frame = std::make_shared<Frame>();

	for (unsigned int i = 0; i < m_joints.size(); i++) {
		std::shared_ptr<Joint> new_joint = std::make_shared<Joint>(*m_joints[i]);

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