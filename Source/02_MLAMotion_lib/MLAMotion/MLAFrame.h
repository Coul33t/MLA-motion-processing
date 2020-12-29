/**
Class for a Frame. A frame is composed of a graph (tree) of joints.
MLAFrame.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_FRAME_H__
#define __MLA_FRAME_H__



#include "MLAJoint.h"

class Frame {

public:
	Frame();
	~Frame();
	Frame(const Frame& frame);
	//Frame(const std::shared_ptr<Frame> frame);

	Frame& operator=(const Frame& frame);
	//std::shared_ptr<Frame> operator=(const std::shared_ptr<Frame> frame);

	void setJoints(const std::vector<std::shared_ptr<Joint>>&);
	const std::vector<std::shared_ptr<Joint>>& getJoints() const;
	std::vector<std::string> getJointsName() const;

	std::shared_ptr<Joint> getJoint(const std::string&);
	std::shared_ptr<Joint> getJoint(const unsigned int);
	void insertJoint(std::shared_ptr<Joint>);

	std::shared_ptr<Joint> getRoot() const;
	void setRoot(std::shared_ptr<Joint>);

	std::shared_ptr<Frame> duplicateFrame();

private:
	std::vector<std::shared_ptr<Joint>> m_joints;
	// A pointer to the root (in m_joints)
	std::shared_ptr<Joint> m_root;
};

#endif //__MLA_FRAME_H__
