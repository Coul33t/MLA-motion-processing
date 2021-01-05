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
	Frame(size_t joints_number);
	~Frame();
	Frame(const Frame& frame);

	Frame& operator=(const Frame& frame);

	void setJoints(const std::vector<Joint*>&);
	const std::vector<Joint*>& getJoints() const;
	std::vector<std::string> getJointsName() const;

	Joint* getJoint(const std::string&);
	Joint* getJoint(const unsigned int);
	void insertJoint(Joint*);

	Joint* getRoot() const;
	void setRoot(Joint*);

	Frame* duplicateFrame();

private:
	std::vector<Joint*> m_joints;
	// A pointer to the root (in m_joints)
	Joint* m_root;
};

#endif //__MLA_FRAME_H__
