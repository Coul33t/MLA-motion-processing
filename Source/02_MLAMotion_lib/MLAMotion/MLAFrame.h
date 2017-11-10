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

	Frame& operator=(const Frame& frame);

	void setJoints(const std::vector<Joint*>&);
	const std::vector<Joint*>& getJoints() const;

	Joint* getJoint(const std::string&);
	Joint* getJoint(const unsigned int);
	void insertJoint(Joint*);

	void setNames(const std::map<std::string, unsigned int>&);
	const std::map<std::string, unsigned int>& getNames() const;

	Frame* duplicateFrame();

private:
	std::vector<Joint*> m_joints;
	std::map<std::string, unsigned int> m_names;
};

#endif //__MLA_FRAME_H__
