/**
Class for a Frame. A frame is composed of a graph (tree) of joints.
Character.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_FRAME_H__
#define __MLA_FRAME_H__


#include "MLATools_lib.h"
#include "MLAJoint.h"

class Frame {

public:
	Frame();
	Frame(const Frame& frame);

	Frame& operator=(const Frame& frame);

	void setJoints(const MlaGraph<Joint>& joints);
	const MlaGraph<Joint>& getJoints() const;


private:
	MlaGraph<Joint> m_joints;
};

#endif //__MLA_FRAME_H__
