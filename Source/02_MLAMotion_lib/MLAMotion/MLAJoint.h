/**
Class for a Joint. A joint has a name, and is composed of a vector for the initial offsets, and a quaternion for 1 frame.
Character.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_JOINT_H__
#define __MLA_JOINT_H__


#include "MLATools_lib.h"

class Joint {

public:
	Joint();
	Joint(const Joint& joint);

	Joint& operator=(const Joint& joint);

	void setJointName(const std::string& jointName);
	void setOffsets(const glm::vec3& offsets);
	void setOrientations(const glm::quat& orientations);

	const std::string& getJointName() const;
	const glm::vec3& getOffsets() const;
	const glm::quat& getOrientations() const;

private:
	std::string m_jointName;

	// a vector with the initial offset
	glm::vec3 m_offsets;

	// a quaternion holding the orientation for the frame
	glm::quat m_orientations;
};

#endif //__MLA_JOINT_H__
