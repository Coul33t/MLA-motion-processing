/**
Class for a Joint. A Joint has a name, and is composed of a vector for the initial offsets, and a quaternion for 1 frame.
Joint.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_Joint_H__
#define __MLA_Joint_H__


#include "MLATools_lib.h"

class Joint {

public:
	Joint();
	~Joint();
	Joint(const Joint& Joint);

	Joint& operator=(const Joint& joint);

	void setJointName(const std::string& jointName);
	void setPositions(const glm::vec3& positions);
	void setOrientations(const glm::quat& orientations);

	const std::string& getJointName() const;
	const glm::vec3& getPositions() const;
	const glm::quat& getOrientations() const;

	void setParent(Joint* parent);
	void addChild(Joint* child);

	std::vector<Joint*> getChilds();
	Joint* getParent();

private:
	std::string m_jointName;

	// a vector with the initial offset
	glm::vec3 m_positions;

	// a quaternion holding the orientation for the frame
	glm::quat m_orientations;

	Joint* m_parent;
	std::vector<Joint*> m_child;
};

#endif //__MLA_Joint_H__
