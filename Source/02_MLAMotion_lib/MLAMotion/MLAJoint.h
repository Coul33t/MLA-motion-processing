/**
Class for a Joint. A Joint has a name, and is composed of a vector for the initial offsets, and a quaternion for 1 frame.
Joint.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_Joint_H__
#define __MLA_Joint_H__


#include "MLACommonInclude.h"

class Joint {

public:
	Joint();
	~Joint();
	Joint(const Joint& joint);
	//Joint(const std::shared_ptr<Joint> joint);

	Joint& operator=(const Joint& joint);
	//std::shared_ptr<Joint> operator=(const std::shared_ptr<Joint> joint);

	void setName(const std::string& jointName);
	void setPositions(const glm::dvec3& positions);
	void setOrientations(const glm::dquat& orientations);

	const std::string& getName() const;
	const glm::dvec3& getPositions() const;
	const glm::dquat& getOrientations() const;

	void setParent(std::shared_ptr<Joint> parent);
	void addChild(std::shared_ptr<Joint> child);

	const std::vector<std::shared_ptr<Joint>>& getChilds() const;
	std::shared_ptr<Joint> getParent() const;

private:
	// TODO: char* instead of std::string (performance related in getName())
	std::string m_joint_name;

	// a vector with the initial offset
	glm::dvec3 m_positions;

	// a quaternion holding the orientation for the frame
	glm::dquat m_orientations;

	std::shared_ptr<Joint> m_parent;
	std::vector<std::shared_ptr<Joint>> m_child;
};

#endif //__MLA_Joint_H__
