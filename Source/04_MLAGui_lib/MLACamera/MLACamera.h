#ifndef __MLA_CAMERA__
#define __MLA_CAMERA__

#include "MLAInput\MLAInput.h"
#include <iostream>

#pragma warning(push, 0)
#include "glm.hpp"
#include "gtx/transform.hpp"
#pragma warning(pop)

#define CAMERA_POS_X -10
#define CAMERA_POS_Y 90
#define CAMERA_POS_Z 150

#define CAMERA_ORI_X 0
#define CAMERA_ORI_Y 50
#define CAMERA_ORI_Z 0

class Camera {

public:
	Camera();
	Camera(glm::dvec3 eye_positon, glm::dvec3 target_center, glm::dvec3 vertical_axis);
	virtual ~Camera();
	void Orient(int x_rel, int y_rel);
	void Move(Input const &input);
	void LookAt(glm::dmat4 &modelview);

	void PrintValues();


private:
	double m_phi_angle;
	double m_theta_angle;

	glm::dvec3 m_orientation_vector;

	glm::dvec3 m_vertical_axis;
	glm::dvec3 m_lateral_shift;

	glm::dvec3 m_eye_position;
	glm::dvec3 m_target_center;

};
#endif //__MLA_CAMERA__