#ifndef __MLA_CAMERA__
#define __MLA_CAMERA__

#include "MLAInput\MLAInput.h"

#include "glm.hpp"
#include "gtx/transform.hpp"

class Camera {

public:
	Camera();
	Camera(glm::dvec3 eyePositon, glm::dvec3 targetCenter, glm::dvec3 verticalAxis);
	virtual ~Camera();
	void Orient(int xRel, int yRel);
	void Move(Input const &input);
	void LookAt(glm::dmat4 &modelview);


private:
	double m_phiAngle;
	double m_thetaAngle;

	glm::dvec3 m_orientationVector;

	glm::dvec3 m_verticalAxis;
	glm::dvec3 m_lateralShift;

	glm::dvec3 m_eyePosition;
	glm::dvec3 m_targetCenter;

};
#endif //__MLA_CAMERA__