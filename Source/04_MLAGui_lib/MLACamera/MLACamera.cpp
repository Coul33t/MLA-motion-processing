#include "MLACamera.h"

Camera::Camera() : m_phiAngle(0.0), m_thetaAngle(0.0), m_orientationVector(), m_verticalAxis(0, 1, 0),
m_lateralShift(), m_eyePosition(), m_targetCenter() {
	//ctor
}

Camera::Camera(glm::dvec3 eye_position, glm::dvec3 target_center, glm::dvec3 vertical_axis) : m_phiAngle(-35.26), m_thetaAngle(-135),
m_orientationVector(), m_verticalAxis(vertical_axis), m_lateralShift(), m_eyePosition(eye_position), m_targetCenter(target_center) {
	//ctor
}

Camera::~Camera() {
	//dtor
}

void Camera::Orient(int x_rel, int y_rel) {

	m_phiAngle += -y_rel * 0.5;
	m_thetaAngle += -x_rel * 0.5;

	if (m_phiAngle > 89.0)
		m_phiAngle = 89.0;

	else if (m_phiAngle < -89.0)
		m_phiAngle = -89.0;

	double phiAngleRad = glm::radians(m_phiAngle);
	double thetaAngleRad = glm::radians(m_thetaAngle);

	if (m_verticalAxis.x == 1.0) {

		m_orientationVector.x = glm::sin(phiAngleRad);
		m_orientationVector.y = glm::cos(phiAngleRad) * glm::cos(thetaAngleRad);
		m_orientationVector.z = glm::cos(phiAngleRad) * glm::sin(thetaAngleRad);
	}

	else if (m_verticalAxis.y == 1.0) {

		m_orientationVector.x = glm::cos(phiAngleRad) * glm::sin(thetaAngleRad);
		m_orientationVector.y = glm::sin(phiAngleRad);
		m_orientationVector.z = glm::cos(phiAngleRad) * glm::cos(thetaAngleRad);
	}

	else {

		m_orientationVector.x = glm::cos(phiAngleRad) * glm::cos(thetaAngleRad);
		m_orientationVector.y = glm::cos(phiAngleRad) * glm::sin(thetaAngleRad);
		m_orientationVector.z = glm::sin(phiAngleRad);
	}

	m_lateralShift = glm::cross(m_verticalAxis, m_orientationVector);
	m_lateralShift = glm::normalize(m_lateralShift);

	m_targetCenter = m_eyePosition + m_orientationVector;
}

void Camera::Move(Input const &input) {

	if (input.MouseMoving())
		Orient(input.GetXRel(), input.GetYRel());


	if (input.GetKey(SDL_SCANCODE_W)) {
		m_eyePosition = m_eyePosition + m_orientationVector * 0.5;
		m_targetCenter = m_eyePosition + m_orientationVector;
	}

	if (input.GetKey(SDL_SCANCODE_S)) {
		m_eyePosition = m_eyePosition - m_orientationVector * 0.5;
		m_targetCenter = m_eyePosition + m_orientationVector;
	}

	if (input.GetKey(SDL_SCANCODE_A)) {
		m_eyePosition = m_eyePosition + m_lateralShift * 0.5;
		m_targetCenter = m_eyePosition + m_orientationVector;
	}

	if (input.GetKey(SDL_SCANCODE_D)) {
		m_eyePosition = m_eyePosition - m_lateralShift * 0.5;
		m_targetCenter = m_eyePosition + m_orientationVector;
	}
}

void Camera::LookAt(glm::mat4 &modelview) {
	modelview = glm::lookAt(m_eyePosition, m_targetCenter, m_verticalAxis);
}
