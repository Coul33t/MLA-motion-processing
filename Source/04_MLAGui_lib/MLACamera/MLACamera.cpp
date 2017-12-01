#include "MLACamera.h"

Camera::Camera() : m_phi_angle(-10), m_theta_angle(-180), m_orientation_vector(), m_vertical_axis(0, 1, 0),
m_lateral_shift(), m_eye_position(glm::dvec3(CAMERA_POS_X, CAMERA_POS_Y, CAMERA_POS_Z)), m_target_center(glm::dvec3(CAMERA_ORI_X, CAMERA_ORI_Y, CAMERA_ORI_Z)) {
	//ctor
}

Camera::Camera(glm::dvec3 eye_position, glm::dvec3 target_center, glm::dvec3 vertical_axis) : m_phi_angle(-15.26), m_theta_angle(-135),
m_orientation_vector(), m_vertical_axis(vertical_axis), m_lateral_shift(), m_eye_position(eye_position), m_target_center(target_center) {
	//ctor
}

Camera::~Camera() {
	//dtor
}

void Camera::Orient(int x_rel, int y_rel) {
	
	// - because of the clockwise/anti-clockwise direction
	// *0.5 to attenuate fast movement
	m_phi_angle += -y_rel * 0.5;
	m_theta_angle += -x_rel * 0.5;

	if (m_phi_angle > 89.0)
		m_phi_angle = 89.0;

	else if (m_phi_angle < -89.0)
		m_phi_angle = -89.0;

	double phiAngleRad = glm::radians(m_phi_angle);
	double thetaAngleRad = glm::radians(m_theta_angle);

	if (m_vertical_axis.x == 1.0) {

		m_orientation_vector.x = glm::sin(phiAngleRad);
		m_orientation_vector.y = glm::cos(phiAngleRad) * glm::cos(thetaAngleRad);
		m_orientation_vector.z = glm::cos(phiAngleRad) * glm::sin(thetaAngleRad);
	}

	else if (m_vertical_axis.y == 1.0) {

		m_orientation_vector.x = glm::cos(phiAngleRad) * glm::sin(thetaAngleRad);
		m_orientation_vector.y = glm::sin(phiAngleRad);
		m_orientation_vector.z = glm::cos(phiAngleRad) * glm::cos(thetaAngleRad);
	}

	else {

		m_orientation_vector.x = glm::cos(phiAngleRad) * glm::cos(thetaAngleRad);
		m_orientation_vector.y = glm::cos(phiAngleRad) * glm::sin(thetaAngleRad);
		m_orientation_vector.z = glm::sin(phiAngleRad);
	}

	m_lateral_shift = glm::cross(m_vertical_axis, m_orientation_vector);
	m_lateral_shift = glm::normalize(m_lateral_shift);

	m_target_center = m_eye_position + m_orientation_vector;
}

void Camera::Move(Input const &input) {

	if (input.MouseMoving())
		Orient(input.GetXRel(), input.GetYRel());


	if (input.GetKey(SDL_SCANCODE_W)) {
		m_eye_position = m_eye_position + m_orientation_vector * 0.5;
		m_target_center = m_eye_position + m_orientation_vector;
	}

	if (input.GetKey(SDL_SCANCODE_S)) {
		m_eye_position = m_eye_position - m_orientation_vector * 0.5;
		m_target_center = m_eye_position + m_orientation_vector;
	}

	if (input.GetKey(SDL_SCANCODE_A)) {
		m_eye_position = m_eye_position + m_lateral_shift * 0.5;
		m_target_center = m_eye_position + m_orientation_vector;
	}

	if (input.GetKey(SDL_SCANCODE_D)) {
		m_eye_position = m_eye_position - m_lateral_shift * 0.5;
		m_target_center = m_eye_position + m_orientation_vector;
	}
}

void Camera::LookAt(glm::dmat4 &modelview) {
	modelview = glm::lookAt(m_eye_position, m_target_center, m_vertical_axis);
}

void Camera::PrintValues() {
	std::cout << "tC: " << m_target_center.x << " " << m_target_center.y << " " << m_target_center.z << "\r";
}