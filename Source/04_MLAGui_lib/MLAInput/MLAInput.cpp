#include "MLAInput.h"

Input::Input() : m_x(0), m_y(0), m_x_rel(0), m_y_rel(0), m_end(false) {
	for (int i = 0; i< SDL_NUM_SCANCODES; i++)
		m_keys[i] = false;

	for (int i = 0; i<8; i++)
		m_mouse_buttons[i] = false;
}

Input::~Input() {
	//dtor
}

int Input::GetX() const {
	return m_x;
}

int Input::GetY() const {
	return m_y;
}

int Input::GetXRel() const {
	return m_x_rel;
}

int Input::GetYRel() const {
	return m_y_rel;
}


void Input::EventUpdate() {

	// Because if we don't move, the relatives coordinates keep the old value
	// (meaning that the mouse is still moving, while it's actually not).
	m_x_rel = 0;
	m_y_rel = 0;

	while (SDL_PollEvent(&m_events)) {

		switch (m_events.type) {

		case SDL_WINDOWEVENT:
			if (m_events.window.event == SDL_WINDOWEVENT_CLOSE)
				m_end = true;
			break;

		case SDL_KEYDOWN:
			m_keys[m_events.key.keysym.scancode] = true;
			break;

		case SDL_KEYUP:
			m_keys[m_events.key.keysym.scancode] = false;
			break;

		case SDL_MOUSEBUTTONDOWN:
			m_mouse_buttons[m_events.button.button] = true;
			break;

		case SDL_MOUSEBUTTONUP:
			m_mouse_buttons[m_events.button.button] = false;
			break;

		case SDL_MOUSEMOTION:
			m_x = m_events.motion.x;
			m_y = m_events.motion.y;
			m_x_rel = m_events.motion.xrel;
			m_y_rel = m_events.motion.yrel;
			
			break;

		default:
			break;
		}
	}
}

bool Input::End() const {
	return m_end;
}

bool Input::GetKey(const SDL_Scancode key) const {
	return m_keys[key];
}

bool Input::GetMouseButton(const Uint8 button) const {
	return m_mouse_buttons[button];
}

bool Input::MouseMoving() const {
	if (m_x_rel == 0 && m_y_rel == 0)
		return false;
	else
		return true;
}

void Input::DisplayCursor(bool disp) const {

	if (disp)
		SDL_ShowCursor(SDL_ENABLE);
	else
		SDL_ShowCursor(SDL_DISABLE);
}

void Input::TrapMouse(bool trap) const {

	if (trap)
		SDL_SetRelativeMouseMode(SDL_TRUE);
	else
		SDL_SetRelativeMouseMode(SDL_FALSE);
}
